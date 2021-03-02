/*****************************************************************************
*
* Copyright 2018 NXP
* All Rights Reserved
*
*****************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

#ifndef __STANDALONE__
#include <signal.h>
#include <termios.h>
//#include <unistd.h>
#endif // #ifdef __STANDALONE__
#include <string.h>

#include "oal.h"
#include "vdb_log.h"
#include "sdi.hpp"

#ifdef __STANDALONE__
#include "frame_output_dcu.h"
#include "uartlinflex_driver.h"
#else // #ifdef __STANDALONE__
#include "frame_output_v234fb.h"
#endif // else from #ifdef __STANDALONE__

#include "isp_cam_sony.h"
#include "sony_dualexp_c.h"             // graph related defines
#include "vignetting.h"      // vignetting table
#include "iofiles_sony_dualexp.h"       // parameters from graph

//***************************************************************************
// constants
//***************************************************************************

#define CAM_SAVE_PGM

// ***** Graph Defines *******
#if(MIPI_PORT == 1)
#define CSI_IDX_CAM CSI_IDX_1
#else
#define CSI_IDX_CAM CSI_IDX_0
#endif

#define OUT_YUV422

#ifndef MULTITRIGGER
#define MULTITRIGGER 0 // one trigger procecesses 1<<MULTITRIGGER lines
#endif

#define EXPHIST_ENGINE 0
#if(EXPHIST_ENGINE == 0)
#define SEQ_EXP_HIST SEQ_IPUS0
#elif(EXPHIST_ENGINE == 1)
#define SEQ_EXP_HIST SEQ_IPUS1
#endif

#define EXPOSURE_ENGINE 1, EXPHIST_ENGINE
#define CHGAIN_ENGINE 1, EXPHIST_ENGINE

#define HDR_ENGINE 1, EXPHIST_ENGINE
#define HDR_ALPHA 0x78

#define VIGNETTING_ENGINE 1, 2
#define DENOISE_ENGINE 0, 2
#define OUTLUT_ENGINE 1, 3
#define OUTLUT_IPU IPUS_3

// Possible to set input resolution (must be supported by the DCU)
#define SCR_WIDTH 1280       ///< width of DDR buffer in pixels
#define SCR_HEIGHT 720       ///< height of DDR buffer in pixels
#define DDR_OUT_BUFFER_CNT 3 ///< number of DDR buffers per ISP stream
#ifdef OUT_GRAY8
#define CHNL_CNT io::IO_DATA_CH2
#else
#ifdef OUT_YUV422
#define CHNL_CNT io::IO_DATA_CH2
#else
#define CHNL_CNT io::IO_DATA_CH3
#endif
#endif

// ***** Run Control and Logging Defines *******

#define IPU_DUMP_STATE
#define DO_IPU_PROF
//#define DO_EXP_LOG
#define DO_CHGAIN_LOG
#define DO_BUFFER_LOG

#ifdef ISP_FRAME_RATE_MEASURE_NUM
#define LOOP_NUM ISP_FRAME_RATE_MEASURE_NUM
#else
#define LOOP_NUM 30
#endif

#ifdef DO_IPU_PROF
#define IPU_PROF_FRAMES ((LOOP_NUM < 10) ? LOOP_NUM : 10)
#endif

#define ALPHA_STEP (16)

#define EXPSKIP 1

#define DDR_BUFFER_CNT 3

//***************************************************************************
// macros
//***************************************************************************

#ifdef __STANDALONE__
extern "C" {
unsigned long get_uptime_microS(void);
}
#define GETTIME(time) (*(time) = get_uptime_microS())
#else // ifdef __STANDALONE__
#define GETTIME(time)                                                                                                  \
  {                                                                                                                    \
    struct timeval lTime;                                                                                              \
    gettimeofday(&lTime, 0);                                                                                           \
    *time = (lTime.tv_sec * 1000000 + lTime.tv_usec);                                                                  \
  }
#endif

//***************************************************************************
// types
//***************************************************************************
struct AppContext
{
  sdi_grabber* mpGrabber; ///< pointer to grabber instance
  sdi_FdmaIO*  mpFdma;    ///< pointer to fdma object
  SDI_Frame    mFrameIsp; ///< currently fetched SDI frame
  uint8_t      mExpCnt;   ///< counter of exposure updates
  uint32_t     mExpVal;   ///< latest exposure control result

  // exposure and channel gain control
  uint32_t      mExpRange;
  SEQ_IpuHist_t mIpuHist;
  uint16_t      mLAlphaVal;
  SONY_ChGain_t mChGainLevel;
  SONY_ChGain_t mChGainHist;

  // ** variables for keyboard interaction
  uint8_t  mKbKey;
  uint8_t  mKbMode;       ///< 0=off, 'k'= knee point, r=redgain, b=bluegain
  uint8_t  mKbLog;        ///< 0=off
  uint16_t mKbNoiseLevel; ///< noise level
  uint16_t mBlackLevel;
  uint8_t  mKbTestbar; ///< 1=on 0=off
  uint8_t  mRbAuto;
  uint32_t mKbI2c;     ///< IIC command 0xaaaavv  a=address v=value
  uint8_t  mKbOutLut;  ///< 0: linear 1: gamma 2: 12to8 3: 16to8
  uint8_t  mKbHDRMode; ///< 0: input tone mapping, 1: output tone mapping

  // ** event counters and flags **
  bool     mError;        ///< to signal ISP problems
  bool     mFrameDone;    ///< to signal frame was finished by ISP
  uint32_t mFrmCnt;       ///< number of frames grabbed by the app
  uint32_t mFrameDoneCnt; ///< number frame done events
  uint32_t mFrameDoneCntLast;

  unsigned long mTimeFrmStart; ///< frame interval start
};                             // struct AppContext

//*****************************************************************************
// static variables
//*****************************************************************************

static bool sStop = false; ///< to signal Ctrl+c from command line

//*****************************************************************************
// function declarations
//*****************************************************************************

#if defined(__STANDALONE__) && defined(__ARMV8)
extern "C" {
void flush_dcache_range(void*, unsigned int);
}
#endif

/************************************************************************/
/** Configures camera parameters and preprocessing.
  *
  * \param  arContext application context structure
  *
  * \return 0 if all ok, <0 otherwise
  ************************************************************************/
static int32_t CamConfig(AppContext& arContext);

/************************************************************************/
/** configure output LUT.
  *
  * \param  aLut 0: linear 1: gamma 2: 12to8bit 3: 16to8bit
  *
  * \return 0 if all ok, <0 otherwise
  ************************************************************************/
static int32_t SetOutLut(uint8_t aLut);

/************************************************************************/
/** Update exposure setup.
  *
  * \param  arContext application context structure
  *
  * \return 0 if all ok, <0 otherwise
  ************************************************************************/
static int32_t ExpControl(AppContext& arContext);

/************************************************************************/
/** Log performance and status.
  *
  * \param  arContext application context structure
  *
  * \return 0 if all ok, <0 otherwise
  ************************************************************************/
static int32_t LogPerformance(AppContext& arContext);

/************************************************************************/
/** If requested from command line, save requested image data.
  * 'S' == input image (raw camera data)
  * 's' == output image (ISP preprocessed data)
  *
  * \param  arContext application context structure
  *
  * \return 0 if all ok -1 otherwise
  ************************************************************************/
static int32_t FrameSave(AppContext& arContext);

/************************************************************************/
/** Prepare everything before executing the main functionality .
  *
  * \param arContext structure capturing the context of the application
  *
  * \return 0 if all ok, <0 otherwise
  ************************************************************************/
static int32_t Prepare(AppContext& arContext);

/************************************************************************/
/** Initial setup of application context.
  *
  * \param arContext structure capturing the context of the application
  ************************************************************************/
static void ContextInit(AppContext& arContext);

/************************************************************************/
/** Prepares required libraries.
  *
  * \param arContext structure capturing the context of the application
  *
  * \return 0 if all ok, != 0 otherwise
  ************************************************************************/
static int32_t LibsPrepare(AppContext& arContext);

/************************************************************************/
/** Prepares DDR buffers.
  *
  * \param arContext structure capturing the context of the application
  *
  * \return 0 if all ok, != 0 otherwise
  ************************************************************************/
static int32_t DdrBuffersPrepare(AppContext& arContext);

/************************************************************************/
/** Execute main functionality of the application.
  *
  * \param arContext structure capturing the context of the application
  *
  * \return 0 if all ok, <0 otherwise
  ************************************************************************/
static int32_t Run(AppContext& arContext);

/************************************************************************/
/** Cleanup all resources before application end.
  *
  * \param arContext structure capturing the context of the application
  *
  * \return 0 if all ok, <0 otherwise
  ************************************************************************/
static int32_t Cleanup(AppContext& arContext);

/************************************************************************/
/** User defined call-back function for Sequencer event handling.
  *
  * \param  aEventType type of the event received from ISP
  * \param  apUserVal  user defined value pointer
  ************************************************************************/
static void SeqEventHandler(uint32_t aEventType, void* apUserVal);

#ifndef __STANDALONE__

/************************************************************************/
/** SIGINT handler.
  *
  * \param  aSigNo
  ************************************************************************/
static void SigintHandler(int aSigNo);

/************************************************************************/
/** SIGINT handler.
  *
  * \param  aSigNo
  *
  * \return SEQ_LIB_SUCCESS if all ok
  *         SEQ_LIB_FAILURE if failed
  ************************************************************************/
static int32_t SigintSetup(void);

/************************************************************************/
/** Checks if key was pressed or not.
  *
  * \return 0 if no key pressed
  *         >0 if key pressed
  ************************************************************************/
static int KeyDown(void);

/************************************************************************/
/** Reads one character from standard input.
  *
  * If no key available in buffer function doesn't block and returns EOF.
  *
  * \return ASCI code of the read character
  *         EOF if no character available
  ************************************************************************/
static int GetCharNonBlock(void);

/************************************************************************/
/** Sets terminal to blocking character input.
  *
  ************************************************************************/
static void TermBlockSet(void);

/************************************************************************/
/** Sets terminal to nonblocking character input.
  *
  ************************************************************************/
static void TermNonBlockSet(void);

//***************************************************************************

#endif // #ifndef __STANDALONE__

/************************************************************************/
/** Reads character from command line.
 *
 * Hides differences between SA and Linux.
 *
 * \return character code being pressed
 ************************************************************************/
static char GetChar(void);

/************************************************************************/
/** Reads character from command line.
 *
 * \param  arContext application context structure
 *
 * \return 0 if all ok, <0 otherwise
 ************************************************************************/
static int32_t KbAction(AppContext& arContext);

//===========================================================================

int main(int argc, char** argv)
{
  int lRet = 0;

  AppContext lContext;

  //*** process command line parameters ***
  const char helpMsg_str[] =
      "\n**************************************************************\n"
      "** Sony imx224 default demo\n"
      "** Description:\n"
      "**  o Sony imx224 camera (on MipiCsi_0) expected as image input.\n"
      "**  o ISP generates histogram for AEC, does vignetting correction,\n"
      "**    debayering, converts pixel format from RGB to YUV420, does\n"
      "**    denoising on Y plane and HDR compresion from 2 exposures.\n"
      "**  o Resulting image is displayed live using DCU.\n"
      "**\n"
      "** Usage:\n"
      "**  o no cmd line parameters available.\n"
      "**  o runtime control using nonblocking keyboard input.\n"
      "**    press 'h' for available commands listing.\n"
      "**\n"
      "**************************************************************\n\n";
  int idxHelp = COMMON_HelpMessage(argc, argv, helpMsg_str);
  if(idxHelp < 0)
  { // print help message even if no help option is provided by the user
    printf("%s", helpMsg_str);
  }

#ifndef __STANDALONE__
  fflush(stdout);
  sleep(1);
#endif // ifndef __STANDALONE__

  if(Prepare(lContext) == 0)
  {
    if(Run(lContext) != 0)
    {
      printf("Demo execution failed.\n");
      lRet = -1;
    } // if Run() failed
  }   // if Prepare() ok
  else
  {
    printf("Demo failed in preparation phase.\n");
    lRet = -1;
  } // else from if Prepare() ok

  if(Cleanup(lContext) != 0)
  {
    printf("Demo failed in cleanup phase.\n");
    lRet = -1;
  } // if cleanup failed

  return lRet;
} // main

//***************************************************************************

static int32_t Prepare(AppContext& arContext)
{
  // init app context
  ContextInit(arContext);

  // enable LIBS
  if(LibsPrepare(arContext) != 0)
  {
    printf("Failed to prepare libraries.\n");
    return -1;
  } // if failed to configure decoder
  // enable OAL

  if(DdrBuffersPrepare(arContext) != 0)
  {
    printf("Failed to prepare DDR buffers.\n");
    return -1;
  } // if fialed to prepare DDR buffers

  // *** prestart grabber ***
  if(arContext.mpGrabber->PreStart() != LIB_SUCCESS)
  {
    printf("Failed to prestart the grabber.\n");
    return -1;
  } // if PreStart() failed

  // *** configure camera ***
  if(CamConfig(arContext) != 0)
  {
    printf("Failed to prestart the grabber.\n");
    return -1;
  } // if CamConfig() failed

  // *** install sequencer event callback ***
  if(arContext.mpGrabber->SeqEventCallBackInstall(&SeqEventHandler, &arContext) != LIB_SUCCESS)
  {
    printf("Failed to install Sequencer event callback.\n");
    return -1;
  } // if callback setup failed

  // *** initial keyboard commands ***
  arContext.mKbKey = 'Z';
  if(KbAction(arContext) != 0) // set blacklevel
  {
    printf("Keyboard control failed.\n");
    return -1;
  }
  arContext.mKbKey = 'N';
  if(KbAction(arContext) != 0) // set denoising
  {
    printf("Keyboard control failed.\n");
    return -1;
  }
  arContext.mKbKey = 'h';
  if(KbAction(arContext) != 0) // print keyboard help
  {
    printf("Keyboard control failed.\n");
    return -1;
  }

  return 0;
} // Prepare()

//***************************************************************************

static void ContextInit(AppContext& arContext)
{
  arContext.mpGrabber = NULL;
  arContext.mpFdma    = NULL;

  // ** event counters and flags **
  arContext.mError            = false;
  arContext.mFrameDone        = false;
  arContext.mFrameDoneCnt     = 0;
  arContext.mFrameDoneCntLast = 0;
  arContext.mFrmCnt           = 0;
  arContext.mTimeFrmStart     = 0;

  // exposure and channel gain control
  arContext.mExpRange = 0;
  arContext.mExpCnt   = 0;
  arContext.mExpVal   = 0;
#if defined(OUTLUT_16TO8)
  arContext.mLAlphaVal = 4;
#else
  arContext.mLAlphaVal = 128;
#endif

  // keyboard interaction
  arContext.mKbKey        = 0;
  arContext.mKbMode       = 0;
  arContext.mKbLog        = 0;
  arContext.mKbNoiseLevel = 0x300;
  arContext.mBlackLevel   = 232;
  arContext.mKbTestbar    = 0;
  arContext.mRbAuto       = 1;
  arContext.mKbI2c        = 0;
  arContext.mKbOutLut     = 0;
  arContext.mKbHDRMode    = 3;
} // ContextInit()

//***************************************************************************

static int32_t LibsPrepare(AppContext& arContext)
{
  // *** Initialize SDI ***
  if(sdi::Initialize(0) != LIB_SUCCESS)
  {
    printf("Failed to initialzie SDI.\n");
    return -1;
  } // if failed to initialize SDI

  // create grabber
  arContext.mpGrabber = new(sdi_grabber);
  if(arContext.mpGrabber == NULL)
  {
    printf("Failed to create sdi grabber.\n");
    return -1;
  } // if failed to create grabber

  if(arContext.mpGrabber->ProcessSet(gpGraph_sony_dualexp, &gGraphMetadata_sony_dualexp,
		  kmem_srec, sequencer_srec ) != LIB_SUCCESS)
  {
    printf("Failed to set ISP graph to grabber.\n");
    return -1;
  } // if ISP graph not set

  // get IOs
  arContext.mpFdma = (sdi_FdmaIO*)arContext.mpGrabber->IoGet(SEQ_OTHRIX_FDMA);
  if(arContext.mpFdma == NULL)
  {
    printf("Failed to get FDMA object.\n");
    return -1;
  } // if no FDMA object

  return 0;
} // LibsPrepare(AppContext &arContext)

//***************************************************************************

static int32_t DdrBuffersPrepare(AppContext& arContext)
{
  // *** UYVY full buffer array ***
  // modify DDR frame geometry to fit display output
  SDI_ImageDescriptor lFrmDesc;
  lFrmDesc = SDI_ImageDescriptor(SCR_WIDTH, NUM_OUT_LINES >> MULTITRIGGER, YUV422Stream_UYVY);

  if(arContext.mpFdma->DdrBufferDescSet(FDMA_IX_ISP_UYVY_OUTPUT_SONY_DUALEXP,
                                        lFrmDesc) != LIB_SUCCESS)
  {
    printf("Failed to set image descriptor.\n");
    return -1;
  } // if frame descriptor setup failed

  // allocate DDR buffers
  if(arContext.mpFdma->DdrBuffersAlloc(DDR_BUFFER_CNT) != LIB_SUCCESS)
  {
    printf("Failed to allocate DDR buffers.\n");
    return -1;
  } // if ddr buffers not allocated

  return 0;
} // DdrBuffersPrepare(AppContext &arContext)

//***************************************************************************

static int32_t Run(AppContext& arContext)
{
//*** Init DCU Output ***
#ifdef __STANDALONE__
  io::FrameOutputDCU lDcuOutput(SCR_WIDTH, SCR_HEIGHT, io::IO_DATA_DEPTH_08, CHNL_CNT, DCU_BPP_YCbCr422);
#else  // #ifdef __STANDALONE__
  io::FrameOutputV234Fb lDcuOutput(SCR_WIDTH, SCR_HEIGHT, io::IO_DATA_DEPTH_08, io::IO_DATA_CH3, DCU_BPP_YCbCr422);

  // setup Ctrl+C handler
  if(SigintSetup() != SEQ_LIB_SUCCESS)
  {
    VDB_LOG_ERROR("Failed to register Ctrl+C signal handler.");
    return -1;
  } // if SigintSetup() failed
  printf("Press Ctrl+C to terminate the demo.\n");

  // set terminal to nonblock input ***
  TermNonBlockSet();
#endif // else from #ifdef __STANDALONE__

  // *** start grabbing ***
  GETTIME(&arContext.mTimeFrmStart);
  if(arContext.mpGrabber->Start() != LIB_SUCCESS)
  {
    printf("Failed to start the grabber.\n");
    return -1;
  } // if Start() failed

  SDI_Frame lFrame;
  // *** grabbing/processing loop ***
  for(;;)
  {
    for(uint32_t lLoop = 0; lLoop < LOOP_NUM; lLoop++)
    {
      arContext.mFrameIsp = arContext.mpGrabber->FramePop(FDMA_IX_ISP_UYVY_OUTPUT_SONY_DUALEXP);

      if(arContext.mFrameIsp.mUMat.empty())
      {
        printf("Failed to pop ISP output frame.\n");
        arContext.mError = true;
        return -1;
      } // if pop failed

      lDcuOutput.PutFrame(arContext.mFrameIsp.mUMat);

      // *** get keyboard input ***
      arContext.mKbKey = GetChar();
      //printf("char %c\n", gKbKey);
      if(arContext.mKbKey)
      {
        if(KbAction(arContext) != 0)
        {
          printf("Keyboard control failed.\n");
          return -1;
        }
        FrameSave(arContext);
      } // if key pressed
      arContext.mKbKey = 0;

      // *** do exposure control ***
      if(ExpControl(arContext) != 0)
      {
        printf("Failed to run exposure control.\n");
        return -1;
      } // if CamConfig() failed

      // *** return frame buffer to the ISP pipeline ***
      if(arContext.mpGrabber->FramePush(arContext.mFrameIsp) != LIB_SUCCESS)
      {
        printf("Failed to push back the frame.\n");
        arContext.mError = true;
        return -1;
      } // if push failed

      arContext.mFrmCnt++;

      // *** if stop requested ***
      if(sStop)
      {
        return 0; // break if Ctrl+C pressed
      }           // if Ctrl+C
    }             // if new frame read

    // *** if stop requested ***
    if(sStop)
    {
      return 0; // break if Ctrl+C pressed
    }           // if Ctrl+C

    // *** log output ****
    LogPerformance(arContext);
  } // for ever

  return 0;
} // Run()

//***************************************************************************

static int32_t Cleanup(AppContext& arContext)
{
  int32_t lRet = 0;

  if(arContext.mpGrabber != NULL)
  {
    if(arContext.mpGrabber->Stop())
    {
      printf("Failed to stop the grabber.\n");
      lRet = -1;
    } // if grabber stop failed

    if(arContext.mpGrabber->Release())
    {
      printf("Failed to release grabber resources.\n");
      lRet = -1;
    } // if grabber resources not released

    delete(arContext.mpGrabber);
    arContext.mpGrabber = NULL;
  } // if grabber exists

#ifdef __STANDALONE__
  for(;;)
    ; // *** don't return ***
#else
  // *** set terminal back to block mode ***
  TermBlockSet();
#endif // #ifdef __STANDALONE__

  if(sdi::Close(0) != LIB_SUCCESS)
  {
    printf("Failed to terminate use of SDI.\n");
    lRet = -1;
  } // if SDI use termination failed

  return lRet;
} // Cleanup()

//***************************************************************************

static int32_t CamConfig(AppContext& arContext)
{
  // *** exposure kernel ***
  // black level correction
  if(seq_setReg(CHGAIN_ENGINE, 0, 0x70,
                arContext.mBlackLevel << 4) != SEQ_LIB_SUCCESS) // GPR0: R
  {
    return -1;
  } // if seq_setReg() failed
  if(seq_setReg(CHGAIN_ENGINE, 0, 0x71,
                arContext.mBlackLevel << 4) != SEQ_LIB_SUCCESS) // GPR1: GR
  {
    return -1;
  } // if seq_setReg() failed
  if(seq_setReg(CHGAIN_ENGINE, 0, 0x72,
                arContext.mBlackLevel << 4) != SEQ_LIB_SUCCESS) // GPR2: GB
  {
    return -1;
  } // if seq_setReg() failed
  if(seq_setReg(CHGAIN_ENGINE, 0, 0x73,
                arContext.mBlackLevel << 4) != SEQ_LIB_SUCCESS) // GPR3: B
  {
    return -1;
  } // if seq_setReg() failed

  // channel gain
  arContext.mChGainLevel.mLevel[0] = 400;
  arContext.mChGainLevel.mLevel[1] = 256;
  arContext.mChGainLevel.mLevel[2] = 256;
  arContext.mChGainLevel.mLevel[3] = 450;
  if(seq_setReg(CHGAIN_ENGINE, 0, 0x74,
                arContext.mChGainLevel.mLevel[0]) != SEQ_LIB_SUCCESS) // GPR4: R
  {
    return -1;
  } // if seq_setReg() failed
  if(seq_setReg(CHGAIN_ENGINE, 0, 0x75,
                arContext.mChGainLevel.mLevel[1]) != SEQ_LIB_SUCCESS) // GPR5: GR
  {
    return -1;
  } // if seq_setReg() failed
  if(seq_setReg(CHGAIN_ENGINE, 0, 0x76,
                arContext.mChGainLevel.mLevel[2]) != SEQ_LIB_SUCCESS) // GPR6: GB
  {
    return -1;
  } // if seq_setReg() failed
  if(seq_setReg(CHGAIN_ENGINE, 0, 0x77,
                arContext.mChGainLevel.mLevel[3]) != SEQ_LIB_SUCCESS) // GPR7: B
  {
    return -1;
  } // if seq_setReg() failed

  if(!arContext.mKbHDRMode)
  { // tone mapping at input
    if(seq_setReg(HDR_ENGINE, 0, HDR_ALPHA, arContext.mLAlphaVal) != SEQ_LIB_SUCCESS)
    {
      return -1;
    } // if seq_setReg() failed
    if(seq_setReg(HDR_ENGINE, 0, HDR_ALPHA + 1, 256 - arContext.mLAlphaVal) != SEQ_LIB_SUCCESS)
    {
      return -1;
    }                        // if seq_setReg() failed
    arContext.mKbOutLut = 1; //gamma
  }
  else
  { // tone mapping at output
    if(seq_setReg(HDR_ENGINE, 0, HDR_ALPHA, 16) != SEQ_LIB_SUCCESS)
    {
      return -1;
    } // if seq_setReg() failed
    if(seq_setReg(HDR_ENGINE, 0, HDR_ALPHA + 1, 240) != SEQ_LIB_SUCCESS)
    {
      return -1;
    }                        // if seq_setReg() failed
    arContext.mKbOutLut = 2; //gamma
  }

#ifdef SEPARATE_EXP_HDR
  // *** tonemap kernel ***
  if(seq_setReg(
       HDR_ENGINE,
       1,
       0x120,
       0x00350000 |
       (256 - arContext.mLAlphaVal) != SEQ_LIB_SUCCESS)//CH2_CFG_INA
  {
    return -1;
  } // if seq_setReg() failed
#endif

  // *** vignetting kernel ***
#define VIGNETTING_LUT_OFFSET 0
  if(seq_setReg(
       VIGNETTING_ENGINE,
       0,
       0x70,
       644) != SEQ_LIB_SUCCESS)   // GPR0 (IPUS)
  {
    return -1;
  } // if seq_setReg() failed
  if(seq_setReg(
       VIGNETTING_ENGINE,
       0,
       0x71,
       364) != SEQ_LIB_SUCCESS)   // GPR1 (IPUS)
  {
    return -1;
  } // if seq_setReg() failed
  if(seq_setReg(
       VIGNETTING_ENGINE,
       0,
       0x72,
       115) != SEQ_LIB_SUCCESS)    // GPR2 (IPUS)
  {
    return -1;
  } // if seq_setReg() failed

  if(seq_setReg(
       VIGNETTING_ENGINE,
       0,
       0x73,
       1024) != SEQ_LIB_SUCCESS)   // GPR0 (IPUS)
  {
    return -1;
  } // if seq_setReg() failed
  if(seq_setReg(
       VIGNETTING_ENGINE,
       1,
       0x78,
       ((VIGNETTING_LUT_OFFSET)>>1)) == SEQ_LIB_SUCCESS) // LUTA (IPUS)
  {
    uint16_t lutcnt = 0;
    while(lutcnt < 1024)
    {
      uint32_t val;
      val = vignetting[lutcnt++];
      val |= ((long)vignetting[lutcnt++] << 16);
      if(seq_setReg(VIGNETTING_ENGINE, 1, 0x7C,
                    val) != SEQ_LIB_SUCCESS) // LUTD (IPUS)
      {
        return -1;
      } // if seq_setReg() succeeded
    }   // while lutcnt < 1024
  }// if seq_setReg() succeeded
  else
  {
    return -1;
  } // else if seq_setReg() failed

  // *** denoise kernel ***
  if(seq_setReg(
       DENOISE_ENGINE,
       0,
       0x56,
       arContext.mKbNoiseLevel) != SEQ_LIB_SUCCESS) // GPR6 (IPUV): Noise level
  {
    return -1;
  } // if seq_setReg() failed

  // *** Output LUT
  if(SetOutLut(arContext.mKbOutLut) != 0)
  {
    return -1;
  }

  // *** Camera Configuration ***
  // modify camera geometry !!! setup before setting up expsoure control
  SONY_Geometry_t lGeo;
  lGeo.mCsiIdx  = CSI_IDX_CAM;
  if(SONY_GeometryGet(lGeo) != CAM_LIB_SUCCESS)
  {
    printf("Failed to get camera geometry.\n");
    return -1;
  } // if SONY_GeometryGet failed

  lGeo.mFps     = 30;
  lGeo.mExpMode = SONY_DUAL_EXP;
  if(SONY_GeometrySet(lGeo) != CAM_LIB_SUCCESS)
  {
    printf("Failed to set camera geometry.\n");
    return -1;
  } // if SONY_GeometrySet failed
  // *** enable IPU histogram readout ***
  memset(&arContext.mIpuHist, 0, sizeof(SEQ_IpuHist_t));
  arContext.mIpuHist.mIpuIdx = SEQ_EXP_HIST;
  SEQ_HistogramEnable(SEQ_EXP_HIST);

  // *** setup initial exposure ***
  SONY_Exposure_t lExp;
  lExp.mCsiIdx = CSI_IDX_CAM;

  if(SONY_ExposureGet(lExp) != CAM_LIB_SUCCESS)
  {
    printf("Failed to get camera exposure config.\n");
    return -1;
  } // if SONY_ExposureGet failed
  lExp.mAnalogGain     = 0;
  lExp.mConversionGain = 1;
  lExp.mExposureLines  = 500<<1;
  lExp.mConfigFlags    = SONY_EXP_EL | SONY_EXP_CG | SONY_EXP_AG;
  if(SONY_ExposureSet(lExp) != CAM_LIB_SUCCESS)
  {
    printf("Failed to set camera exposure config.\n");
    return -1;
  } // if SONY_ExposureSet failed

  if(SONY_GeometryGet(lGeo) != CAM_LIB_SUCCESS)
  {
    printf("Failed to get camera geometry.\n");
    return -1;
  } // if SONY_GeometryGet failed
  arContext.mExpRange = lGeo.mVmax;
  arContext.mExpRange<<=1;  // for dual exposure

  printf("VMAX = %4d, Lines = %3d, OB = %2d\n",
          lGeo.mVmax,
          lGeo.mHeight,
          lGeo.mOb);

  return 0;
} // CamConfig();

//***************************************************************************

static int32_t SetOutLut(uint8_t aLut)

{
  uint32_t val0;
  uint32_t val1;
  uint16_t lutcnt = 0;
#if defined(__STANDALONE__)
  volatile struct IPUS_tag* lpOutLut = &(OUTLUT_IPU);
#endif
#define OUTLUT_OFFSET 0
#define OUTLUT_SIZE 4096

  // GPR1 (IPUS)
  if(seq_setReg(OUTLUT_ENGINE, 0, 0x71, OUTLUT_OFFSET) != SEQ_LIB_SUCCESS)
  {
    return -1;
  } // if seq_setReg() failed
    // LUTA
#if defined(__STANDALONE__)
  lpOutLut->HOST_LUTA.R = OUTLUT_OFFSET >> 1;
#else
  if(seq_setReg(OUTLUT_ENGINE, 1, 0x78, ((OUTLUT_OFFSET) >> 1)) != SEQ_LIB_SUCCESS)
  {
    return -1;
  } // if seq_setReg() failed
#endif

  if(aLut == 0)
  {
    printf("output lut: copy\n");
    while(lutcnt < OUTLUT_SIZE)
    {
      val0 = 512; //((lutcnt++)<<4);  // including 15 to 16 bit conversion
      lutcnt++;
      val1 = 512; //((lutcnt++)<<4);
      lutcnt++;
#if defined(__STANDALONE__)
      lpOutLut->HOST_LUTD.R = val0 | (val1 << 16);
#else
      if(seq_setReg(OUTLUT_ENGINE, 1, 0x7C, val0 | (val1 << 16)) != SEQ_LIB_SUCCESS)

      {
        return -1;
      } // if seq_setReg() failed
#endif
    } // while lutcnt < OUTLUT_SIZE
  }   // if lut == 0
  else if(aLut == 1)
  {
#define GAMMA 0.8
#define GAMMA_SCALE (0xFFF0)
    printf("output lut: gamma %f\n", GAMMA);
    while(lutcnt < OUTLUT_SIZE)
    {
      if(!lutcnt)
      {
        val0 = 0;
      }
      else
      {
        val0 = (uint32_t)(256.0 * // 8.8 factors
                          ((float)GAMMA_SCALE) *
                          pow(((float)lutcnt) / ((float)(OUTLUT_SIZE - 1)),
                              GAMMA) /
                          (float(lutcnt << 3)) // 15 bit input values
        );
      }
      lutcnt++;
      val1 = (uint32_t)(256.0 * // 8.8 factors
                        ((float)GAMMA_SCALE) *
                        pow(((float)lutcnt) / ((float)(OUTLUT_SIZE - 1)),
                            GAMMA) /
                        (float(lutcnt << 3)) // 15 bit input values
      );
      lutcnt++;
      if(val0 > 0xffff)
      {
        val0 = 0xffff;
      }
      if(val1 > 0xffff)
      {
        val1 = 0xffff;
      }
#if defined(__STANDALONE__)
      lpOutLut->HOST_LUTD.R = val0 | (val1 << 16);
#else
      if(seq_setReg(OUTLUT_ENGINE, 1, 0x7C, val0 | (val1 << 16)) != SEQ_LIB_SUCCESS)
      {
        return -1;
      } // if seq_setReg() failed
#endif
    } // while lutcnt < OUTLUT_SIZE
  }
  else if(aLut == 2)
  {
#define LOG_A_12TO8 15453.32
#define LOG_B_12TO8 0.016704
    printf("output lut: log 12to8\n");
    while(lutcnt < OUTLUT_SIZE)
    {
      if(!lutcnt)
      {
        val0 = 0;
      }
      else
      {
        val0 = (uint32_t)(256.0 * LOG_A_12TO8 * logf((lutcnt)*LOG_B_12TO8 + 1) / ((float)(lutcnt << 3)));
      }
      lutcnt++;
      val1 = (uint32_t)(256.0 * LOG_A_12TO8 * logf((lutcnt)*LOG_B_12TO8 + 1) / ((float)(lutcnt << 3)));
      lutcnt++;
      if(val0 > 0xffff)
      {
        val0 = 0xffff;
      }
      if(val1 > 0xffff)
      {
        val1 = 0xffff;
      }
#if defined(__STANDALONE__)
      lpOutLut->HOST_LUTD.R = val0 | (val1 << 16);
#else
      if(seq_setReg(OUTLUT_ENGINE, 1, 0x7C, val0 | (val1 << 16)) != SEQ_LIB_SUCCESS)
      {
        return -1;
      } // if seq_setReg() failed
#endif
    } // while lutcnt < OUTLUT_SIZE
  }
  else if(aLut == 3)
  {
#define LOG_A_16TO8 8332.00
#define LOG_B_16TO8 0.634928
    uint16_t lutcnt = 0;
    printf("output lut: log 16to8\n");
    while(lutcnt < OUTLUT_SIZE)
    {
      if(!lutcnt)
      {
        val0 = 0x0;
      }
      else
      {
        val0 = (uint32_t)(256.0 * LOG_A_16TO8 * logf((lutcnt)*LOG_B_16TO8 + 1) / ((float)(lutcnt << 3)));
      }
      lutcnt++;
      val1 = (uint32_t)(256.0 * LOG_A_16TO8 * logf((lutcnt)*LOG_B_16TO8 + 1) / ((float)(lutcnt << 3)));
      lutcnt++;

      if(val0 > 0xffff)
      {
        val0 = 0xffff;
      }
      if(val1 > 0xffff)
      {
        val1 = 0xffff;
      }
#if defined(__STANDALONE__)
      lpOutLut->HOST_LUTD.R = val0 | (val1 << 16);
#else
      if(seq_setReg(OUTLUT_ENGINE, 1, 0x7C, val0 | (val1 << 16)) != SEQ_LIB_SUCCESS)
      {
        return -1;
      } // if seq_setReg() failed
#endif
    } // while lutcnt < OUTLUT_SIZE
  }
  return 0;
} // SetOutLut()

//***************************************************************************

static int32_t ExpControl(AppContext& arContext)
{
  // *** perform white balancing ***
  if(!arContext.mExpCnt && arContext.mFrmCnt && arContext.mRbAuto)
  {
    SONY_ChGainMeasure(&arContext.mIpuHist,
                       // # of pixels to be measured in histogram
                       // higher value makes brighter
                       (1288 * 728 / 4) * 50 / 100, &arContext.mChGainHist);
#define TAU (5)
    // implement an IIR filter with time constant 2^TAU frames
    arContext.mChGainLevel.mLevel[0] +=
        (((((int32_t)256) * arContext.mChGainHist.mLevel[1]) / arContext.mChGainHist.mLevel[0]) -
         arContext.mChGainLevel.mLevel[0] + // reduce previsous sum
         (1 << (TAU - 1)))                  // for rounding
        >> TAU;
    if(arContext.mChGainLevel.mLevel[0] < 256)
    {
      arContext.mChGainLevel.mLevel[0] = 256;
    }
    if(arContext.mChGainLevel.mLevel[0] > 640)
    {
      arContext.mChGainLevel.mLevel[0] = 640;
    }

    arContext.mChGainLevel.mLevel[3] +=
        (((((int32_t)256) * arContext.mChGainHist.mLevel[1]) / arContext.mChGainHist.mLevel[3]) -
         arContext.mChGainLevel.mLevel[3] + // reduce previsous sum
         (1 << (TAU - 1)))                  // for rounding
        >> TAU;
    if(arContext.mChGainLevel.mLevel[3] < 256)
    {
      arContext.mChGainLevel.mLevel[3] = 256;
    }
    if(arContext.mChGainLevel.mLevel[3] > 640)
    {
      arContext.mChGainLevel.mLevel[3] = 640;
    }

    if(seq_setReg(CHGAIN_ENGINE, 0, 0x74,
                  arContext.mChGainLevel.mLevel[0]) != SEQ_LIB_SUCCESS) // R
    {
      return -1;
    }
    if(seq_setReg(CHGAIN_ENGINE, 0, 0x77,
                  arContext.mChGainLevel.mLevel[3]) != SEQ_LIB_SUCCESS) // B
    {
      return -1;
    }

    if(arContext.mKbLog == 4)
    {
      printf("ChGains = GainR %3d/256, GainB=%3d/256", arContext.mChGainLevel.mLevel[0],
             arContext.mChGainLevel.mLevel[3]);
      printf(" (%d,%d,%d,%d,max %d, min %d)", arContext.mChGainHist.mLevel[0], arContext.mChGainHist.mLevel[1],
             arContext.mChGainHist.mLevel[2], arContext.mChGainHist.mLevel[3], arContext.mChGainHist.mMax,
             arContext.mChGainHist.mMin);
      printf("\n");
    }
  }

  // *** perform exposure control ***
  if((arContext.mExpCnt == EXPSKIP) && (!arContext.mKbTestbar))
  {
    // do expsoure control
    if(SEQ_HistogramGet(&arContext.mIpuHist) != SEQ_LIB_SUCCESS)
    {
      return -1;
    }
    SONY_ExposureControlOver(&arContext.mIpuHist,
                             // # of over exposed pixels in GR
                             // higher value makes brighter
                             (1288 * 728 / 4) * 65 / 100,
                             // bin from which on value is
                             // "over exposed"
                             // higher value makes brighter
                             16, &arContext.mExpVal);

    if(SONY_ExposureSet(CSI_IDX_CAM, arContext.mExpVal) != CAM_LIB_SUCCESS)
    {
      return -1;
    }
    if(arContext.mKbLog == 3)
    {
      SONY_Exposure_t lExp;

      lExp.mCsiIdx = CSI_IDX_CAM;
      if(SONY_ExposureGet(lExp) != CAM_LIB_SUCCESS)
      {
        return -1;
      }

      printf("%d ", arContext.mExpVal);
      printf("Exposure Lines = %3d:%3d CG= %d AG=%2ddB Knee=%3d\n", lExp.mExposureLines, lExp.mExposureLinesShort,
             lExp.mConversionGain, lExp.mAnalogGain / 10, arContext.mLAlphaVal);
    }
    arContext.mExpCnt = 0;
  } // if EXPSKIP matched
  else
  {
    arContext.mExpCnt++;
  } // else from if EXPSKIP matched
  return 0;
} // ExpControl()

//***************************************************************************

static int32_t LogPerformance(AppContext& arContext)
{
  unsigned long lTimeEnd = 0, lTimeDiff = 0;

  GETTIME(&lTimeEnd);
  lTimeDiff               = lTimeEnd - arContext.mTimeFrmStart;
  arContext.mTimeFrmStart = lTimeEnd;

  uint32_t lFrameDoneDiff     = arContext.mFrameDoneCnt - arContext.mFrameDoneCntLast;
  arContext.mFrameDoneCntLast = arContext.mFrameDoneCnt;

  if(arContext.mKbLog != 0)
  {
    if(arContext.mKbLog == 1)
    {
      printf("Frame: %7d (Isp.GetFrame:%d)\n", arContext.mFrameDoneCnt, arContext.mFrmCnt);

      printf("%d frames took %lu usec (%5.2ffps)\n", lFrameDoneDiff, lTimeDiff,
             ((double)lFrameDoneDiff) * 1000000.0 / ((float)lTimeDiff));

    } // kb_log < 3
    if(arContext.mKbLog == 2)
    {
      printf("Frame: %7d (Isp.GetFrame:%d)\n", arContext.mFrameDoneCnt, arContext.mFrmCnt);
      ISP_DumpPerfCounters(IPU_PROF_FRAMES);
      if(SEQ_IpuPerformanceInfoRequest(IPU_PROF_FRAMES) != SEQ_LIB_SUCCESS)
      {
        return -1;
      }
    } // (if arContext.mKbLog ==2)
#ifdef DO_BUFFER_LOG
    if(arContext.mKbLog == 5)
    {
      SEQ_Head_Ptr_t* lpFetchedGraph = NULL;
      SEQ_Buf_t**     lppFetchedBufferList;

      lppFetchedBufferList = arContext.mpGrabber->GraphFetch(&lpFetchedGraph);
      ISP_PrintGraphStat(lppFetchedBufferList, gGraphMetadata_sony_dualexp);
    } // (if arContext.mKbLog == 4)
#endif
  } // (if arContext.mKbLog !=0)
  return 0;
} // LogPerformace()

//****************************************************************************

static void SeqEventHandler(uint32_t aEventType, void* apUserVal)
{
  AppContext* lpContext = (AppContext*)apUserVal;
  if(apUserVal == NULL)
  {
    printf("SeqEvent callbec parameter is NULL.\n");
    return;
  } // if param is NULL

  switch(aEventType)
  {
    case SEQ_MSG_TYPE_FRAMEDONE:
    {
      lpContext->mFrameDoneCnt++;
    } // case SEQ_MSG_TYPE_FRAMEDONE:
    break;
    case SEQ_MSG_TYPE_OTHERERROR:   // fallthrough
    case SEQ_MSG_TYPE_FDMAERROR:    // fallthrough
    case SEQ_MSG_TYPE_FDMACRCERROR: // fallthrough
    {
      char lString[SEQ_PRAM_AUX_DATA_SIZE] = {0};

      lpContext->mError = true;
      sStop             = true;
      if(aEventType != SEQ_MSG_TYPE_OTHERERROR)
      {
        VDB_LOG_WARNING("Unknown event type (%u) received.\n", aEventType);
      }
      VDB_LOG_NOTE("*** Sequencer error text:\n");
      if(SEQ_MsgAuxDataGet(lString, SEQ_PRAM_AUX_DATA_SIZE) != SEQ_LIB_SUCCESS)
      {
        VDB_LOG_NOTE("Failed to get the auxiliary data from PRAM.\n");
      } // if failed to read the error message
      printf("  %s\n", lString);
    } // default
    break;
    default:
      break;
  } // switch(aEventType)

  return;
} // SeqEventHandler(uint32_t aEventType)

  //***************************************************************************

#ifndef __STANDALONE__
static void SigintHandler(int /*aSigNo*/)
{
  sStop = true;
} // SigintHandler()

//***************************************************************************

static int32_t SigintSetup()
{
  int32_t lRet = SEQ_LIB_SUCCESS;

  // prepare internal signal handler
  struct sigaction lSa;
  memset(&lSa, 0, sizeof(lSa));
  lSa.sa_handler = SigintHandler;

  if(sigaction(SIGINT, &lSa, NULL) != 0)
  {
    VDB_LOG_ERROR("Failed to register signal handler.\n");
    lRet = SEQ_LIB_FAILURE;
  } // if signal not registered

  return lRet;
} // SigintSetup()

//***************************************************************************

static int KeyDown()
{
  struct timeval lTv;
  fd_set         lFdSet;
  lTv.tv_sec  = 0;
  lTv.tv_usec = 50;
  FD_ZERO(&lFdSet);
  FD_SET(STDIN_FILENO, &lFdSet);
  select(STDIN_FILENO + 1, &lFdSet, NULL, NULL, &lTv);
  return FD_ISSET(STDIN_FILENO, &lFdSet);
} // KeyDown()

//***************************************************************************

static int GetCharNonBlock()
{
  int lChar = EOF;

  usleep(1);
  if(KeyDown())
  {
    lChar = fgetc(stdin);
  } // if Key pressed

  return lChar;
} // KeyDown()

//***************************************************************************

static void TermNonBlockSet()
{
  struct termios lTermState;

  // get current state of the terminal
  tcgetattr(STDIN_FILENO, &lTermState);

  // disable canonical mode
  lTermState.c_lflag &= ~ICANON;
  // set minimum number of input characters to read
  lTermState.c_cc[VMIN] = 1;

  // set the terminal attributes
  tcsetattr(STDIN_FILENO, TCSANOW, &lTermState);
} // TermNonBlockSet()

//***************************************************************************

static void TermBlockSet()
{
  struct termios lTermState;

  // get current state of the terminal
  tcgetattr(STDIN_FILENO, &lTermState);

  //enable canonical mode
  lTermState.c_lflag |= ICANON;

  // set the terminal attributes
  tcsetattr(STDIN_FILENO, TCSANOW, &lTermState);
} // TermBlockSet()

#endif // #ifndef __STANDALONE__

//***************************************************************************

static int32_t FrameSave(AppContext& arContext)
{
  int32_t lRet = 0;
#ifndef __STANDALONE__
  if(arContext.mKbMode == 's')
  {
    if(!arContext.mFrameIsp.mUMat.empty())
    {
      vsdk::Mat frame_map  = arContext.mFrameIsp.mUMat.getMat(vsdk::ACCESS_READ | OAL_USAGE_CACHED);
      FILE*     lpSavefile = 0;
      lpSavefile           = fopen("sony_dualexp.uyvy", "w+");
      if(lpSavefile)
      {
        fwrite((void*)(uintptr_t)frame_map.data, SCR_WIDTH * 2, SCR_HEIGHT, lpSavefile);
        printf("saved output images as sony_dualexp.uyvy\n");
        fclose(lpSavefile);
      } // if file opened
      else
      {
        lRet = -1;
      } // else from if file opened
      arContext.mKbMode = 0;
    } // if image available
    else
    {
      printf("output image NULL\n");
    } // else from if image available
  }   // if save output image

#else  // #ifndef __STANDALONE__
  if(arContext.mKbMode == 's')
  {
    arContext.mKbMode = 0;
    printf("%s(): Function NOT IMPLEMTED.\n", __func__);
  } // if save output image

  if(arContext.mKbMode == 'S')
  {
    arContext.mKbMode = 0;
    printf("%s(): Function NOT IMPLEMTED.\n", __func__);
  } // if save input image
#endif // #ifndef __STANDALONE__

  return lRet;
} // FrameSave()

//***************************************************************************

static char GetChar()
{
#ifdef __STANDALONE__
  return sci_0_testchar();
#else  // #ifdef __STANDALONE__
  int lChar = GetCharNonBlock();
  return (char)((lChar < 0) ? 0 : lChar & 0xff);
#endif // else from #ifdef __STANDALONE__
} // GetChar()

//***************************************************************************

static int32_t KbAction(AppContext& arContext)
{
  int32_t lRet = 0;
  uint8_t lKey = arContext.mKbKey;
  if(lKey == 'h')
  { // help
    printf(
        "keyboard help:\n"
        "h .........help\n"
        "p .........change logging print mode\n"
#ifndef __STANDALONE__
        "s .........save output frame\n"
        "S .........save  input frame\n"
#endif
        "t .........toggle test bar\n"
        "i .........enter IIC command mode\n"
        "  aaaavv ..enter IIC command to 0xaaaa with value 0xvv\n"
        "  ret .....write to IIC\n"
        "  r .......read from IIC\n"
        "M .........toggle tone mapping mode\n"
        "O .........toggle Output LUT mode\n"
        "k .........enter set knee point mode\n"
        "  + .......move up   knee point by %d/256\n"
        "  - .......move down knee point by %d/256\n"
        "  0 .......move knee point to 128/256 (default)\n"
        "  1 .......move knee point to   0/256 (short exposure)\n"
        "  2 .......move knee point to 256/256 (long exposure)\n"
        "N .........enter set denoise mode\n"
        "  + .......move up   noise threshold by 256\n"
        "  - .......move down noise threshold by 256\n"
        "R .........select set red channel gain mode\n"
        "  + .......move up   r_chgain by 16/256\n"
        "  - .......move down r_chgain by 16/256\n"
        "B .........select set blue channel gain mode\n"
        "  + .......move up   b_chgain by 16/256\n"
        "  - .......move down b_chgain by 16/256\n"
        "A .........select auto color gain mode\n"
        "Z .........select black level mode\n",
        ALPHA_STEP, ALPHA_STEP);
    return lRet;
  }
  if(lKey == 'p')
  {
    static char lPrintmode[6][11] = {"off", "fps", "ipu prof", "exposure", "awb", "buffer log"};

    arContext.mKbLog++;
    if(arContext.mKbLog == 6)
      arContext.mKbLog = 0;

    printf("Statistics Print = %s\n", lPrintmode[arContext.mKbLog]);
    arContext.mKbI2c  = 0;
    arContext.mKbMode = 'p';
    return lRet;
  }
  if(lKey == 'i')
  {
    arContext.mKbMode = 'i';
    printf("Mode = IIC command\n");
    arContext.mKbI2c = 0;
    return lRet;
  }
  if(lKey == 't')
  {
    if(arContext.mKbTestbar)
    {
      arContext.mKbMode    = 0;
      arContext.mKbTestbar = 0;
      printf("testbar off\n");
      if(SONY_RegWrite(CSI_IDX_CAM, 0x308c, 0x00) != CAM_LIB_SUCCESS)
      {
        lRet = -1;
      }
    }
    else
    {
      arContext.mKbMode    = 't';
      arContext.mKbTestbar = 1;
      if(SONY_RegWrite(CSI_IDX_CAM, 0x308c, 0x29) != CAM_LIB_SUCCESS)
      {
        lRet = -1;
      }
      printf("testbar on\n");
      arContext.mRbAuto = 0;
    }
    arContext.mKbI2c = 0;
    return lRet;
  }

  if(lKey == 'M')
  {
    arContext.mKbHDRMode++;
    if(arContext.mKbHDRMode > 1)
      arContext.mKbHDRMode = 0;

    printf(
        "\n"
        "Toggle tone mapping mode %d\n",
        arContext.mKbHDRMode);
    switch(arContext.mKbHDRMode)
    {
      case 0: // tone mapping at input
        arContext.mLAlphaVal = 128;
        arContext.mKbOutLut  = 1;
        break;
      case 1: // remap input, copy output
        arContext.mLAlphaVal = 16;
        arContext.mKbOutLut  = 2;
        break;
    }
    if(seq_setReg(HDR_ENGINE, 0, HDR_ALPHA, arContext.mLAlphaVal) != SEQ_LIB_SUCCESS)
    {
      lRet = -1;
    }
    if(seq_setReg(HDR_ENGINE, 0, HDR_ALPHA + 1, 256 - arContext.mLAlphaVal) != SEQ_LIB_SUCCESS)
    {
      lRet = -1;
    }
    if(SetOutLut(arContext.mKbOutLut) != 0)
    {
      lRet = -1;
    }

    arContext.mKbI2c = 0;
    return lRet;
  }
  if(lKey == 'O')
  {
    arContext.mKbOutLut++;
    if(arContext.mKbOutLut > 3)
      arContext.mKbOutLut = 0;

    printf("OutLut = %d\n", arContext.mKbOutLut);
    if(SetOutLut(arContext.mKbOutLut) != 0)
    {
      lRet = -1;
    }
    arContext.mKbI2c  = 0;
    arContext.mKbMode = 0;
    return lRet;
  }
  if(lKey == 'k')
  {
    printf("Mode = Alpha selection (0/1/2/+/-)\n");
    arContext.mKbMode = 'k';
    return lRet;
  }
  if(lKey == 'N')
  {
    printf("Mode = deNoise tune (+/-): %d\n", arContext.mKbNoiseLevel);
    arContext.mKbMode = 'N';
    return lRet;
  }
  if(lKey == 'R')
  {
    arContext.mKbMode = 'R';
    printf("Mode = Red Gain set %d\n", arContext.mChGainLevel.mLevel[0]);
    arContext.mRbAuto = 0;
    arContext.mKbI2c  = 0;
    return lRet;
  }
  if(lKey == 'B')
  {
    arContext.mKbMode = 'B';
    printf("Mode = Blue Gain set %d\n", arContext.mChGainLevel.mLevel[3]);
    arContext.mRbAuto = 0;
    arContext.mKbI2c  = 0;
    return lRet;
  }
  // channel gain
  if(lKey == 'A')
  {
    arContext.mKbMode = 0;
    printf("Mode = Auto Channel Gain\n");
    arContext.mRbAuto = 1;
    arContext.mKbI2c  = 0;
    return lRet;
  }
  if(lKey == 'Z')
  {
    arContext.mKbMode = 'Z';
    printf("Mode = Black Level %d\n", arContext.mBlackLevel);
    arContext.mKbI2c = 0;
    return lRet;
  }

  if(lKey == 's')
  {
    arContext.mKbMode = 's';
    printf("Mode = Save output frame\n");
    arContext.mKbI2c = 0;
    return lRet;
  }

  if(lKey == 'S')
  {
    arContext.mKbMode = 'S';
    printf("Mode = Save input frame\n");
    arContext.mKbI2c = 0;
    return lRet;
  }

  if(arContext.mKbMode == 'p')
  {
    if(lKey == '0')
    {
      arContext.mKbLog = 0;
      printf("Statistics Print = off\n");
    }
  }

  if(arContext.mKbMode == 'k')
  {
    switch(lKey)
    {
      case '0':
        arContext.mLAlphaVal = 128;
        arContext.mKbOutLut  = 0;
        printf("combined mode\n");
        break;
      case '1':
        arContext.mLAlphaVal = 0;
        arContext.mKbOutLut  = 0;
        printf("short exposure\n");
        break;
      case '2':
        arContext.mLAlphaVal = 256;
        arContext.mKbOutLut  = 0;
        printf("2 long exposure\n");
        break;
      case '+':
        if(arContext.mLAlphaVal <= 240)
          arContext.mLAlphaVal += ALPHA_STEP;
        arContext.mKbOutLut = 0;
        printf("LAlpha = %d\n", arContext.mLAlphaVal);
        break;
      case '-':
        if(arContext.mLAlphaVal >= 16)
          arContext.mLAlphaVal -= ALPHA_STEP;
        arContext.mKbOutLut = 0;
        printf("LAlpha = %d\n", arContext.mLAlphaVal);
        break;
    }
    if(seq_setReg(HDR_ENGINE, 0, HDR_ALPHA, arContext.mLAlphaVal) != SEQ_LIB_SUCCESS)
    {
      lRet = -1;
    }
    if(seq_setReg(HDR_ENGINE, 0, HDR_ALPHA + 1, 256 - arContext.mLAlphaVal) != SEQ_LIB_SUCCESS)
    {
      lRet = -1;
    }
    if(SetOutLut(arContext.mKbOutLut) != 0)
    {
      lRet = -1;
    }
    arContext.mKbI2c = 0;
    return lRet;
  }
  if(arContext.mKbMode == 'N')
  {
    switch(lKey)
    {
      case '+':
        arContext.mKbNoiseLevel += 256;
        break;
      case '-':
        arContext.mKbNoiseLevel -= 256;
        break;
    }
    arContext.mKbNoiseLevel &= 0xffff;
    printf("Noise Level = %d\n", arContext.mKbNoiseLevel);
    if(seq_setReg(DENOISE_ENGINE, 0, 0x56, arContext.mKbNoiseLevel) != SEQ_LIB_SUCCESS)
    {
      lRet = -1;
    }
    arContext.mKbI2c = 0;
    return lRet;
  }
  if(arContext.mKbMode == 'R')
  {
    switch(lKey)
    {
      case '+':
        arContext.mChGainLevel.mLevel[0] += 16;
        if(arContext.mChGainLevel.mLevel[0] > 640)
        {
          arContext.mChGainLevel.mLevel[0] = 640;
        }
        break;
      case '-':
        arContext.mChGainLevel.mLevel[0] -= 16;
        if(arContext.mChGainLevel.mLevel[0] < 240)
        {
          arContext.mChGainLevel.mLevel[0] = 240;
        }
        break;
    }
    printf("ChGainRed = %d\n", arContext.mChGainLevel.mLevel[0]);
    if(seq_setReg(EXPOSURE_ENGINE, 0, 0x74, arContext.mChGainLevel.mLevel[0]) != SEQ_LIB_SUCCESS)
    {
      lRet = -1;
    } // R
    arContext.mKbI2c = 0;
  }
  if(arContext.mKbMode == 'B')
  {
    switch(lKey)
    {
      case '+':
        arContext.mChGainLevel.mLevel[3] += 16;
        if(arContext.mChGainLevel.mLevel[3] > 640)
        {
          arContext.mChGainLevel.mLevel[3] = 640;
        }
        break;
      case '-':
        arContext.mChGainLevel.mLevel[3] -= 16;
        if(arContext.mChGainLevel.mLevel[3] < 240)
        {
          arContext.mChGainLevel.mLevel[3] = 240;
        }
        break;
    }
    printf("ChGainBlue = %d\n", arContext.mChGainLevel.mLevel[3]);
    if(seq_setReg(EXPOSURE_ENGINE, 0, 0x77, arContext.mChGainLevel.mLevel[3]) != SEQ_LIB_SUCCESS)
    {
      lRet = -1;
    } // R
    arContext.mKbI2c = 0;
  }
  if(arContext.mKbMode == 'Z')
  {
    switch(lKey)
    {
      case '+':
        if(arContext.mBlackLevel <= 504)
          arContext.mBlackLevel += 8;
        printf("Black Level = %d\n", arContext.mBlackLevel);
        break;
      case '-':
        if(arContext.mBlackLevel >= 8)
          arContext.mBlackLevel -= 8;
        printf("Black Level = %d\n", arContext.mBlackLevel);
        break;
    }
    // GPR0 (IPUS): R
    if(seq_setReg(CHGAIN_ENGINE, 0, 0x70, arContext.mBlackLevel << 4) != SEQ_LIB_SUCCESS)
    {
      lRet = -1;
    }
    // GPR1 (IPUS): GR
    if(seq_setReg(CHGAIN_ENGINE, 0, 0x71, arContext.mBlackLevel << 4) != SEQ_LIB_SUCCESS)
    {
      lRet = -1;
    }
    // GPR2 (IPUS): GB
    if(seq_setReg(CHGAIN_ENGINE, 0, 0x72, arContext.mBlackLevel << 4) != SEQ_LIB_SUCCESS)
    {
      lRet = -1;
    }
    // GPR3 (IPUS): B
    if(seq_setReg(CHGAIN_ENGINE, 0, 0x73, arContext.mBlackLevel << 4) != SEQ_LIB_SUCCESS)
    {
      lRet = -1;
    }
    arContext.mKbI2c = 0;
    return lRet;
  }
  if(arContext.mKbMode == 'i')
  {
    if((lKey >= '0') && (lKey <= '9'))
    { // enter IIC command
      arContext.mKbI2c <<= 4;
      arContext.mKbI2c += (lKey - '0') & 0xf;
      arContext.mKbI2c &= 0xffffff;
      printf("IIC command 0x%x - 0x%x\n", (arContext.mKbI2c >> 8), (arContext.mKbI2c & 0xff));
      return lRet;
    }
    if((lKey >= '0') && (lKey <= '9'))
    { // enter IIC command
      arContext.mKbI2c <<= 4;
      arContext.mKbI2c += (lKey - '0') & 0xf;
      arContext.mKbI2c &= 0xffffff;
      printf("IIC command 0x%x - 0x%x\n", (arContext.mKbI2c >> 8), (arContext.mKbI2c & 0xff));
      return lRet;
    }
    if((lKey >= 'a') && (lKey <= 'f'))
    { // enter IIC command
      arContext.mKbI2c <<= 4;
      arContext.mKbI2c += (lKey - 'a' + 10) & 0xf;
      arContext.mKbI2c &= 0xffffff;
      printf("IIC command 0x%x - 0x%x\n", (arContext.mKbI2c >> 8), (arContext.mKbI2c & 0xff));
      return lRet;
    }
    if(strrchr("\n\rw", lKey))
    {
      printf("Writing to IIC\n");
      if(SONY_RegWrite(CSI_IDX_CAM, arContext.mKbI2c >> 8, arContext.mKbI2c & 0xff) != CAM_LIB_SUCCESS)
      {
        lRet = -1;
      }

      return lRet;
    }
    if(lKey == 'r')
    {
      printf("Read from IIC\n");
      {
        uint8_t val = 0;
        if(SONY_RegRead(CSI_IDX_CAM, (arContext.mKbI2c >> 8) & 0xffff, val) != CAM_LIB_SUCCESS)
        {
          lRet = -1;
        }
        printf("%x = 0x%x / %d\n", arContext.mKbI2c >> 8, val, val);
      }
      return lRet;
    }
  }
  return lRet;
} // KbAction()

//***************************************************************************
/*EOF*/
