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
#include <sys/time.h>
#endif // #ifdef __STANDALONE__
#include <string.h>

#include "config.h"

#ifdef __STANDALONE__
#include "frame_output_dcu.h"
#else // #ifdef __STANDALONE__
#include "frame_output_v234fb.h"
#endif // else from #ifdef __STANDALONE__

#include "isp_cam_sony.h"

#include "sdi.hpp"
#include "rgb_yuv_gs8_c.h"
#include "iofiles.h"        // *** from graph ***
#include "lut.h" // gamma table

#include "vdb_log.h"
#include "iofiles.h"       // *** from graph ***
#include "rgb_yuv_gs8_c.h" // *** from graph ***

#include <common_helpers.h>

//***************************************************************************
// constants
//***************************************************************************

#define ISP_MODE_PERIOD 300 //< number of frames after which the mode changes

#define IPU_DUMP_STATE
#define DO_IPU_PROF
//#define DO_EXP_LOG

#define DDR_OUT_BUFFER_CNT 3 ///< number of DDR buffers per ISP stream

// Possible to set input resolution (must be supported by the DCU)
#define IMG_WIDTH 1280 ///< width of DDR buffer in pixels
#define IMG_HEIGHT 720 ///< height of DDR buffer in pixels
#define LUMA_WIDTH (IMG_WIDTH)
#define LUMA_HEIGHT (IMG_HEIGHT)

#define CHNL_CNT_Y io::IO_DATA_CH1
#define CHNL_CNT_YUV io::IO_DATA_CH2
#define CHNL_CNT_RGB io::IO_DATA_CH3

#define LOOP_NUM 100

// ISP preprocessing parameters
#define EXPOSURE_ENGINE 1, 1
#define TONEHDR_ENGINE 1, 1
#define GAMMA_ENGINE 1, 3
#define GAMMA_ON 1
#define KNEE_VAL 256
#define GAMMA_LUT_OFFSET 0

#define DDR_BUFFER_CNT 3

#ifdef DO_IPU_PROF
#define IPU_PROF_FRAMES ((LOOP_NUM < 10) ? LOOP_NUM : 10)
#endif

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
#endif // else from #ifdef __STANDALONE__

//***************************************************************************
// types
//***************************************************************************
struct AppContext
{
  sdi_grabber* mpGrabber; ///< pointer to grabber instance
  sdi_FdmaIO*  mpFdma;    ///< fdma object pointer
  uint16_t     mKneeVal;  ///< knee value for IPU
  int32_t      mFdmaChannel;

  unsigned long mTimeStart;

  bool     mError;  ///< to signal ISP problems
  uint32_t mFrmCnt; ///< number of frames captured so far
};                  // struct AppContext

//***************************************************************************
// local functions
//***************************************************************************

/**************************************************************************/
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
/** Configure camera.
  * 
  * \return 0 if all ok, <0 otherwise
  ************************************************************************/
static int32_t CamConfig(void);

/************************************************************************/
/** Update exposure setup.
  *
  * \return 0 if all ok, <0 otherwise 
  ************************************************************************/
static int32_t ExpControl(void);

/************************************************************************/
/** Log performance and status.
  * 
  * \param arContext application context
  ************************************************************************/
static int32_t LogPerformance(AppContext& arContext);

//extern SEQ_Buf_t	producerless_buffer_1;
#ifndef __STANDALONE__
/************************************************************************/
/** Handler for ISP events.
  * 
  * \param  aEventType type of the event received from ISP
  ************************************************************************/
static void SeqEventHandler(uint32_t aEventType, void*);

/************************************************************************/
/** Init DCU mode.
  * 
  * \param  arDCU reference to DCU handling object
  * \param  aChan number of channels  
  ************************************************************************/
static void DcuInit(io::FrameOutputV234Fb& arDCU, int32_t aChan);
#else  // #ifdef __STANDALONE__

/************************************************************************/
/** Init DCU mode.
  * 
  * \param  arDCU reference to DCU handling object
  * \param  aChan number of channels  
  ************************************************************************/
static void DcuInit(io::FrameOutputDCU& arDCU, int32_t aChan);
#endif // else from #ifdef __STANDALONE__

#ifndef __STANDALONE__
/************************************************************************/
/** SIGINT handler.
    * 
    * \param  aSigNo 
    ************************************************************************/
void SigintHandler(int aSigNo);

/************************************************************************/
/** SIGINT handler.
    * 
    * \param  aSigNo 
    * 
    * \return SEQ_LIB_SUCCESS if all ok
    *         SEQ_LIB_FAILURE if failed
    ************************************************************************/
int32_t SigintSetup(void);

//***************************************************************************

static bool sStop = false; ///< to signal Ctrl+c from command line

#endif // #ifndef __STANDALONE__

//***************************************************************************

int main(int argc, char** argv)
{
  int lRet = 0;

  AppContext lContext;

  //*** process command line parameters ***
  const char helpMsg_str[] =
      "\n**************************************************************\n"
      "** Sony imx224 multi output ISP demo\n"
      "** Description:\n"
      "**  o Sony imx224 camera (on MipiCsi_0) expected as image input.\n"
      "**  o ISP generates histogram for AEC, does debayering and\n"
      "**    generates 1280x720 RGB, YUV422 and grayscale images + 640x360\n"
      "**    grayscale image in parallel.\n"
      "**  o Live image from differrent pixel format streams are displayed\n"
      "**    in turns using DCU.\n"
      "**\n"
      "** Usage:\n"
      "**  o no cmd line parameters available.\n"
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
} // main()

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
  if(CamConfig() != 0)
  {
    printf("Failed to configure the camera.\n");
    return -1;
  } // if CamConfig() failed

#ifndef __STANDALONE__
  // *** install sequencer event callback ***
  if(arContext.mpGrabber->SeqEventCallBackInstall(&SeqEventHandler, NULL) != LIB_SUCCESS)
  {
    printf("Failed to install Sequencer event callback.\n");
    return -1;
  }    // if callback setup failed
#endif // #ifndef __STANDALONE__

  return 0;
} // Prepare()

//***************************************************************************

static void ContextInit(AppContext& arContext)
{
  arContext.mpGrabber    = NULL; ///< pointer to grabber instance
  arContext.mpFdma       = NULL; ///< fdma object pointer
  arContext.mFrmCnt      = 0;
  arContext.mFdmaChannel = FDMA_IX_YUV_TO_DDR;

  arContext.mTimeStart = 0;

  arContext.mError  = false;
  arContext.mFrmCnt = 0;
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

  if(arContext.mpGrabber->ProcessSet(gpGraph, &gGraphMetadata) != LIB_SUCCESS)
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
  sdi_FdmaIO*         lpFdma = arContext.mpFdma;

  lFrmDesc = SDI_ImageDescriptor(IMG_WIDTH, NUM_OUT_LINES, YUV422Stream_YUYV);
  if(lpFdma->DdrBufferDescSet(FDMA_IX_YUV_TO_DDR, lFrmDesc) != LIB_SUCCESS)
  {
    printf("Failed to set image descriptor.\n");
    return -1;
  } // if frame descriptor setup failed
  lFrmDesc = SDI_ImageDescriptor(LUMA_WIDTH, NUM_OUT_LINES, GS8);
  if(lpFdma->DdrBufferDescSet(FDMA_IX_Y_TO_DDR, lFrmDesc) != LIB_SUCCESS)
  {
    printf("Failed to set image descriptor.\n");
    return -1;
  } // if frame descriptor setup failed
  lFrmDesc = SDI_ImageDescriptor(IMG_WIDTH, NUM_OUT_LINES, RGB888);
  if(lpFdma->DdrBufferDescSet(FDMA_IX_RGB_TO_DDR, lFrmDesc) != LIB_SUCCESS)
  {
    printf("Failed to set image descriptor.\n");
    return -1;
  } // if frame descriptor setup failed
  lFrmDesc = SDI_ImageDescriptor(NUM_OUT_Y_SCALED_BYTES, NUM_OUT_Y_SCALED_LINES, GS8);
  if(lpFdma->DdrBufferDescSet(FDMA_IX_Y_SCALED_TO_DDR, lFrmDesc) != LIB_SUCCESS)
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
  io::FrameOutputDCU lDcuOutput;
#else // #ifdef __STANDALONE__
  // setup Ctrl+C handler
  if(SigintSetup() != SEQ_LIB_SUCCESS)
  {
    VDB_LOG_ERROR("Failed to register Ctrl+C signal handler.");
    return -1;
  }

  printf("Press Ctrl+C to terminate the demo.\n");

  io::FrameOutputV234Fb lDcuOutput;
#endif

  // *** init DCU ***
  DcuInit(lDcuOutput, arContext.mFdmaChannel);

  // *** start grabbing ***

  GETTIME(&arContext.mTimeStart);
  if(arContext.mpGrabber->Start() != LIB_SUCCESS)
  {
    printf("Failed to start the grabber.\n");
    return -1;
  } // if Start() failed

  SDI_Frame lFrame;
  // *** grabbing/processing loop ***
  while(1)
  {
#ifndef __STANDALONE__
    if(sStop)
    {
      break; // break if Ctrl+C pressed
    }        // if Ctrl+C
#endif       //#ifndef __STANDALONE__

    for(uint32_t lLoop = 0; lLoop < LOOP_NUM; lLoop++)
    {
      lFrame = arContext.mpGrabber->FramePop(arContext.mFdmaChannel);
      if(lFrame.mUMat.empty())
      {
        printf("Failed to pop ISP output frame.\n");
        arContext.mError = true;
        break;
      } // if pop failed

      // *** do exposure control ***
      if(ExpControl() != 0)
      {
        printf("Failed to run exposure control.\n");
        return -1;
      } // if CamConfig() failed

      arContext.mFrmCnt++;
      // display the frame
      lDcuOutput.PutFrame(lFrame.mUMat);

      if(arContext.mpGrabber->FramePush(lFrame) != LIB_SUCCESS)
      {
        printf("Failed to push ISP frame.\n");
        arContext.mError = true;
        break;
      } // if push failed
    }   // for LOOP_NUM

    if(arContext.mError)
    {
      break;
    }

    // *** change mode ***
    if(0 == (arContext.mFrmCnt % ISP_MODE_PERIOD))
    {
      while(1)
      {
        arContext.mFdmaChannel++;
        if(arContext.mFdmaChannel >= FDMA_IX_END)
        {
          arContext.mFdmaChannel = 0;
        } // if arAppContext.mFdmaChannel wrap

        break;
      } // while(1)

      DcuInit(lDcuOutput, arContext.mFdmaChannel);
      if(io::IO_OK != lDcuOutput.GetStatus())
      {
        printf("********** DCU Error !!!!!!!!!!! ***********\n");
      } // if DCU ok
    }   // if time to switch mode

    if(LogPerformance(arContext) != 0)
    {
      printf("Failed to log performance\n");
      arContext.mError = true;
      return -1;
    }

#ifndef __STANDALONE__
    if(sStop)
    {
      break; // break if Ctrl+C pressed
    }        // if Ctrl+C
#endif       // #ifndef __STANDALONE__

  } // while(1)

  if(arContext.mError)
  {
    return -1;
  }
  else
  {
    return 0;
  }
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
    ;  // *** don't return ***
#endif // #ifdef __STANDALONE__

  if(sdi::Close(0) != LIB_SUCCESS)
  {
    printf("Failed to terminate use of SDI.\n");
    lRet = -1;
  } // if SDI use termination failed

  return lRet;
} // Cleanup()

//***************************************************************************

static int32_t CamConfig()
{
  // *** exposure kernel ***
  // black level correction
  if(seq_setReg(EXPOSURE_ENGINE, 0, 0x70, 0xf08) != SEQ_LIB_SUCCESS) // GPR0
  {
    return -1;
  }                                                                  // if seq_setReg() failed
  if(seq_setReg(EXPOSURE_ENGINE, 0, 0x71, 0xf08) != SEQ_LIB_SUCCESS) // GPR1
  {
    return -1;
  }                                                                  // if seq_setReg() failed
  if(seq_setReg(EXPOSURE_ENGINE, 0, 0x72, 0xf08) != SEQ_LIB_SUCCESS) // GPR2
  {
    return -1;
  }                                                                  // if seq_setReg() failed
  if(seq_setReg(EXPOSURE_ENGINE, 0, 0x73, 0xf08) != SEQ_LIB_SUCCESS) // GPR3
  {
    return -1;
  } // if seq_setReg() failed

  // channel gain
  if(seq_setReg(EXPOSURE_ENGINE, 0, 0x74, 450) != SEQ_LIB_SUCCESS) // GPR4: R
  {
    return -1;
  }                                                                // if seq_setReg() failed
  if(seq_setReg(EXPOSURE_ENGINE, 0, 0x75, 256) != SEQ_LIB_SUCCESS) // GPR5: GR
  {
    return -1;
  }                                                                // if seq_setReg() failed
  if(seq_setReg(EXPOSURE_ENGINE, 0, 0x76, 256) != SEQ_LIB_SUCCESS) // GPR6: GB
  {
    return -1;
  }                                                                // if seq_setReg() failed
  if(seq_setReg(EXPOSURE_ENGINE, 0, 0x77, 400) != SEQ_LIB_SUCCESS) // GPR7: B
  {
    return -1;
  } // if seq_setReg() failed

  // *** tonemap kernel ***
  if(seq_setReg(TONEHDR_ENGINE, 0, 0x78, KNEE_VAL) != SEQ_LIB_SUCCESS) // GPR8
  {
    return -1;
  } // if seq_setReg() failed

  // *** gamma kernel ***
  if(seq_setReg(GAMMA_ENGINE, 0, 0x71, GAMMA_LUT_OFFSET) != SEQ_LIB_SUCCESS) // GPR1
  {
    return -1;
  }                                                                                   // if seq_setReg() failed
  if(seq_setReg(GAMMA_ENGINE, 1, 0x78, ((GAMMA_LUT_OFFSET) >> 1)) != SEQ_LIB_SUCCESS) // LUTA
  {
    return -1;
  } // if seq_setReg() failed

  {
    uint16_t lutcnt = 0;
    while(lutcnt < 2048)
    {
      uint32_t val;
      val = lut[lutcnt] | ((long)lut[lutcnt] << 16);
      lutcnt++;
      if(seq_setReg(GAMMA_ENGINE, 1, 0x7C, val) != SEQ_LIB_SUCCESS) // LUTD
      {
        return -1;
      }                                                          // if seq_setReg() failed
    }                                                            // while not end of LUT
  }                                                              // LUT setup block
  if(seq_setReg(GAMMA_ENGINE, 0, 0x70, GAMMA_ON) != LIB_SUCCESS) // GPR0
  {
    return -1;
  } // if seq_setReg() failed

  // Camera Configuration
  // modify camera geometry !!! setup before setting up expsoure control
  SONY_Geometry_t lGeo;
  lGeo.mCsiIdx = CSI_IDX_0;
  if(SONY_GeometryGet(lGeo) != CAM_LIB_SUCCESS)
  {
    printf("Failed to get camera geometry.\n");
    return -1;
  } // if SONY_GeometryGet failed
  lGeo.mWinMode = SONY_CROP;
  //lGeo.mHeight = 729;
  lGeo.mWidth   = 1288;
  lGeo.mHmax    = 0;
  lGeo.mOb      = 2;
  lGeo.mXoff    = 0;
  lGeo.mYoff    = 0;
  lGeo.mVerFlip = 1;
  lGeo.mHorFlip = 1;
  lGeo.mFps     = 20;

  if(SONY_GeometrySet(lGeo) != CAM_LIB_SUCCESS)
  {
    printf("Failed to set camera geometry.\n");
    return -1;
  } // if SONY_GeometrySet failed

  // modify camera exposure
  SONY_Exposure_t lExp;
  lExp.mCsiIdx = CSI_IDX_0;
  if(SONY_ExposureGet(lExp) != CAM_LIB_SUCCESS)
  {
    printf("Failed to get camera exposure config.\n");
    return -1;
  } // if SONY_ExposureGet failed
  lExp.mConversionGain = 1;
  lExp.mExposureLines  = 500;
  lExp.mAnalogGain     = 0;
  lExp.mConfigFlags    = SONY_EXP_EL | SONY_EXP_CG | SONY_EXP_AG;
  if(SONY_ExposureSet(lExp) != CAM_LIB_SUCCESS)
  {
    printf("Failed to set camera exposure config.\n");
    return -1;
  } // if SONY_ExposureSet failed

  printf("VMAX = %4d, Lines = %3d, OB = %2d\n", lGeo.mVmax, lGeo.mHeight, lGeo.mOb);

  // *** enable IPU histogram readout ***
  if(SEQ_HistogramEnable(SEQ_IPUS1) != SEQ_LIB_SUCCESS)
  {
    printf("Failed to enable histogram readout.\n");
    return -1;
  } // if histogram not enabled

  return 0;
} // CamConfig();

//***************************************************************************

static int32_t ExpControl()
{
  SONY_ExpState_t lExpState = SONY_EXP_SENT;
  if(SONY_ExposureStatusGet(CSI_IDX_0, lExpState) != CAM_LIB_SUCCESS)
  {
    printf("Failed to get camera exposure status.\n");
    return -1;
  } // if failed to get exp status

  if(lExpState == SONY_EXP_READY)
  {
    static SEQ_IpuHist_t lIpuHist;
    uint32_t             lExp = 0;
    memset(&lIpuHist, 0, sizeof(SEQ_IpuHist_t));
    lIpuHist.mIpuIdx = SEQ_IPUS1;

    // do expsoure control
    if(SEQ_HistogramGet(&lIpuHist) != SEQ_LIB_SUCCESS)
    {
      printf("Failed to get histogram data.\n");
      return -1;
    } // if failed to get exp status
    SONY_ExposureControlEqual(&lIpuHist, 0.0, &lExp);

    if(SONY_ExposureSet(CSI_IDX_0, lExp) != CAM_LIB_SUCCESS)
    {
      printf("Failed to update camera exposure.\n");
      return -1;
    } // if failed to get exp status
  }   // if ready for exp update

  return 0;
} // ExpControl

//***************************************************************************

static int32_t LogPerformance(AppContext& arContext)
{
  unsigned long lTimeEnd = 0, lTimeDiff = 0;

  GETTIME(&lTimeEnd);
  lTimeDiff            = lTimeEnd - arContext.mTimeStart;
  arContext.mTimeStart = lTimeEnd;

  printf("%d frames took %lu usec (%5.2ffps)\n", LOOP_NUM, lTimeDiff, LOOP_NUM * 1000000.0 / ((float)lTimeDiff));

#ifdef DO_EXP_LOG
  {
    SONY_Exposure_t lExp;
    if(SONY_ExposureGet(CSI_IDX_0, &lExp) != LIB_SUCCESS)
    {
      return -1;
    }

    printf("Exposure Lines = %3d CG= %d AG=%2ddB\n", lGeo.mVmax - lExp.mExposureLines - 1, lExp.mConversionGain,
           lExp.mAnalogGain / 10 /*sGain * 6*/
    );
  }
#endif

#ifdef DO_IPU_PROF
  ISP_DumpPerfCounters(IPU_PROF_FRAMES);
#endif
  return 0;
} // LogPerformace()

//***************************************************************************

static void SeqEventHandler(uint32_t aEventType, void*)
{
  switch(aEventType)
  {
    case SEQ_MSG_TYPE_OTHERERROR:
      char lString[SEQ_PRAM_AUX_DATA_SIZE] = {0};

      printf("*** Sequencer-Error: ");

      if(SEQ_MsgAuxDataGet(lString, SEQ_PRAM_AUX_DATA_SIZE) != SEQ_LIB_SUCCESS)
      {
        VDB_LOG_NOTE("Failed to get the auxiliary data from PRAM.\n");
      }
      printf("%s\n", lString);
      break;
  }

} // SeqEventHandler(uint32_t aEventType)

  //***************************************************************************

#ifdef __STANDALONE__
static void DcuInit(io::FrameOutputDCU& arDCU, int32_t aChan)
#else  //#ifdef __STANDALONE__
static void DcuInit(io::FrameOutputV234Fb& arDCU, int32_t aChan)
#endif // else from #ifdef __STANDALONE__
{
  switch(aChan)
  {
    case FDMA_IX_Y_TO_DDR:
      printf("*** Mode: Y ***\n");
      arDCU.Init(LUMA_WIDTH, LUMA_HEIGHT, io::IO_DATA_DEPTH_08, CHNL_CNT_Y, DCU_BPP_8);
      break;

    case FDMA_IX_YUV_TO_DDR:
      printf("*** Mode: YUV ***\n");
      arDCU.Init(IMG_WIDTH, IMG_HEIGHT, io::IO_DATA_DEPTH_08, io::IO_DATA_CH3, DCU_BPP_YCbCr422);
      break;

    case FDMA_IX_RGB_TO_DDR:
      printf("*** Mode: RGB ***\n");
      arDCU.Init(IMG_WIDTH, IMG_HEIGHT, io::IO_DATA_DEPTH_08, CHNL_CNT_RGB);
      break;

    case FDMA_IX_Y_SCALED_TO_DDR:
      printf("*** Mode: Y Scaled ***\n");
      arDCU.Init(NUM_OUT_Y_SCALED_BYTES, NUM_OUT_Y_SCALED_LINES, io::IO_DATA_DEPTH_08, CHNL_CNT_Y, DCU_BPP_8);
      break;

    default:
    {
      printf("Output format not supported!\n");
    } // default

  } // switch()
} // DcuInit()

  //***************************************************************************

#ifndef __STANDALONE__
void SigintHandler(int)
{
  sStop = true;
} // SigintHandler()

//***************************************************************************

int32_t SigintSetup()
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
#endif // #ifndef __STANDALONE__

/* EOF */
