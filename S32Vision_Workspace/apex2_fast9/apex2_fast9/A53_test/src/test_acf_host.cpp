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

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <umat.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>

#include "../../A53_gen/inc/apu_fast9color_process_controller.hpp"
#include "apu_fast9_color_process_controller_policy.hpp"

#include "apu_process_controller_data.hpp"

using namespace cv;
using namespace std;
using namespace apugen;

#ifdef APEX2_EMULATE
#include "apu_app_config.hpp"
#include "apu_lib.hpp"
#include "apu_extras.hpp"
#include "acf_lib.hpp"
using namespace APEX2;
#else
#include <icp_data.h>
#include <apex.h>
#include <oal.h>
using namespace icp;
#endif
//Start of user code user_define
/*
 * This is the region protected from the source code emitter,
 * all the rest can be changed by the source code emitter without notice.
 */

#if !defined(APEX2_EMULATE)
#include "frame_output_v234fb.h"
#define CHNL_CNT io::IO_DATA_CH3
#endif
//End of user code

int TEST_CALL()
{
#ifdef APEX2_EMULATE
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stderr, NULL, _IONBF, 0);
  APU_App_Config::LoadConfig("./A53_inc/apu_app.cfg");
#endif

  uint8_t lNumberFast9Color0Threshold = 50;
  uint8_t lNumberFast9Color0MarkColorChannel = 1;
  
  #ifdef APEX2_EMULATE
    auto lFast9Color0InputFileName = APU_App_Config::GetValue<std::string>("INPUT_ROOT") + "in_color_256x256.png";
  #else
    auto lFast9Color0InputFileName = "data/common/in_color_256x256.png";
  #endif
    cv::Mat lImageFast9Color0Input = cv::imread(lFast9Color0InputFileName, CV_LOAD_IMAGE_COLOR);

  //Start of user code declare_inputs
  /*
  * This is the region protected from the source code emitter,
  * all the rest can be changed by the source code emitter without notice.
  */

  //End of user code

  cv::UMat lFast9Color0OutputData;

  //Start of user code declare_outputs
  /*
  * This is the region protected from the source code emitter,
  * all the rest can be changed by the source code emitter without notice.
  */

  //End of user code

  //========================================================================
  // Processing part
  //========================================================================
  int32_t retVal = 0;

  APU_Fast9Color_Process_Controller Fast9Color0ProcessController(0);

  //-------------------------------- apuFast9color0ProcessController --------------------------------

  InputProcessControllerData<cv::Mat, Fast9ColorInputConversionPolicy> lFast9Color0Input(256, 256, VSDK_CV_8UC3);
  InputProcessControllerData<uint8_t, Fast9ColorThresholdConversionPolicy> lFast9Color0Threshold(1, 1, VSDK_CV_8UC1);
  InputProcessControllerData<uint8_t, Fast9ColorMarkColorChannelConversionPolicy> lFast9Color0MarkColorChannel(1, 1, VSDK_CV_8UC1);

  //Start of user code input_data_Fast9Color0
/*
 * This is the region protected from the source code emitter,
 * all the rest can be changed by the source code emitter without notice.
 */

  //End of user code

  
  lFast9Color0Threshold.SetData(lNumberFast9Color0Threshold);
  lFast9Color0MarkColorChannel.SetData(lNumberFast9Color0MarkColorChannel);
  
  lFast9Color0Input.SetData(lImageFast9Color0Input);

  // Verification
    if (lFast9Color0Input.GetDataPtr()->empty())
    {
       cout << "Input image " << lFast9Color0InputFileName << "was not found\n";
       exit(1);
    }
  #ifdef APEX2_EMULATE
    imshow("SourceImage", *lFast9Color0Input.GetDataPtr());
  #endif

  OutputProcessControllerData<cv::UMat, Fast9ColorOutputConversionPolicy> lFast9Color0Output(256, 256, VSDK_CV_8UC3);

  //Start of user code output_data_Fast9Color0
/*
 * This is the region protected from the source code emitter,
 * all the rest can be changed by the source code emitter without notice.
 */

  //End of user code

  retVal |= Fast9Color0ProcessController.Init();
  retVal |= Fast9Color0ProcessController.Connect(lFast9Color0Input.mGraphDescr, lFast9Color0Threshold.mGraphDescr, lFast9Color0MarkColorChannel.mGraphDescr, lFast9Color0Output.mGraphDescr);
  retVal |= Fast9Color0ProcessController.Start();
  retVal |= Fast9Color0ProcessController.Wait();
  
    //Start of user code Fast9Color0_after_run
  /*
   * This is the region protected from the source code emitter,
   * all the rest can be changed by the source code emitter without notice.
   */
  
    //End of user code
  
  if(retVal)
  {
    cout << "Fast9Color0ProcessController error\n";
    return retVal;
  }

  lFast9Color0Output.GetData(lFast9Color0OutputData);
  //Start of user code processing_part
  /*
  * This is the region protected from the source code emitter,
  * all the rest can be changed by the source code emitter without notice.
  */

  //End of user code

  //========================================================================
  // Display part
  //========================================================================
// Output the picture
#ifdef APEX2_EMULATE
  imshow("Result", lFast9Color0OutputData);
  waitKey(0);
#endif

#ifdef APEX2_EMULATE
  auto lFast9Color0OutputFileName = APU_App_Config::GetValue<std::string>("OUTPUT_ROOT") + "out_fast9_color.png";
#else
  auto lFast9Color0OutputFileName = "out_fast9_color.png";
#endif
  imwrite(lFast9Color0OutputFileName, lFast9Color0OutputData);

  //Start of user code display_part
  /*
  * This is the region protected from the source code emitter,
  * all the rest can be changed by the source code emitter without notice.
  */
#if !defined(APEX2_EMULATE)
  // Initialize different output class for Standalone and Linux
  io::FrameOutputV234Fb fbScreen(1280, 720, io::IO_DATA_DEPTH_08, CHNL_CNT);

  // Output buffer (screen size) and it's mapped version (using cv mat in order to have copyTo functions)
  vsdk::UMat umatOutput = vsdk::UMat(720, 1280, VSDK_CV_8UC3);
  {
    cv::Mat matOutput = umatOutput.getMat(ACCESS_WRITE | OAL_USAGE_CACHED);
    cv::Mat matResult = lFast9Color0OutputData.getMat(ACCESS_WRITE | OAL_USAGE_CACHED);
    memset(matOutput.data, 0, 720 * 1280 * 3);

    matResult.copyTo(matOutput(cv::Rect(0, 232, 256, 256)));
  }
  fbScreen.PutFrame(umatOutput);
#endif

  //End of user code

  return 0;
}
