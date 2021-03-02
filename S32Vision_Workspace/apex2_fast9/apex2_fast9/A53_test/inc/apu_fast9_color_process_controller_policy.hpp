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

/*!*********************************************************************************
*  @file    apu_fast9_color_process_controller_policy.hpp
*  @brief   Policy for APEX2 process controller data
***********************************************************************************/

#ifndef APUFAST9COLORPROCESSCONTROLLERPOLICY_HPP
#define APUFAST9COLORPROCESSCONTROLLERPOLICY_HPP

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <umat.hpp>

#ifdef APEX2_EMULATE
#include "apu_app_config.hpp"
#endif

namespace apugen{
class Fast9ColorInputConversionPolicy
{
protected:
  void Conversion(cv::Mat* aSource, vsdk::UMat& aGraphDescr)
  {
    aGraphDescr = aSource->getUMat(cv::ACCESS_READ);
  }
};
class Fast9ColorThresholdConversionPolicy
{
  protected:
    void Conversion(uint8_t* aSource, vsdk::UMat& aGraphDescr)
    {
      aGraphDescr.getMat(cv::ACCESS_WRITE | OAL_USAGE_CACHED).at<uint8_t>(0) = *aSource;
    }
};
class Fast9ColorMarkColorChannelConversionPolicy
{
  protected:
    void Conversion(uint8_t* aSource, vsdk::UMat& aGraphDescr)
    {
      aGraphDescr.getMat(cv::ACCESS_WRITE | OAL_USAGE_CACHED).at<uint8_t>(0) = *aSource;
    }
};





class Fast9ColorOutputConversionPolicy
{
protected:
  void Conversion(cv::UMat* aSink, vsdk::UMat& aGraphDescr)
  {
    cv::UMat lUMat = (cv::UMat)aGraphDescr;
    *aSink = lUMat.clone();
  }
};


}  /* namespace apugen */
#endif /* APUFAST9COLORPROCESSCONTROLLERPOLICY_HPP */
