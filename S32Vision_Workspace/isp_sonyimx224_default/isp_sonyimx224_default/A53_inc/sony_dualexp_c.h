#pragma once

/*****************************************************************************
* consts
*****************************************************************************/

enum {
  FDMA_IX_ISP_UYVY_OUTPUT_SONY_DUALEXP,
  FDMA_IX_END_SONY_DUALEXP
};

/*****************************************************************************
* exported variables
*****************************************************************************/
extern char sequencer_sony_dualexp_srec[];
extern char kmem_sony_dualexp_srec[];
extern SEQ_Head_Ptr_t   gpGraph_sony_dualexp[];
extern GraphMetadata_t  gGraphMetadata_sony_dualexp;

extern SEQ_Buf_t	ConvertOut_Buffer_0_sony_dualexp;
extern SEQ_Buf_t	FastDMA_Buffer_0_sony_dualexp;
extern SEQ_Buf_t	Vignetting_Buffer_0_sony_dualexp;
extern SEQ_Buf_t	Debayer_G_Buffer_0_sony_dualexp;
extern SEQ_Buf_t	Debayer_G_Buffer_1_sony_dualexp;
extern SEQ_Buf_t	Debayer_BR_Buffer_2_sony_dualexp;
extern SEQ_Buf_t	Debayer_BR_Buffer_1_sony_dualexp;
extern SEQ_Buf_t	Debayer_BR_Buffer_0_sony_dualexp;
extern SEQ_Buf_t	RGB_2_YUV_Buffer_0_sony_dualexp;
extern SEQ_Buf_t	RGB_2_YUV_Buffer_1_sony_dualexp;
extern SEQ_Buf_t	RGB_2_YUV_Buffer_2_sony_dualexp;
extern SEQ_Buf_t	CAM0_Buffer_0_sony_dualexp;
extern SEQ_Buf_t	ChGain_Exposure_Buffer_0_sony_dualexp;
extern SEQ_Buf_t	DenoiseFilter_Buffer_0_sony_dualexp;

