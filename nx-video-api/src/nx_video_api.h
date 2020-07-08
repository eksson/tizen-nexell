/*
 *	Copyright (C) 2016 Nexell Co. All Rights Reserved
 *	Nexell Co. Proprietary & Confidential
 *
 *	NEXELL INFORMS THAT THIS CODE AND INFORMATION IS PROVIDED "AS IS" BASE
 *  AND	WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING
 *  BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS
 *  FOR A PARTICULAR PURPOSE.
 *
 *	File		: nx_video_api.h
 *	Brief		: V4L2 Video En/Decoder
 *	Author		: SungWon Jo (doriya@nexell.co.kr)
 *	History		: 2016.04.25 : Create
 */

#ifndef __NX_VIDEO_API_H__
#define __NX_VIDEO_API_H__

#include <stdint.h>
#include <nx_video_alloc.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define MAX_FRAME_BUFFER_NUM		32
#define MAX_IMAGE_WIDTH				1920
#define MAX_IMAGE_HEIGHT			1088


  typedef struct NX_V4L2ENC_INFO *NX_V4L2ENC_HANDLE;
  typedef struct NX_V4L2DEC_INFO *NX_V4L2DEC_HANDLE;

  enum
  {
    PIC_TYPE_I = 0,
    PIC_TYPE_P = 1,
    PIC_TYPE_B = 2,
    PIC_TYPE_VC1_BI = 3,
    PIC_TYPE_SKIP = 5,
    PIC_TYPE_IDR = 6,
#if 0
    /* TBD */
    PIC_TYPE_VC1_B = 3,
    PIC_TYPE_D = 3,             /* D picture in mpeg2, and is only composed of DC codfficients */
    PIC_TYPE_S = 3,             /* S picture in mpeg4, and is an acronym of Sprite. and used for GMC */
    PIC_TYPE_MP4_P_SKIP_NOT_CODED = 4,  /* Not Coded P Picture at mpeg4 packed mode */
#endif
    PIC_TYPE_UNKNOWN = 0xff,
  };

  enum
  {
    NONE_FIELD = 0,
    FIELD_INTERLACED = 1,
    TOP_FIELD_FIRST = 2,
    BOTTOM_FIELD_FIRST = 3,
  };

  enum
  {
    DECODED_FRAME = 0,
    DISPLAY_FRAME = 1
  };

  typedef enum
  {
    VID_CHG_KEYFRAME = (1 << 1),        /* Key frame interval */
    VID_CHG_BITRATE = (1 << 2), /* Bit Rate */
    VID_CHG_FRAMERATE = (1 << 3),       /* Frame Rate */
    VID_CHG_INTRARF = (1 << 4), /* Intra Refresh */
  } VID_ENC_CHG_PARA_E;

  typedef struct
  {
    uint32_t dispLeft;          /* Specifies the x-coordinate of the upper-left corner of the frame memory */
    uint32_t dispTop;           /* Specifies the y-coordinate of the upper-left corner of the frame memory */
    uint32_t dispRight;         /* Specifies the x-coordinate of the lower-right corner of the frame memory */
    uint32_t dispBottom;        /* Specifies the y-coordinate of the lower-right corner of the frame memory */
  } IMG_DISP_INFO;

  typedef struct tNX_V4L2ENC_PARA
  {
    int32_t width;              /* Width of image */
    int32_t height;             /* Height of image */
    int32_t keyFrmInterval;     /* Size of key frame interval */
    int32_t fpsNum;             /* Frame per second */
    int32_t fpsDen;

    uint32_t profile;

    /* Rate Control Parameters (They are valid only when CBR.) */
    uint32_t bitrate;           /* Target bitrate in bits per second */
    int32_t maximumQp;          /* Maximum quantization parameter in CBR */
    /* In MPEG-4/H.263 mode, available range is 3 to 31. In H.264 mode, the allowed range is 13 to 51. */
    int32_t disableSkip;        /* Disable rate control automatic skip frame */
    int32_t RCDelay;            /* Reference decoder initial buffer removal delay in mili-second(ms) */
    /* Valid value is 0 ~ 0x7FFF. */
    /* 0 does not check reference decoder buffer delay constraint. */
    uint32_t rcVbvSize;         /* Reference decoder buffer size in bits */
    /* This value is ignored if RCDelay is 0. */
    /* Valid value is 0 ~ 0x7FFFFFFF. */
    /* 0 dose not check Reference decoder buffer size constraint. */
    int32_t gammaFactor;        /* A gamma is the smoothing factor in the estimation. A value for gamma is factor * 32768, factor value is selected from the 0 <= factor >= 1. */
    /* If the factor value getting close to 0, Qp will be changed slowly. If the factor value getting close to 1, Qp will be chgnged quickly. */
    /* Default gamma value is 0.75 * 32768. */
    int32_t initialQp;          /* Initial quantization parameter */

    int32_t numIntraRefreshMbs; /* Intra MB refresh number(Cyclic Intra Refresh) */
    /* It must be less than encoded Mbs(width * height / 256) */
    int32_t searchRange;        /* search range of motion estimaiton(0 : 128 x 64, 1 : 64 x 32, 2 : 32 x 16, 3 : 16 x 16) */

    /* for H.264 Encoder */
    int32_t enableAUDelimiter;  /* Insert Access Unit Delimiter before NAL unit. */

    uint32_t imgFormat;         /* Fourcc of Input Image */
    uint32_t imgBufferNum;      /* Number of Input Image Buffer */
    uint32_t imgPlaneNum;       /* Number of Input Image Plane */
	  NX_VID_MEMORY_INFO*	pImage;	/* Frame Buffer of Input Image */

    /* for JPEG Specific Parameter */
    int32_t rotAngle;
    int32_t mirDirection;       /* 0 : not mir, 1 : horizontal mir, 2 : vertical mir, 3 : horizontal & vertical mir */

    int32_t jpgQuality;         /* 1 ~ 100 */
  } NX_V4L2ENC_PARA;

  typedef struct tNX_V4L2ENC_IN
  {
    NX_VID_MEMORY_HANDLE pImage;        /* Original captured frame's pointer */
    int32_t imgIndex;
    uint64_t timeStamp;         /* Time stamp */
    int32_t forcedIFrame;       /* Flag of forced intra frame */
    int32_t forcedSkipFrame;    /* Flag of forced skip frame */
    int32_t quantParam;         /* User quantization parameter(It is valid only when VBR.) */
  } NX_V4L2ENC_IN;

  typedef struct tNX_V4L2ENC_OUT
  {
    uint8_t *strmBuf;           /* compressed stream's pointer */
    int32_t strmSize;           /* compressed stream's size */
    int32_t frameType;          /* Frame type */
    NX_VID_MEMORY_INFO reconImg;        /* TBD. Reconstructed image's pointer */
  } NX_V4L2ENC_OUT;

  typedef struct tNX_V4L2ENC_CHG_PARA
  {
    int32_t chgFlg;
    int32_t keyFrmInterval;     /* Size of key frame interval */
    int32_t bitrate;            /* Target bitrate in bits/second */
    int32_t fpsNum;             /* Frame per second */
    int32_t fpsDen;
    int32_t disableSkip;        /* Disable skip frame mode */
    int32_t numIntraRefreshMbs; /* Intra MB refresh number(Cyclic Intra Refresh) */
  } NX_V4L2ENC_CHG_PARA;

  typedef struct tNX_V4L2DEC_SEQ_IN
  {
    uint32_t imgFormat;         /* Fourcc for Decoded Image */
    uint32_t imgPlaneNum;       /* Number of Input Image Plane */

    uint8_t *seqBuf;            /* Sequence header's pointer */
    int32_t seqSize;            /* Sequence header's size */

    uint64_t timeStamp;

    int32_t width;
    int32_t height;

    /* for External Buffer(optional) */
    NX_VID_MEMORY_HANDLE *pMemHandle;   /* Frame buffer for external buffer mode */

    int32_t numBuffers;         /* Internal buffer mode : number of extra buffer */
    /* External buffer mode : number of external frame buffer */

    /* for JPEG Decoder */
    int32_t thumbnailMode;      /* 0 : jpeg mode, 1 : thumbnail mode */

  	/* for Video Out Reorder */
	  /* Used when there is no B-Frame. */
	  int32_t disableVideoOutReorder;	/* 0 : EnableVideoOutReorder, 1 : DisableVideoOutReorder */
  } NX_V4L2DEC_SEQ_IN;

  typedef struct tNX_V4L2DEC_SEQ_OUT
  {
    int32_t minBuffers;         /* Needed minimum number for decoder */
    int32_t width;
    int32_t height;
    int32_t interlace;

    int32_t frameRateNum;       /* Frame Rate Numerator */
    int32_t frameRateDen;       /* Frame Rate Denominator (-1 : no information) */

    int32_t imgFourCC;          /* FourCC according to decoded image type */                // need delete
    int32_t thumbnailWidth;     /* Width of thumbnail image */
    int32_t thumbnailHeight;    /* Height of thumbnail image */

    uint32_t usedByte;

    IMG_DISP_INFO dispInfo;
  } NX_V4L2DEC_SEQ_OUT;

  typedef struct tNX_V4L2DEC_IN
  {
    uint8_t *strmBuf;           /* A compressed stream's pointer */
    int32_t strmSize;           /* A compressed stream's size */
    uint64_t timeStamp;         /* Time stamp */
    int32_t eos;

    /* for JPEG Decoder */
    int32_t downScaleWidth;     /* 0 : No scaling, 1 : 1/2 down scaling, 2 : 1/4 down scaling, 3 : 1/8 down scaling */
    int32_t downScaleHeight;    /* 0 : No scaling, 1 : 1/2 down scaling, 2 : 1/4 down scaling, 3 : 1/8 down scaling      */
  } NX_V4L2DEC_IN;

  typedef struct tNX_V4L2DEC_OUT
  {
    NX_VID_MEMORY_INFO hImg;    /* Decoded frame's pointer */
    IMG_DISP_INFO dispInfo;

    int32_t decIdx;             /* Decode Index */
    int32_t dispIdx;            /* Display Index */

    uint32_t usedByte;
  
    int32_t picType[2];         /* Picture Type */
    uint64_t timeStamp[2];      /* Time stamp */
    int32_t interlace[2];
    int32_t outFrmReliable_0_100[2];    /* Percentage of MB's are reliable ranging from 0[all damage] to 100 [all clear] */

  	uint32_t remainByte;
  } NX_V4L2DEC_OUT;


/*
 *	V4L2 Encoder
 */
  NX_V4L2ENC_HANDLE NX_V4l2EncOpen (uint32_t codecType);
  int32_t NX_V4l2EncClose (NX_V4L2ENC_HANDLE hEnc);
  int32_t NX_V4l2EncInit (NX_V4L2ENC_HANDLE hEnc, NX_V4L2ENC_PARA * pEncPara);
  int32_t NX_V4l2EncGetSeqInfo (NX_V4L2ENC_HANDLE hEnc, uint8_t ** ppSeqBuf,
      int32_t * iSeqSize);
  int32_t NX_V4l2EncEncodeFrame (NX_V4L2ENC_HANDLE hEnc, NX_V4L2ENC_IN * pEncIn,
      NX_V4L2ENC_OUT * pEncOut);
  int32_t NX_V4L2EncChangeParameter (NX_V4L2ENC_HANDLE hEnc,
      NX_V4L2ENC_CHG_PARA * pChgPara);


/*
 *	V4L2 Decoder
 */
  NX_V4L2DEC_HANDLE NX_V4l2DecOpen (uint32_t codecType);
  int32_t NX_V4l2DecClose (NX_V4L2DEC_HANDLE hDec);
  int32_t NX_V4l2DecParseVideoCfg (NX_V4L2DEC_HANDLE hDec,
      NX_V4L2DEC_SEQ_IN * pSeqIn, NX_V4L2DEC_SEQ_OUT * pSeqOut);
  int32_t NX_V4l2DecInit (NX_V4L2DEC_HANDLE hDec, NX_V4L2DEC_SEQ_IN * pSeqIn);
  int32_t NX_V4l2DecDecodeFrame (NX_V4L2DEC_HANDLE hDec, NX_V4L2DEC_IN * pDecIn,
      NX_V4L2DEC_OUT * pDecOut);
  int32_t NX_V4l2DecClrDspFlag (NX_V4L2DEC_HANDLE hDec,
      NX_VID_MEMORY_HANDLE hFrameBuf, int32_t iFrameIdx);
  int32_t NX_V4l2DecFlush (NX_V4L2DEC_HANDLE hDec);
  int32_t NX_DecGetFrameType (NX_V4L2DEC_HANDLE hDec, NX_V4L2DEC_IN * pDecIn,
      uint32_t codecType, int32_t * piFrameType);


#ifdef __cplusplus
}
#endif

#endif                          /* __NX_VIDEO_API_H__ */