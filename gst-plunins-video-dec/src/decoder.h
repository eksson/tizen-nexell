#include <gst/gst.h>
#include <gst/base/gstbasetransform.h>
#include <nx_video_api.h>
#include <gstnxvideodec.h>
#include <videodev2_nxp_media.h>

#ifndef __DECODER_H__
#define __DECODER_H__

G_BEGIN_DECLS
#define	VID_OUTPORT_MIN_BUF_CNT					12      // Max Avaiable Frames
#define	VID_OUTPORT_MIN_BUF_CNT_H264_UNDER720P	22      // ~720p
#define	VID_OUTPORT_MIN_BUF_CNT_H264_1080P		12      // 1080p
#define	MAX_INPUT_BUF_SIZE		(1024*1024*4)
//////////////////////////////////////////////////////////////////////////////
//
#define	NX_MAX_WIDTH		1920
#define	NX_MAX_HEIGHT		1088
#define	NX_MAX_BUF			32
    enum
{
  DEC_INIT_ERR = -1,
  DEC_ERR = -2,
};

enum
{
  H264_PARSE_ALIGN_NONE = 0,
  H264_PARSE_ALIGN_NAL,
  H264_PARSE_ALIGN_AU
};

typedef enum
{
  NX_H264_STREAM_UNKNOWN,
  NX_H264_STREAM_AVCC,
  NX_H264_STREAM_ANNEXB,
} NX_H264_STREAM_TYPE;

typedef struct
{
  NX_H264_STREAM_TYPE eStreamType;
  gint profileIndication;
  gint compatibleProfile;
  gint levelIndication;
  gint nalLengthSize;           //      for AVCC Type stream
  gint numSps;
  gint numPps;
  guint8 spsppsData[2048];
  gint spsppsSize;
} NX_AVCC_TYPE;

struct OutBufferTimeInfo
{
  gint64 timestamp;
  guint flag;
};

struct _NX_VDEC_SEMAPHORE
{
  guint value;
  pthread_cond_t cond;
  pthread_mutex_t mutex;
};

typedef struct _NX_VIDEO_DEC_STRUCT NX_VIDEO_DEC_STRUCT;
typedef struct _NX_VDEC_SEMAPHORE NX_VDEC_SEMAPHORE;

struct _NX_VIDEO_DEC_STRUCT
{
  // input stream informations
  gint width;
  gint height;
  guint fpsNum;
  guint fpsDen;

  // decoder
  NX_V4L2DEC_HANDLE hCodec;
  gboolean bInitialized;
  gint codecType;
  guint8 *pExtraData;
  gint extraDataSize;
  gint bufferCountActual;
  gint minRequiredFrameBuffer;
  gboolean bFlush;
  gboolean bNeedKey;
  gboolean bNeedIframe;
  gint imgPlaneNum;
  gint pos;
  gint size;

  //      Temporal Buffer
  guint8 *pTmpStrmBuf;
  gint tmpStrmBufSize;
  gint tmpStrmBufIndex;
  //      Output Timestamp
  struct OutBufferTimeInfo outTimeStamp[NX_MAX_BUF];
  gint inFlag;
  gint outFlag;
  //
  //      Codec Specific Informations
  //
  //      for H.264
  NX_AVCC_TYPE *pH264Info;
  gint h264Alignment;

  // Input to two frame NX_V4l2DecDecodeFrame() after seek(flush)
  gint frameCount;

  gboolean bIsFlush;

  NX_VDEC_SEMAPHORE *pSem;
#ifdef TIZEN_FEATURE_ARTIK530
  GstBuffer *codec_data;
#endif
};
//
//////////////////////////////////////////////////////////////////////////////

//Find Codec Matching Codec Information
gint FindCodecInfo (GstVideoCodecState * pState,
    NX_VIDEO_DEC_STRUCT * pDecHandle);
gboolean GetExtraInfo (NX_VIDEO_DEC_STRUCT * pDecHandle, guint8 * pCodecData,
    gint codecDataSize);

//Video Decoder
NX_VIDEO_DEC_STRUCT *OpenVideoDec ();
gint InitVideoDec (NX_VIDEO_DEC_STRUCT * pDecHandle);
gint VideoDecodeFrame (NX_VIDEO_DEC_STRUCT * pDecHandle, GstBuffer * pGstBuf,
    NX_V4L2DEC_OUT * pDecOut, gboolean bKeyFrame);
void CloseVideoDec (NX_VIDEO_DEC_STRUCT * pDecHandle);

gint DisplayDone (NX_VIDEO_DEC_STRUCT * pDecHandle, gint v4l2BufferIdx);
gint GetTimeStamp (NX_VIDEO_DEC_STRUCT * pDecHandle, gint64 * pTimestamp);
gint CopyImageToBufferYV12 (uint8_t * pSrcY, uint8_t * pSrcU, uint8_t * pSrcV,
    uint8_t * pDst, uint32_t strideY, uint32_t strideUV, uint32_t width,
    uint32_t height);

//
//      Semaphore functions for output buffer.
//
NX_VDEC_SEMAPHORE *VDecSemCreate (int init);
void VDecSemDestroy (NX_VDEC_SEMAPHORE * pSem);
gboolean VDecSemPend (NX_VDEC_SEMAPHORE * pSem);
gboolean VDecSemPost (NX_VDEC_SEMAPHORE * pSem);
gboolean VDecSemSignal (NX_VDEC_SEMAPHORE * pSem);

G_END_DECLS
#endif //__DECODER_H__
