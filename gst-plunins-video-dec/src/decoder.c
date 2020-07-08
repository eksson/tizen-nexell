#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string.h>
#include <stdio.h>
#include <linux/videodev2.h>

#include "decoder.h"

#include "gstnxvideodec.h"

#define	MAX_OUTPUT_BUF	12

static gint ParseH264Info (guint8 * pData, gint size, NX_AVCC_TYPE * pH264Info);
static gint ParseAvcStream (guint8 * pInBuf, gint inSize, gint nalLengthSize,
    unsigned char *pBuffer, gint * pIsKey);
static gint InitializeCodaVpu (NX_VIDEO_DEC_STRUCT * pHDec, guint8 * pInitBuf,
    gint initBufSize);
static gint FlushDecoder (NX_VIDEO_DEC_STRUCT * pNxVideoDecHandle);
static gint Initialize (NX_VIDEO_DEC_STRUCT * pHDec, GstBuffer * pGstBuf,
    NX_V4L2DEC_OUT * pDecOut, gboolean bKeyFrame, guint8 * pInBuf, gint inSize,
    gint64 timestamp, NX_AVCC_TYPE * h264Info);
//TimeStamp
static void InitVideoTimeStamp (NX_VIDEO_DEC_STRUCT * hDec);
static void PushVideoTimeStamp (NX_VIDEO_DEC_STRUCT * hDec, gint64 timestamp,
    guint flag);
static gint PopVideoTimeStamp (NX_VIDEO_DEC_STRUCT * hDec, gint64 * pTimestamp,
    guint * pFlag);

//
//                      Find Codec Matching Codec Information
//
gint
FindCodecInfo (GstVideoCodecState * pState, NX_VIDEO_DEC_STRUCT * pDecHandle)
{
  guint codecType = -1;
  GstStructure *pStructure = gst_caps_get_structure (pState->caps, 0);;
  const gchar *pMime = gst_structure_get_name (pStructure);

  FUNC_IN ();

  pDecHandle->width = GST_VIDEO_INFO_WIDTH (&pState->info);
  pDecHandle->height = GST_VIDEO_INFO_HEIGHT (&pState->info);
  pDecHandle->fpsNum = GST_VIDEO_INFO_FPS_N (&pState->info);
  pDecHandle->fpsDen = GST_VIDEO_INFO_FPS_D (&pState->info);

  if (0 == pDecHandle->fpsNum) {
    pDecHandle->fpsNum = 30;
    pDecHandle->fpsDen = 1;
  }

  GST_LOG ("mime type = %s\n", pMime);

  // H.264
  if (!strcmp (pMime, "video/x-h264")) {
    codecType = V4L2_PIX_FMT_H264;
  }
  // H.263
  else if (!strcmp (pMime, "video/x-h263")) {
    codecType = V4L2_PIX_FMT_H263;
  }
  // xvid
  else if (!strcmp (pMime, "video/x-xvid")) {
    codecType = V4L2_PIX_FMT_MPEG4;
  }
  // mpeg 2 & 4
  else if (!strcmp (pMime, "video/mpeg")) {
    gint mpegVer = 0;
    gst_structure_get_int (pStructure, "mpegversion", &mpegVer);
    if ((2 == mpegVer) || (1 == mpegVer)) {
      codecType = V4L2_PIX_FMT_MPEG2;
    } else if (4 == mpegVer) {
      codecType = V4L2_PIX_FMT_MPEG4;
    }
  }
  // divx
  else if (!strcmp (pMime, "video/x-divx")) {
    gint divxVer = 0;
    gst_structure_get_int (pStructure, "divxversion", &divxVer);

    if (3 == divxVer) {
      codecType = V4L2_PIX_FMT_DIV3;
    } else if (4 == divxVer) {
      codecType = V4L2_PIX_FMT_DIV4;
    } else if (5 == divxVer) {
      codecType = V4L2_PIX_FMT_DIV5;
    } else if (6 == divxVer) {
      codecType = V4L2_PIX_FMT_DIV6;
    }
  }
  // msmpeg
  else if (!strcmp (pMime, "video/x-msmpeg")) {
    gint msMpegVer = 0;
    gst_structure_get_int (pStructure, "msmpegversion", &msMpegVer);
    if (43 == msMpegVer) {
      codecType = V4L2_PIX_FMT_DIV3;
    }
  }

  if (codecType == -1) {
    GST_ERROR ("out of profile or not supported video codec.(mime_type=%s)\n",
        pMime);
  }

  if (pDecHandle->width > NX_MAX_WIDTH || pDecHandle->height > NX_MAX_HEIGHT)
    goto error_outofrange;

  FUNC_OUT ();

  return codecType;

error_outofrange:
  GST_ERROR ("out of resolution for %s.(Max %dx%d, In %dx%d )\n", pMime,
      NX_MAX_WIDTH, NX_MAX_HEIGHT, pDecHandle->width, pDecHandle->height);
  return -1;
}

gboolean
GetExtraInfo (NX_VIDEO_DEC_STRUCT * pDecHandle, guint8 * pCodecData,
    gint codecDataSize)
{
  if (codecDataSize > 0 && pCodecData) {
    if (pDecHandle->codecType == V4L2_PIX_FMT_H264) {
      if (pDecHandle->pH264Info) {
        g_free (pDecHandle->pH264Info);
      }
      pDecHandle->pH264Info = (NX_AVCC_TYPE *) g_malloc (sizeof (NX_AVCC_TYPE));
      memset (pDecHandle->pH264Info, 0, sizeof (NX_AVCC_TYPE));
      // H264(AVC)
      if (ParseH264Info (pCodecData, codecDataSize, pDecHandle->pH264Info) != 0) {
        GST_ERROR ("Error unsupported h264 stream!\n");
        return FALSE;
      } else {
        // Debugging
        GST_DEBUG ("NumSps = %d, NumPps = %d, type = %s\n",
            pDecHandle->pH264Info->numSps,
            pDecHandle->pH264Info->numPps,
            (pDecHandle->pH264Info->eStreamType ==
                NX_H264_STREAM_AVCC) ? "avcC type" : "AnnexB type");
      }
    } else {
      memcpy (pDecHandle->pExtraData, pCodecData, codecDataSize);
    }
  } else {
    GST_LOG ("Codec_data not exist.\n");
  }

  return TRUE;
}

NX_VIDEO_DEC_STRUCT *
OpenVideoDec ()
{
  NX_VIDEO_DEC_STRUCT *pDecHandle = NULL;

  FUNC_IN ();

  pDecHandle = g_malloc (sizeof (NX_VIDEO_DEC_STRUCT));

  if (NULL == pDecHandle) {
    GST_ERROR ("%s(%d) Create VideoHandle failed.\n", __FILE__, __LINE__);
    return NULL;
  }

  memset (pDecHandle, 0, sizeof (NX_VIDEO_DEC_STRUCT));

  FUNC_OUT ();

  return pDecHandle;
}

gint
InitVideoDec (NX_VIDEO_DEC_STRUCT * pDecHandle)
{
  gint ret = 0;
  FUNC_IN ();

  pDecHandle->hCodec = NX_V4l2DecOpen (pDecHandle->codecType);
  if (NULL == pDecHandle->hCodec) {
    GST_ERROR ("%s(%d) NX_V4l2DecOpen() failed.\n", __FILE__, __LINE__);
    return -1;
  }

  pDecHandle->pTmpStrmBuf = g_malloc (MAX_INPUT_BUF_SIZE);
  pDecHandle->tmpStrmBufSize = MAX_INPUT_BUF_SIZE;
  pDecHandle->tmpStrmBufIndex = 0;

  InitVideoTimeStamp (pDecHandle);

  pDecHandle->bNeedIframe = TRUE;

  FUNC_OUT ();

  return ret;
}

gint
VideoDecodeFrame (NX_VIDEO_DEC_STRUCT * pDecHandle, GstBuffer * pGstBuf,
    NX_V4L2DEC_OUT * pDecOut, gboolean bKeyFrame)
{
  NX_VIDEO_DEC_STRUCT *pHDec = pDecHandle;
  guint8 *pInBuf = NULL;
  GstMapInfo mapInfo;
  gint inSize = 0;
  NX_AVCC_TYPE *h264Info = NULL;
  gint isKey = 0;
  guint8 *pDecBuf = NULL;
  gint decBufSize = 0;
  gint ret = 0;
  gint64 timestamp = 0;
  NX_V4L2DEC_IN decIn;

  FUNC_IN ();

  if (pHDec->bFlush) {
    FlushDecoder (pHDec);
    pHDec->bFlush = FALSE;
    pHDec->bNeedKey = TRUE;
    pHDec->bNeedIframe = TRUE;
    pHDec->bIsFlush = TRUE;
  }

  if (pHDec->bNeedKey) {
    if (FALSE == bKeyFrame) {
      pDecOut->dispIdx = -1;
      return ret;
    }
    pHDec->bNeedKey = FALSE;
  }

  h264Info = pHDec->pH264Info;
  gst_buffer_map (pGstBuf, &mapInfo, GST_MAP_READ);
  pInBuf = mapInfo.data;
  inSize = gst_buffer_get_size (pGstBuf);

  // Push Input Time Stamp
  if (GST_BUFFER_PTS_IS_VALID (pGstBuf)) {
    PushVideoTimeStamp (pHDec, GST_BUFFER_PTS (pGstBuf),
        GST_BUFFER_FLAGS (pGstBuf));
    timestamp = GST_BUFFER_PTS (pGstBuf);
  } else if (GST_BUFFER_DTS_IS_VALID (pGstBuf)) {
    PushVideoTimeStamp (pHDec, GST_BUFFER_DTS (pGstBuf),
        GST_BUFFER_FLAGS (pGstBuf));
    timestamp = GST_BUFFER_DTS (pGstBuf);
  }

  if (FALSE == pHDec->bInitialized) {
    ret =
        Initialize (pHDec, pGstBuf, pDecOut, bKeyFrame, pInBuf, inSize,
        timestamp, h264Info);

    if ((ret == DEC_ERR) && (pDecOut->dispIdx = -1)) {
      goto VideoDecodeFrame_Exit;
    } else if ((ret == DEC_INIT_ERR) || (ret == DEC_ERR)) {
      goto VideoDecodeFrame_Exit;
    } else if ((ret == 0) && (pDecOut->dispIdx = -1)) {
      if (0 == pHDec->extraDataSize) {
        if (((GST_BUFFER_FLAG_DISCONT | GST_BUFFER_FLAG_DELTA_UNIT) ==
                GST_BUFFER_FLAGS (pGstBuf)) && (0 == bKeyFrame)) {
          return ret;
        }
      } else {
        goto VideoDecodeFrame_Exit;
      }
    }
  } else {
    if (pHDec->bIsFlush) {
      // Input to two frame NX_V4l2DecDecodeFrame() after seek(flush)
      pHDec->frameCount++;

      if (pHDec->codecType == V4L2_PIX_FMT_H264) {
        if (pHDec->h264Alignment == H264_PARSE_ALIGN_NAL) {
          pDecBuf = pHDec->pTmpStrmBuf + pHDec->tmpStrmBufIndex;
          decBufSize = ParseAvcStream (pInBuf, inSize, 4, pDecBuf, &isKey);
          pHDec->tmpStrmBufIndex = pHDec->tmpStrmBufIndex + decBufSize;
        } else if ((h264Info) && (h264Info->eStreamType == NX_H264_STREAM_AVCC)) {
          pDecBuf = pHDec->pTmpStrmBuf + pHDec->tmpStrmBufIndex;
          decBufSize =
              ParseAvcStream (pInBuf, inSize, h264Info->nalLengthSize, pDecBuf,
              &isKey);
          pHDec->tmpStrmBufIndex = pHDec->tmpStrmBufIndex + decBufSize;
        }
        // Annex B Type
        else {
          memcpy (pHDec->pTmpStrmBuf + pHDec->tmpStrmBufIndex, pInBuf, inSize);
          pHDec->tmpStrmBufIndex = pHDec->tmpStrmBufIndex + inSize;
        }
      } else {
        memcpy (pHDec->pTmpStrmBuf + pHDec->tmpStrmBufIndex, pInBuf, inSize);
        pHDec->tmpStrmBufIndex = pHDec->tmpStrmBufIndex + inSize;

      }
      if (pHDec->frameCount >= 2) {
        pHDec->bIsFlush = FALSE;
        pHDec->frameCount = 0;
        pDecBuf = pHDec->pTmpStrmBuf;
        decBufSize = pHDec->tmpStrmBufIndex;
        pHDec->tmpStrmBufIndex = 0;
      } else {
        ret = 0;
        pDecOut->dispIdx = -1;
        goto VideoDecodeFrame_Exit;
      }
    } else {
      if (pHDec->codecType == V4L2_PIX_FMT_H264) {
        if (pHDec->h264Alignment == H264_PARSE_ALIGN_NAL) {
          pDecBuf = pHDec->pTmpStrmBuf;
          decBufSize = ParseAvcStream (pInBuf, inSize, 4, pDecBuf, &isKey);
        } else if ((h264Info) && (h264Info->eStreamType == NX_H264_STREAM_AVCC)) {
          pDecBuf = pHDec->pTmpStrmBuf;
          decBufSize =
              ParseAvcStream (pInBuf, inSize, h264Info->nalLengthSize, pDecBuf,
              &isKey);
        }
        // Annex B Type
        else {
          pDecBuf = pInBuf;
          decBufSize = inSize;
        }
      } else {
        pDecBuf = pInBuf;
        decBufSize = inSize;
      }
    }

    decIn.strmBuf = pDecBuf;
    decIn.strmSize = decBufSize;
    decIn.timeStamp = timestamp;
    decIn.eos = 0;
    VDecSemPend (pHDec->pSem);
    ret = NX_V4l2DecDecodeFrame (pHDec->hCodec, &decIn, pDecOut);

    if ((0 == ret) && (0 <= pDecOut->dispIdx)) {
      if ((TRUE == pHDec->bNeedIframe)
          && (PIC_TYPE_I != pDecOut->picType[DISPLAY_FRAME])) {
        NX_V4l2DecClrDspFlag (pHDec->hCodec, NULL, pDecOut->dispIdx);
        VDecSemPost (pHDec->pSem);
        ret = DEC_ERR;
        goto VideoDecodeFrame_Exit;
      } else {
        pHDec->bNeedIframe = FALSE;
      }
    }

    if ((0 != ret) || (0 > pDecOut->dispIdx)) {
      VDecSemPost (pHDec->pSem);
    }

    if (0 != ret) {
      GST_ERROR ("NX_V4l2DecDecodeFrame!!!!, ret = %d\n", ret);
      ret = DEC_ERR;
    }
  }
VideoDecodeFrame_Exit:
  gst_buffer_unmap (pGstBuf, &mapInfo);

  FUNC_OUT ();

  return ret;
}

void
CloseVideoDec (NX_VIDEO_DEC_STRUCT * pDecHandle)
{
  if (pDecHandle == NULL) {
    g_free (pDecHandle);
    GST_ERROR ("pDecHandle is null\n");
    return;
  }
  if (pDecHandle->hCodec) {
    NX_V4l2DecClose (pDecHandle->hCodec);
    pDecHandle->hCodec = NULL;
  }

  if (pDecHandle->pExtraData) {
    g_free (pDecHandle->pExtraData);
    pDecHandle->pExtraData = NULL;
    pDecHandle->extraDataSize = 0;
  }

  if (pDecHandle->pH264Info) {
    g_free (pDecHandle->pH264Info);
    pDecHandle->pH264Info = NULL;
  }

  if (pDecHandle->pTmpStrmBuf) {
    g_free (pDecHandle->pTmpStrmBuf);
    pDecHandle->pTmpStrmBuf = NULL;
  }

  g_free (pDecHandle);
}

gint
DisplayDone (NX_VIDEO_DEC_STRUCT * pDecHandle, gint v4l2BufferIdx)
{
  FUNC_IN ();

  if (pDecHandle->hCodec && (v4l2BufferIdx >= 0)) {
    NX_V4l2DecClrDspFlag (pDecHandle->hCodec, NULL, v4l2BufferIdx);
    VDecSemPost (pDecHandle->pSem);
  }

  FUNC_OUT ();

  return 0;
}

gint
GetTimeStamp (NX_VIDEO_DEC_STRUCT * pDecHandle, gint64 * pTimestamp)
{
  gint ret = 0;
  guint flag;

  ret = PopVideoTimeStamp (pDecHandle, pTimestamp, &flag);

  return ret;
}

// Copy Image YV12 to General YV12
gint
CopyImageToBufferYV12 (uint8_t * pSrcY, uint8_t * pSrcU, uint8_t * pSrcV,
    uint8_t * pDst, uint32_t strideY, uint32_t strideUV, uint32_t width,
    uint32_t height)
{
  uint32_t i;
  if (width == strideY) {
    memcpy (pDst, pSrcY, width * height);
    pDst += width * height;
  } else {
    for (i = 0; i < height; i++) {
      memcpy (pDst, pSrcY, width);
      pSrcY += strideY;
      pDst += width;
    }
  }

  width /= 2;
  height /= 2;
  if (width == strideUV) {
    memcpy (pDst, pSrcU, width * height);
    pDst += width * height;
    memcpy (pDst, pSrcV, width * height);
  } else {
    for (i = 0; i < height; i++) {
      memcpy (pDst, pSrcU, width);
      pSrcU += strideUV;
      pDst += width;
    }
    for (i = 0; i < height; i++) {
      memcpy (pDst, pSrcV, width);
      pSrcV += strideUV;
      pDst += width;
    }
  }
  return 0;
}

static gint
Initialize (NX_VIDEO_DEC_STRUCT * pHDec, GstBuffer * pGstBuf,
    NX_V4L2DEC_OUT * pDecOut, gboolean bKeyFrame, guint8 * pInBuf, gint inSize,
    gint64 timestamp, NX_AVCC_TYPE * h264Info)
{
  gint seqSize = 0;
  gint ret = 0;
  guint8 *pSeqData = NULL;
  gboolean bDecode = FALSE;
  gint isKey = 0;
  gint decBufSize = 0;
  NX_V4L2DEC_IN decIn;
  guint8 *pDecBuf = pHDec->pTmpStrmBuf;

  if (0 == pHDec->extraDataSize) {
    if (((GST_BUFFER_FLAG_DISCONT | GST_BUFFER_FLAG_DELTA_UNIT) ==
            GST_BUFFER_FLAGS (pGstBuf)) && (0 == bKeyFrame)) {
      pDecOut->dispIdx = -1;
      return ret;
    }

    if (pHDec->h264Alignment == H264_PARSE_ALIGN_NAL) {
      gint size = 0;
      if (0 == GST_BUFFER_DURATION (pGstBuf)) {
        size = ParseAvcStream (pInBuf, inSize, 4, pDecBuf + pHDec->pos, &isKey);
        pHDec->size = pHDec->size + size;
        pHDec->pos = pHDec->pos + size;

        pDecOut->dispIdx = -1;
        ret = DEC_ERR;
        return ret;
      } else {
        size = ParseAvcStream (pInBuf, inSize, 4, pDecBuf + pHDec->pos, &isKey);
        pHDec->size = pHDec->size + size;
        pHDec->pos = pHDec->pos + size;
        seqSize = pHDec->pos;
        pSeqData = pDecBuf;
        bDecode = TRUE;
      }
    } else if (V4L2_PIX_FMT_DIV3 == pHDec->codecType) {
      seqSize = 0;
      pSeqData = NULL;
      bDecode = TRUE;
    } else {
      seqSize = inSize;
      pSeqData = pInBuf;
    }
  } else {
    if (V4L2_PIX_FMT_H263 == pHDec->codecType
        || V4L2_PIX_FMT_MPEG2 == pHDec->codecType) {
      memcpy (pDecBuf, pHDec->pExtraData, pHDec->extraDataSize);
      decBufSize = pHDec->extraDataSize;
      memcpy (pDecBuf + decBufSize, pInBuf, inSize);
      decBufSize += inSize;

      seqSize = decBufSize;
      pSeqData = pDecBuf;
    } else if (V4L2_PIX_FMT_H264 == pHDec->codecType) {
      if ((h264Info) && (h264Info->eStreamType == NX_H264_STREAM_AVCC)) {
        gint size;
        memcpy (pDecBuf, h264Info->spsppsData, h264Info->spsppsSize);
        decBufSize = h264Info->spsppsSize;
        size =
            ParseAvcStream (pInBuf, inSize, h264Info->nalLengthSize,
            pDecBuf + decBufSize, &isKey);
        decBufSize += size;
      }
      // Annex B Type
      else {
        memcpy (pDecBuf, pHDec->pExtraData, pHDec->extraDataSize);
        decBufSize = pHDec->extraDataSize;
        memcpy (pDecBuf + decBufSize, pInBuf, inSize);
        decBufSize += inSize;
      }
      seqSize = decBufSize;
      pSeqData = pDecBuf;
    } else {
      seqSize = pHDec->extraDataSize;
      pSeqData = pHDec->pExtraData;
      bDecode = TRUE;
    }
  }

  // Initialize VPU
  ret = InitializeCodaVpu (pHDec, pSeqData, seqSize);

  if (0 > ret) {
    GST_ERROR ("VPU initialized Failed!!!!\n");
    NX_V4l2DecClose (pHDec->hCodec);
    pHDec->hCodec = NULL;
    ret = DEC_INIT_ERR;
    return ret;
  }

  pHDec->bInitialized = TRUE;

  if (TRUE == bDecode) {
    if (pHDec->codecType == V4L2_PIX_FMT_H264) {
      if (pHDec->h264Alignment == H264_PARSE_ALIGN_NAL) {
        pDecBuf = pDecBuf + pHDec->pos - inSize;
        decBufSize = inSize;
      } else if ((h264Info) && (h264Info->eStreamType == NX_H264_STREAM_AVCC)) {
        pDecBuf = pHDec->pTmpStrmBuf;
        decBufSize =
            ParseAvcStream (pInBuf, inSize, h264Info->nalLengthSize, pDecBuf,
            &isKey);
      }
      // Annex B Type
      else {
        pDecBuf = pInBuf;
        decBufSize = inSize;
      }
    } else {
      pDecBuf = pInBuf;
      decBufSize = inSize;
    }

    decIn.strmBuf = pDecBuf;
    decIn.strmSize = decBufSize;
    decIn.timeStamp = timestamp;
    decIn.eos = 0;
    VDecSemPend (pHDec->pSem);
    ret = NX_V4l2DecDecodeFrame (pHDec->hCodec, &decIn, pDecOut);

    if ((0 == ret) && (0 <= pDecOut->dispIdx)) {
      if ((TRUE == pHDec->bNeedIframe)
          && (PIC_TYPE_I != pDecOut->picType[DISPLAY_FRAME])) {
        NX_V4l2DecClrDspFlag (pHDec->hCodec, NULL, pDecOut->dispIdx);
        VDecSemPost (pHDec->pSem);
        ret = DEC_ERR;
        return ret;
      } else {
        pHDec->bNeedIframe = FALSE;
      }
    }

    if ((0 != ret) || (0 > pDecOut->dispIdx)) {
      VDecSemPost (pHDec->pSem);
    }

    if (0 != ret) {
      GST_ERROR ("NX_V4l2DecDecodeFrame!!!!, ret = %d\n", ret);
      ret = DEC_ERR;
    }
  } else {
    ret = 0;
    pDecOut->dispIdx = -1;
  }

  return ret;
}

static gint
FlushDecoder (NX_VIDEO_DEC_STRUCT * pDecHandle)
{

  FUNC_IN ();

  InitVideoTimeStamp (pDecHandle);

  if (pDecHandle->hCodec) {
    NX_V4l2DecFlush (pDecHandle->hCodec);
  }

  FUNC_OUT ();

  return 0;
}

static gint
InitializeCodaVpu (NX_VIDEO_DEC_STRUCT * pHDec, guint8 * pSeqInfo,
    gint seqInfoSize)
{
  gint ret = -1;

  FUNC_IN ();

  if (pHDec->hCodec) {
    NX_V4L2DEC_SEQ_IN seqIn;
    NX_V4L2DEC_SEQ_OUT seqOut;
    memset (&seqIn, 0, sizeof (seqIn));
    memset (&seqOut, 0, sizeof (seqOut));
    seqIn.width = pHDec->width;
    seqIn.height = pHDec->height;
    seqIn.seqBuf = pSeqInfo;
    seqIn.seqSize = seqInfoSize;

    if (0 != (ret = NX_V4l2DecParseVideoCfg (pHDec->hCodec, &seqIn, &seqOut))) {
      GST_ERROR ("NX_V4l2DecParseVideoCfg() is failed!!, ret = %d\n", ret);
      return ret;
    }

    seqIn.width = seqOut.width;
    seqIn.height = seqOut.height;
    pHDec->bufferCountActual = seqOut.minBuffers + MAX_OUTPUT_BUF;
    seqIn.numBuffers = pHDec->bufferCountActual;
    seqIn.imgPlaneNum = pHDec->imgPlaneNum;
    seqIn.imgFormat = seqOut.imgFourCC;
    ret = NX_V4l2DecInit (pHDec->hCodec, &seqIn);

    if (0 != ret) {
      GST_ERROR ("NX_V4l2DecInit() is failed!!, ret = %d\n", ret);
    }

    pHDec->minRequiredFrameBuffer = seqOut.minBuffers;
    pHDec->pSem = VDecSemCreate (MAX_OUTPUT_BUF);
    GST_LOG
        ("<<<<<<<<<< InitializeCodaVpu(Min=%d, %dx%d) (ret = %d) >>>>>>>>>\n",
        pHDec->minRequiredFrameBuffer, seqOut.width, seqOut.height, ret);

    pHDec->frameCount = 0;
    pHDec->bIsFlush = FALSE;
  }

  FUNC_OUT ();

  return ret;
}

//
//
//                                                              H.264 Decoder
//

//
//                      avcC format
//      Name                                    Bits            Descriptions
//      ===============================================
//      CFG version                             8 bits          "1"
//      AVC porfile indication  8 bits          Profile code
//      Profile compatibility   8 bits          Compatible profile
//      AVC level indication    8 bits          Level code
//      Reserved                                6 bits          0b111111
//      Length size minus one   2 bits          Nal unit length size
//      Reserved                                3 bits          0b111
//      Num of SPS                              5 bits          Number of SPS
//      SPS length                              16bits          SPS length N
//      SPS Data                                N byts          SPS data
//      Num of PPS                              8 bits          Number of PPS
//      PPS length                              16bits          PPS length M
//      PPS Data                                M Byts          PPS data
//
static gint
ParseSpsPpsFromAVCC (unsigned char *pExtraData, gint extraDataSize,
    NX_AVCC_TYPE * pH264Info)
{

  gint length, i, pos = 0;

  FUNC_IN ();

  if (1 != pExtraData[0] || 11 > extraDataSize) {
    GST_ERROR ("Error : Invalid \"avcC\" data(%d)\n", extraDataSize);
    return -1;
  }
  // Parser "avcC" format data
  pos++;                        // Skip Version
  pH264Info->profileIndication = pExtraData[pos];
  pos++;
  pH264Info->compatibleProfile = pExtraData[pos];
  pos++;
  pH264Info->levelIndication = pExtraData[pos];
  pos++;
  pH264Info->nalLengthSize = (pExtraData[pos] & 0x03) + 1;
  pos++;

  if (100 < pH264Info->profileIndication) {
    GST_ERROR ("H264 profile too high!(%d)\n", pH264Info->profileIndication);
    return -1;
  }
  // parser spsp
  pH264Info->spsppsSize = 0;
  pH264Info->numSps = (pExtraData[pos] & 0x1f);
  pos++;

  for (i = 0; i < pH264Info->numSps; i++) {
    length = (pExtraData[pos] << 8) | pExtraData[pos + 1];
    pos += 2;
    if ((pos + length) > extraDataSize) {
      GST_ERROR ("extraData size too small(SPS)\n");
      return -1;
    }
    pH264Info->spsppsData[pH264Info->spsppsSize + 0] = 0;
    pH264Info->spsppsData[pH264Info->spsppsSize + 1] = 0;
    pH264Info->spsppsData[pH264Info->spsppsSize + 2] = 0;
    pH264Info->spsppsData[pH264Info->spsppsSize + 3] = 1;
    pH264Info->spsppsSize += 4;
    memcpy (pH264Info->spsppsData + pH264Info->spsppsSize, pExtraData + pos,
        length);
    pH264Info->spsppsSize += length;
    pos += length;
  }

  // parse pps
  pH264Info->numPps = pExtraData[pos];
  pos++;
  for (i = 0; i < pH264Info->numPps; i++) {
    length = (pExtraData[pos] << 8) | pExtraData[pos + 1];
    pos += 2;
    if ((pos + length) > extraDataSize) {
      GST_ERROR ("extraData size too small(PPS)\n");
      return -1;
    }
    pH264Info->spsppsData[pH264Info->spsppsSize + 0] = 0;
    pH264Info->spsppsData[pH264Info->spsppsSize + 1] = 0;
    pH264Info->spsppsData[pH264Info->spsppsSize + 2] = 0;
    pH264Info->spsppsData[pH264Info->spsppsSize + 3] = 1;
    pH264Info->spsppsSize += 4;
    memcpy (pH264Info->spsppsData + pH264Info->spsppsSize, pExtraData + pos,
        length);
    pH264Info->spsppsSize += length;
    pos += length;
  }

  if (1 > pH264Info->numSps || 1 > pH264Info->numPps) {
    return -1;
  }

  FUNC_OUT ();

  return 0;
}

static gint
ParseH264Info (guint8 * pData, gint size, NX_AVCC_TYPE * pH264Info)
{
  FUNC_IN ();

  if (size <= 0) {
    return -1;
  }

  if (pData[0] == 0) {
    pH264Info->eStreamType = NX_H264_STREAM_ANNEXB;
    memcpy (pH264Info->spsppsData, pData, size);
    pH264Info->spsppsSize = size;
  } else {
    pH264Info->eStreamType = NX_H264_STREAM_AVCC;
    return ParseSpsPpsFromAVCC (pData, size, pH264Info);
  }

  FUNC_OUT ();

  return 0;
}

static gint
ParseAvcStream (guint8 * pInBuf, gint inSize, gint nalLengthSize,
    unsigned char *pBuffer, gint * pIsKey)
{

  int nalLength;
  int pos = 0;
  *pIsKey = 0;

  FUNC_IN ();

  // 'avcC' format
  do {
    nalLength = 0;
    if (nalLengthSize == 4) {
      nalLength =
          pInBuf[0] << 24 | pInBuf[1] << 16 | pInBuf[2] << 8 | pInBuf[3];
    } else if (nalLengthSize == 2) {
      nalLength = pInBuf[0] << 8 | pInBuf[1];
    } else if (nalLengthSize == 3) {
      nalLength = pInBuf[0] << 16 | pInBuf[1] << 8 | pInBuf[2];
    } else if (nalLengthSize == 1) {
      nalLength = pInBuf[0];
    }

    pInBuf += nalLengthSize;
    inSize -= nalLengthSize;

    if (0 == nalLength || inSize < (int) nalLength) {
      GST_ERROR
          ("Error : avcC type nal length error (nalLength = %d, inSize=%d, nalLengthSize=%d)\n",
          nalLength, inSize, nalLengthSize);
      return -1;
    }
    // put nal start code
    pBuffer[pos + 0] = 0x00;
    pBuffer[pos + 1] = 0x00;
    pBuffer[pos + 2] = 0x00;
    pBuffer[pos + 3] = 0x01;
    pos += 4;

    if ((pInBuf[0] & 0x1f) == 0x5) {
      *pIsKey = 1;
    }

    memcpy (pBuffer + pos, pInBuf, nalLength);
    pos += nalLength;

    inSize -= nalLength;
    pInBuf += nalLength;
  } while (2 < inSize);

  FUNC_OUT ();

  return pos;
}

///////////////////////////////////////////////////////////////////////////////
static void
InitVideoTimeStamp (NX_VIDEO_DEC_STRUCT * hDec)
{
  gint i;
  for (i = 0; i < NX_MAX_BUF; i++) {
    hDec->outTimeStamp[i].flag = (gint) - 1;
    hDec->outTimeStamp[i].timestamp = (gint) 0;
  }
  hDec->inFlag = 0;
  hDec->outFlag = 0;
}

static void
PushVideoTimeStamp (NX_VIDEO_DEC_STRUCT * hDec, gint64 timestamp, guint flag)
{
  gint i = 0;
  if (-1 != timestamp) {
    hDec->inFlag++;
    if (NX_MAX_BUF <= hDec->inFlag)
      hDec->inFlag = 0;

    for (i = 0; i < NX_MAX_BUF; i++) {
      if (hDec->outTimeStamp[i].flag == (gint) - 1) {
        hDec->outTimeStamp[i].timestamp = timestamp;
        hDec->outTimeStamp[i].flag = flag;
        break;
      }
    }
  }
}

static gint
PopVideoTimeStamp (NX_VIDEO_DEC_STRUCT * hDec, gint64 * pTimestamp,
    guint * pFlag)
{
  gint i = 0;
  gint64 minTime = 0x7FFFFFFFFFFFFFFFll;
  gint minIdx = -1;
  for (i = 0; i < NX_MAX_BUF; i++) {
    if (hDec->outTimeStamp[i].flag != (guint) - 1) {
      if (minTime > hDec->outTimeStamp[i].timestamp) {
        minTime = hDec->outTimeStamp[i].timestamp;
        minIdx = i;
      }
    }
  }
  if (minIdx != -1) {
    *pTimestamp = hDec->outTimeStamp[minIdx].timestamp;
    *pFlag = hDec->outTimeStamp[minIdx].flag;
    hDec->outTimeStamp[minIdx].flag = (gint) - 1;
    return 0;
  } else {
//  GST_ERROR ("Cannot Found Time Stamp!!!\n");
    return -1;
  }
}

///////////////////////////////////////////////////////////////////////////////

//
//      Semaphore functions for output buffer.
//
NX_VDEC_SEMAPHORE *
VDecSemCreate (int init)
{
  NX_VDEC_SEMAPHORE *pSem =
      (NX_VDEC_SEMAPHORE *) g_malloc (sizeof (NX_VDEC_SEMAPHORE));
  FUNC_IN ();
  pSem->value = init;
  pthread_mutex_init (&pSem->mutex, NULL);
  pthread_cond_init (&pSem->cond, NULL);
  FUNC_OUT ();
  return pSem;
}

void
VDecSemDestroy (NX_VDEC_SEMAPHORE * pSem)
{
  FUNC_IN ();
  if (pSem) {
    pthread_mutex_destroy (&pSem->mutex);
    pthread_cond_destroy (&pSem->cond);
    g_free (pSem);
  }
  FUNC_OUT ();
}

gboolean
VDecSemPend (NX_VDEC_SEMAPHORE * pSem)
{
  FUNC_IN ();
  pthread_mutex_lock (&pSem->mutex);

  if (pSem->value == 0) {
    pthread_cond_wait (&pSem->cond, &pSem->mutex);
  }
  pSem->value--;

  pthread_mutex_unlock (&pSem->mutex);
  FUNC_OUT ();
  return TRUE;
}

gboolean
VDecSemPost (NX_VDEC_SEMAPHORE * pSem)
{
  FUNC_IN ();
  pthread_mutex_lock (&pSem->mutex);

  pSem->value++;
  pthread_cond_signal (&pSem->cond);

  pthread_mutex_unlock (&pSem->mutex);
  FUNC_OUT ();
  return TRUE;
}

gboolean
VDecSemSignal (NX_VDEC_SEMAPHORE * pSem)
{
  FUNC_IN ();
  pthread_mutex_lock (&pSem->mutex);
  pthread_cond_signal (&pSem->cond);
  pthread_mutex_unlock (&pSem->mutex);
  FUNC_OUT ();
  return TRUE;
}
