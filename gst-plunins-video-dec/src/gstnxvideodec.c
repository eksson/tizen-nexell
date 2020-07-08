/*
 * GStreamer
 * Copyright (C) 2005 Thomas Vander Stichele <thomas@apestaart.org>
 * Copyright (C) 2005 Ronald S. Bultje <rbultje@ronald.bitfreak.net>
 * Copyright (C) 2016 ray <<user@hostname.org>>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Alternatively, the contents of this file may be used under the
 * GNU Lesser General Public License Version 2.1 (the "LGPL"), in
 * which case the following provisions apply instead of the ones
 * mentioned above:
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * SECTION:element-nxvideodec
 *
 * FIXME:Describe nxvideodec here.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch -v -m fakesrc ! nxvideodec ! fakesink silent=TRUE
 * ]|
 * </refsect2>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <string.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#include <gst/video/gstvideodecoder.h>
#include <gstmmvideobuffermeta.h>
#include <linux/videodev2.h>
#include "gstnxvideodec.h"
#include <gst/allocators/gsttizenmemory.h>

// This SUPPORT_NO_MEMORY_COPY function is disabled now.
// if the video decoder mmap is supported this function, it will be enabled.
#define SUPPORT_NO_MEMORY_COPY	1
//#define CODEC_DEC_OUTPUT_DUMP	1

GST_DEBUG_CATEGORY_STATIC (gst_nxvideodec_debug_category);
#define GST_CAT_DEFAULT gst_nxvideodec_debug_category

/* prototypes */
static void gst_nxvideodec_set_property (GObject * object,
    guint property_id, const GValue * value, GParamSpec * pspec);
static void gst_nxvideodec_get_property (GObject * object,
    guint property_id, GValue * value, GParamSpec * pspec);

static gboolean gst_nxvideodec_start (GstVideoDecoder * decoder);
static gboolean gst_nxvideodec_stop (GstVideoDecoder * decoder);
static gboolean gst_nxvideodec_set_format (GstVideoDecoder * decoder,
    GstVideoCodecState * state);
static gboolean gst_nxvideodec_flush (GstVideoDecoder * decoder);
static GstFlowReturn gst_nxvideodec_handle_frame (GstVideoDecoder * decoder,
    GstVideoCodecFrame * frame);
static void nxvideodec_base_init (gpointer gclass);
static void nxvideodec_buffer_finalize (gpointer pData);

#if SUPPORT_NO_MEMORY_COPY
static void nxvideodec_get_offset_stride (gint width, gint height,
    guint8 * pSrc, gsize * pOffset, gint * pStride);
enum
{
  PROP_0,
};
#else
enum
{
  PROP_0,
  PROP_TYPE                     //0: 1:MM_VIDEO_BUFFER_TYPE_GEM
};
enum
{
  BUFFER_TYPE_NORMAL,
  BUFFER_TYPE_GEM
};
#endif

enum
{
  STOP,
  PLAY
};

#ifndef ALIGN
#define  ALIGN(X,N) ( (X+N-1) & (~(N-1)) )
#endif

struct video_meta_mmap_buffer
{
  gint v4l2BufferIdx;
  GstNxVideoDec *pNxVideoDec;
};

#define	PLUGIN_LONG_NAME		"S5P6818 H/W Video Decoder"
#define PLUGIN_DESC				"Nexell H/W Video Decoder for S5P6818, Version: 0.1.0"
#define	PLUGIN_AUTHOR			"Hyun Chul Jun <hcjun@nexell.co.kr>"

// pad templates
static GstStaticPadTemplate gst_nxvideodec_src_template =
GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("video/x-raw, "
        "format = (string) { S420 }, "
        "width = (int) [ 64, 1920 ], " "height = (int) [ 64, 1088 ] ")

    );

#ifdef CODEC_DEC_OUTPUT_DUMP /* for decoder output dump */
static inline void
decoder_output_dump (GstBuffer *buffer)
{
  int i = 0;
  char filename[100]={0};
  FILE *fp = NULL;
  tbm_surface_h surface;
  tbm_surface_info_s info;
  GstMemory *mem = gst_buffer_peek_memory (buffer, 0);
  unsigned char *temp;

  surface = gst_tizen_memory_get_surface(mem);

  tbm_surface_get_info (surface, &info);

  temp = info.planes[0].ptr;

  sprintf(filename, "/tmp/dec_output_dump_%d_%d.yuv", info.width, info.height);
  fp = fopen(filename, "ab");

  if(!fp) {
    return;
  }

  for (i = 0; i < info.height; i++) {
    fwrite(temp, info.width, 1, fp);
    temp += info.planes[0].stride;
  }

  temp = info.planes[1].ptr;

  for(i = 0; i < info.height/2 ; i++) {
    fwrite(temp, info.width/2, 1, fp);
    temp += info.planes[1].stride;
  }

  temp = info.planes[2].ptr;

  for(i = 0; i < info.height/2 ; i++) {
    fwrite(temp, info.width/2, 1, fp);
    temp += info.planes[2].stride;
  }

  fclose(fp);
}
#endif

static void
nxvideodec_base_init (gpointer gclass)
{
  GstElementClass *pElement_class = GST_ELEMENT_CLASS (gclass);
  GstCaps *pCapslist = NULL;
  GstNxVideoDecClass *pKlass = GST_NXVIDEODEC_CLASS (pElement_class);

  FUNC_IN ();

  gst_element_class_set_details_simple (pElement_class,
      PLUGIN_LONG_NAME, "Codec/Decoder/Video", PLUGIN_DESC, PLUGIN_AUTHOR);

  pCapslist = gst_caps_new_empty ();

  //      H.263
  gst_caps_append_structure (pCapslist,
      gst_structure_new ("video/x-h263",
          "variant", G_TYPE_STRING, "itu", NULL));

  //      H.264
  gst_caps_append_structure (pCapslist,
      gst_structure_new ("video/x-h264",
          "alignment", G_TYPE_STRING, "au",
          "width", GST_TYPE_INT_RANGE, 64, NX_MAX_WIDTH,
          "height", GST_TYPE_INT_RANGE, 64, NX_MAX_HEIGHT, NULL));

  //      XVID
  gst_caps_append_structure (pCapslist,
      gst_structure_new ("video/x-xvid",
          "width", GST_TYPE_INT_RANGE, 64, NX_MAX_WIDTH,
          "height", GST_TYPE_INT_RANGE, 64, NX_MAX_HEIGHT, NULL));


  //      MPEG2
  gst_caps_append_structure (pCapslist,
      gst_structure_new ("video/mpeg",
          "mpegversion", GST_TYPE_INT_RANGE, 1, 2,
          "systemstream", G_TYPE_BOOLEAN, FALSE, NULL));

  //      MPEG4
  gst_caps_append_structure (pCapslist,
      gst_structure_new ("video/mpeg",
          "mpegversion", G_TYPE_INT, 4,
          "systemstream", G_TYPE_BOOLEAN, FALSE, NULL));

  //      DIVX
  gst_caps_append_structure (pCapslist,
      gst_structure_new ("video/x-divx",
          "width", GST_TYPE_INT_RANGE, 64, NX_MAX_WIDTH,
          "height", GST_TYPE_INT_RANGE, 64, NX_MAX_HEIGHT,
          "divxversion", GST_TYPE_INT_RANGE, 3, 6, NULL));

  //      MSMPEG
  gst_caps_append_structure (pCapslist,
      gst_structure_new ("video/x-msmpeg",
          "width", GST_TYPE_INT_RANGE, 64, NX_MAX_WIDTH,
          "height", GST_TYPE_INT_RANGE, 64, NX_MAX_HEIGHT,
          "msmpegversion", G_TYPE_INT, 43, NULL));

  // pad templates
  pKlass->pSinktempl =
      gst_pad_template_new ("sink", GST_PAD_SINK, GST_PAD_ALWAYS, pCapslist);
  gst_element_class_add_pad_template (pElement_class, pKlass->pSinktempl);
  gst_element_class_add_pad_template (pElement_class,
      gst_static_pad_template_get (&gst_nxvideodec_src_template));

  FUNC_OUT ();
}

G_DEFINE_TYPE_WITH_CODE (GstNxVideoDec, gst_nxvideodec, GST_TYPE_VIDEO_DECODER,
    GST_DEBUG_CATEGORY_INIT (gst_nxvideodec_debug_category, "nxvideodec", 0,
        "debug category for nxvideodec element"));

// class initialization
static void
gst_nxvideodec_class_init (GstNxVideoDecClass * pKlass)
{
  FUNC_IN ();

  GObjectClass *pGobjectClass = G_OBJECT_CLASS (pKlass);
  GstVideoDecoderClass *pVideoDecoderClass = GST_VIDEO_DECODER_CLASS (pKlass);

  nxvideodec_base_init (pKlass);

  pGobjectClass->set_property = gst_nxvideodec_set_property;
  pGobjectClass->get_property = gst_nxvideodec_get_property;

  pVideoDecoderClass->start = GST_DEBUG_FUNCPTR (gst_nxvideodec_start);
  pVideoDecoderClass->stop = GST_DEBUG_FUNCPTR (gst_nxvideodec_stop);

  pVideoDecoderClass->set_format =
      GST_DEBUG_FUNCPTR (gst_nxvideodec_set_format);
  pVideoDecoderClass->flush = GST_DEBUG_FUNCPTR (gst_nxvideodec_flush);
  pVideoDecoderClass->handle_frame =
      GST_DEBUG_FUNCPTR (gst_nxvideodec_handle_frame);

#if SUPPORT_NO_MEMORY_COPY
#else
  g_object_class_install_property (pGobjectClass,
      PROP_TYPE,
      g_param_spec_int ("buffer-type", "buffer-type",
          "Buffer Type(0:NORMAL 1:MM_VIDEO_BUFFER_TYPE_GEM)", 0, 1,
          BUFFER_TYPE_GEM, G_PARAM_READWRITE));
#endif

  FUNC_OUT ();
}

static void
gst_nxvideodec_init (GstNxVideoDec * pNxVideoDec)
{
  FUNC_IN ();

  GST_DEBUG_OBJECT (pNxVideoDec, "dec_init");

  // Initialize variables
  pNxVideoDec->pNxVideoDecHandle = NULL;
  pNxVideoDec->pInputState = NULL;
  pNxVideoDec->isState = STOP;
#if SUPPORT_NO_MEMORY_COPY
#else
  pNxVideoDec->bufferType = BUFFER_TYPE_GEM;
#endif
#ifdef TIZEN_FEATURE_ARTIK530
  pNxVideoDec->allocator = gst_tizen_allocator_new ();
#endif
  pthread_mutex_init (&pNxVideoDec->mutex, NULL);

  FUNC_OUT ();
}

void
gst_nxvideodec_set_property (GObject * pObject, guint propertyId,
    const GValue * pValue, GParamSpec * pPspec)
{
  GstNxVideoDec *pNxvideodec = GST_NXVIDEODEC (pObject);
  FUNC_IN ();

  GST_DEBUG_OBJECT (pNxvideodec, "set_property");

  switch (propertyId) {
#if SUPPORT_NO_MEMORY_COPY
#else
    case PROP_TYPE:
      pNxvideodec->bufferType = g_value_get_int (pValue);
      break;
#endif
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (pObject, propertyId, pPspec);
      break;
  }

  FUNC_OUT ();
}

void
gst_nxvideodec_get_property (GObject * pObject, guint propertyId,
    GValue * pValue, GParamSpec * pPspec)
{
  GstNxVideoDec *pNxvideodec = GST_NXVIDEODEC (pObject);
  FUNC_IN ();

  GST_DEBUG_OBJECT (pNxvideodec, "get_property");

  switch (propertyId) {
#if SUPPORT_NO_MEMORY_COPY
#else
    case PROP_TYPE:
      g_value_set_int (pValue, pNxvideodec->bufferType);
      break;
#endif
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (pObject, propertyId, pPspec);
      break;
  }

  FUNC_OUT ();
}

static gboolean
gst_nxvideodec_start (GstVideoDecoder * pDecoder)
{
  GstNxVideoDec *pNxVideoDec = GST_NXVIDEODEC (pDecoder);
  FUNC_IN ();

  GST_DEBUG_OBJECT (pNxVideoDec, "start");

  if (pNxVideoDec->pNxVideoDecHandle) {
    CloseVideoDec (pNxVideoDec->pNxVideoDecHandle);
    pNxVideoDec->pNxVideoDecHandle = NULL;
  }

  pNxVideoDec->pNxVideoDecHandle = OpenVideoDec ();

  if (pNxVideoDec->pNxVideoDecHandle == NULL) {
    GST_ERROR ("VideoDecHandle is NULL !\n");
    return FALSE;
  }

  pthread_mutex_lock (&pNxVideoDec->mutex);
  pNxVideoDec->isState = PLAY;
  pthread_mutex_unlock (&pNxVideoDec->mutex);

  FUNC_OUT ();

  return TRUE;
}

static gboolean
gst_nxvideodec_stop (GstVideoDecoder * pDecoder)
{
  GstNxVideoDec *pNxVideoDec = GST_NXVIDEODEC (pDecoder);
  FUNC_IN ();
  if (pNxVideoDec == NULL) {
    GST_ERROR ("pDecoder is NULL !\n");
    return FALSE;
  }

  GST_DEBUG_OBJECT (pNxVideoDec, "stop");

  pthread_mutex_lock (&pNxVideoDec->mutex);
  pNxVideoDec->isState = STOP;
  pthread_mutex_unlock (&pNxVideoDec->mutex);

  if (pNxVideoDec->pNxVideoDecHandle->pSem) {
    VDecSemSignal (pNxVideoDec->pNxVideoDecHandle->pSem);
    VDecSemDestroy (pNxVideoDec->pNxVideoDecHandle->pSem);
    pNxVideoDec->pNxVideoDecHandle->pSem = NULL;
  }

  CloseVideoDec (pNxVideoDec->pNxVideoDecHandle);

#ifdef TIZEN_FEATURE_ARTIK530
  if (pNxVideoDec->allocator) {
    gst_object_unref (pNxVideoDec->allocator);
    pNxVideoDec->allocator = NULL;
  }
#endif
  pthread_mutex_destroy (&pNxVideoDec->mutex);

  FUNC_OUT ();
  return TRUE;
}

static gboolean
gst_nxvideodec_set_format (GstVideoDecoder * pDecoder,
    GstVideoCodecState * pState)
{
  GstNxVideoDec *pNxVideoDec = GST_NXVIDEODEC (pDecoder);
  GstStructure *pStructure = NULL;
  const gchar *pMimeType = NULL;
  GstBuffer *pCodecData = NULL;
  GstVideoCodecState *pOutputState = NULL;
  NX_VIDEO_DEC_STRUCT *pDecHandle = NULL;
  gint ret = FALSE;
#ifdef TIZEN_FEATURE_ARTIK530
  gboolean bIsFormatChange = FALSE;
#endif

  FUNC_IN ();

  GST_DEBUG_OBJECT (pNxVideoDec, "set_format");

  if (pNxVideoDec->pInputState) {
    gst_video_codec_state_unref (pNxVideoDec->pInputState);
    pNxVideoDec->pInputState = NULL;
  }

  pNxVideoDec->pInputState = gst_video_codec_state_ref (pState);

  // Check Support Codec Type
  pStructure = gst_caps_get_structure (pNxVideoDec->pInputState->caps, 0);
  pMimeType = gst_structure_get_name (pStructure);
  if (pNxVideoDec->pNxVideoDecHandle) {
    pDecHandle = pNxVideoDec->pNxVideoDecHandle;
  } else {
    return FALSE;
  }

#ifdef TIZEN_FEATURE_ARTIK530
  GST_DEBUG_OBJECT (pNxVideoDec, "old %dx%d fps=%d/%d %p",
      pDecHandle->width, pDecHandle->height,
      pDecHandle->fpsNum, pDecHandle->fpsDen,
      pDecHandle->codec_data);
  GST_DEBUG_OBJECT (pNxVideoDec, "new %dx%d fps=%d/%d %p",
      GST_VIDEO_INFO_WIDTH (&pState->info),
      GST_VIDEO_INFO_HEIGHT (&pState->info),
      GST_VIDEO_INFO_FPS_N (&pState->info),
      GST_VIDEO_INFO_FPS_D (&pState->info),
      pState->codec_data);

  bIsFormatChange |=
      pDecHandle->width != GST_VIDEO_INFO_WIDTH (&pState->info);
  bIsFormatChange |=
      pDecHandle->height != GST_VIDEO_INFO_HEIGHT (&pState->info);
  if (GST_VIDEO_INFO_FPS_N (&pState->info) != 0) {
    bIsFormatChange |=
        pDecHandle->fpsNum != GST_VIDEO_INFO_FPS_N (&pState->info);
    bIsFormatChange |=
        pDecHandle->fpsDen != GST_VIDEO_INFO_FPS_D (&pState->info);
  } else {
    bIsFormatChange |= pDecHandle->fpsNum != 30;
    bIsFormatChange |= pDecHandle->fpsDen != 1;
  }
  bIsFormatChange |= (pDecHandle->codec_data != pState->codec_data);
  gst_buffer_replace (&pDecHandle->codec_data, pState->codec_data);
  GST_DEBUG_OBJECT (pNxVideoDec, "format changed=%d", bIsFormatChange);
#endif

  pDecHandle->codecType = FindCodecInfo (pState, pDecHandle);

  if (pDecHandle->codecType < 0) {
    GST_ERROR ("Unsupported VideoDecoder Mime Type : %s\n", pMimeType);
    return FALSE;
  }

  if (pDecHandle->pExtraData) {
    g_free (pDecHandle->pExtraData);
    pDecHandle->pExtraData = NULL;
    pDecHandle->extraDataSize = 0;
  }

  pCodecData = pNxVideoDec->pInputState->codec_data;

  if (pCodecData) {
    GstMapInfo mapInfo;

    if (!gst_buffer_map (pCodecData, &mapInfo, GST_MAP_READ)) {
      GST_ERROR ("Cannot map input buffer!\n");
      return FALSE;
    }

    if (mapInfo.size > 0 && mapInfo.data) {
      pDecHandle->pExtraData = (guint8 *) g_malloc (mapInfo.size);
      pDecHandle->extraDataSize = mapInfo.size;
    }

    if (FALSE == GetExtraInfo (pDecHandle, (guint8 *) mapInfo.data,
            mapInfo.size)) {
      gst_buffer_unmap (pCodecData, &mapInfo);
      return FALSE;
    }
    gst_buffer_unmap (pCodecData, &mapInfo);
  } else {
    GST_LOG ("No Codec Data\n");
  }

  if (pDecHandle->codecType == V4L2_PIX_FMT_H264) {
    const gchar *pStr = NULL;

    if ((pStr = gst_structure_get_string (pStructure, "alignment"))) {
      if (strcmp (pStr, "au") == 0) {
        pDecHandle->h264Alignment = H264_PARSE_ALIGN_AU;
        GST_DEBUG_OBJECT (pNxVideoDec, ">>>>> H264 alignment: au Type.");
      } else if (strcmp (pStr, "nal") == 0) {
        pDecHandle->h264Alignment = H264_PARSE_ALIGN_NAL;
        GST_DEBUG_OBJECT (pNxVideoDec, ">>>>> H264 alignment: nal Type.");
      } else {
        GST_DEBUG_OBJECT (pNxVideoDec, "unknown alignment: %s", pStr);
      }
    }
  }

  pOutputState =
      gst_video_decoder_set_output_state (pDecoder, GST_VIDEO_FORMAT_S420,
      pDecHandle->width, pDecHandle->height, pNxVideoDec->pInputState);

  pOutputState->caps = gst_caps_new_simple ("video/x-raw",
      "format", G_TYPE_STRING,
      gst_video_format_to_string (GST_VIDEO_FORMAT_S420), "width", G_TYPE_INT,
      pDecHandle->width, "height", G_TYPE_INT, pDecHandle->height, "framerate",
      GST_TYPE_FRACTION, pDecHandle->fpsNum, pDecHandle->fpsDen, NULL);

  gst_video_codec_state_unref (pOutputState);

  pNxVideoDec->pNxVideoDecHandle->imgPlaneNum = 1;
#if SUPPORT_NO_MEMORY_COPY
  GST_DEBUG_OBJECT (pNxVideoDec, ">>>>> Accelerable.");
#else
  if (BUFFER_TYPE_GEM == pNxVideoDec->bufferType) {
    GST_DEBUG_OBJECT (pNxVideoDec, ">>>>> Accelerable.");
  }
#endif

  ret = gst_video_decoder_negotiate (pDecoder);

  if (FALSE == ret) {
    GST_ERROR ("Fail Negotiate !\n");
    return ret;
  }

#ifdef TIZEN_FEATURE_ARTIK530
  if (!bIsFormatChange) {
    FUNC_OUT ();
    return TRUE;
  }

  if (pDecHandle->hCodec) {
    NX_V4l2DecClose (pDecHandle->hCodec);
    pDecHandle->hCodec = NULL;

    if (pDecHandle->pTmpStrmBuf) {
      g_free (pDecHandle->pTmpStrmBuf);
      pDecHandle->pTmpStrmBuf = NULL;
    }

    pDecHandle->bInitialized = FALSE;
  }
#endif

  if (0 != InitVideoDec (pNxVideoDec->pNxVideoDecHandle)) {
    return FALSE;
  }

  FUNC_OUT ();

  return ret;
}

static gboolean
gst_nxvideodec_flush (GstVideoDecoder * pDecoder)
{
  GstNxVideoDec *pNxvideodec = GST_NXVIDEODEC (pDecoder);

  FUNC_IN ();

  GST_DEBUG_OBJECT (pNxvideodec, "flush");

  if (pNxvideodec->pNxVideoDecHandle) {
    pNxvideodec->pNxVideoDecHandle->bFlush = TRUE;
  }

  FUNC_OUT ();

  return TRUE;
}

#if SUPPORT_NO_MEMORY_COPY
static void
nxvideodec_get_offset_stride (gint width, gint height, guint8 * pSrc,
    gsize * pOffset, gint * pStride)
{
  guint8 *plu = NULL;
  guint8 *pcb = NULL;
  guint8 *pcr = NULL;
  gint luStride = 0;
  gint luVStride = 0;
  gint cStride = 0;
  gint cVStride = 0;

  luStride = ALIGN (width, 32);
  luVStride = ALIGN (height, 16);
  cStride = luStride / 2;
  cVStride = ALIGN (height / 2, 16);
  plu = pSrc;
  pcb = plu + luStride * luVStride;
  pcr = pcb + cStride * cVStride;

  pOffset[0] = 0;
  pOffset[1] = pcb - plu;
  pOffset[2] = pcr - plu;

  pStride[0] = luStride;
  pStride[1] = cStride;
  pStride[2] = cStride;
}

static GstFlowReturn
gst_nxvideodec_handle_frame (GstVideoDecoder * pDecoder,
    GstVideoCodecFrame * pFrame)
{
  GstNxVideoDec *pNxVideoDec = GST_NXVIDEODEC (pDecoder);
  NX_V4L2DEC_OUT decOut;
  gint64 timeStamp = 0;
  GstMapInfo mapInfo;
  gint ret = 0;
  gboolean bKeyFrame = FALSE;

  GstMemory *pTmem = NULL;
  GstBuffer *pGstbuf = NULL;
  struct video_meta_mmap_buffer *pMeta = NULL;

  NX_VID_MEMORY_INFO *pImg = NULL;
  GstVideoCodecState *pState = NULL;

  GstVideoMeta *pVmeta = NULL;

  guint n_planes = 0;
  gsize offset[3] = { 0, };
  gint stride[3] = { 0, };

  FUNC_IN ();

  if (!gst_buffer_map (pFrame->input_buffer, &mapInfo, GST_MAP_READ)) {
    GST_ERROR ("Cannot map input buffer!");
    gst_video_codec_frame_unref (pFrame);
    return GST_FLOW_ERROR;
  }

  bKeyFrame = GST_VIDEO_CODEC_FRAME_IS_SYNC_POINT (pFrame);

  ret =
      VideoDecodeFrame (pNxVideoDec->pNxVideoDecHandle, pFrame->input_buffer,
      &decOut, bKeyFrame);

  gst_buffer_unmap (pFrame->input_buffer, &mapInfo);
  if (DEC_ERR == ret) {
    GetTimeStamp (pNxVideoDec->pNxVideoDecHandle, &timeStamp);
    return gst_video_decoder_drop_frame (pDecoder, pFrame);
  } else if (DEC_INIT_ERR == ret) {
    return GST_FLOW_ERROR;
  }

  if (decOut.dispIdx < 0) {
    return GST_FLOW_OK;
  }

  GST_DEBUG_OBJECT (pNxVideoDec, " decOut.dispIdx: %d\n", decOut.dispIdx);

  pMeta =
      (struct video_meta_mmap_buffer *) g_malloc (sizeof (struct
          video_meta_mmap_buffer));

  if (!pMeta) {
    GST_ERROR_OBJECT (pNxVideoDec, "failed to malloc for meta");
    gst_video_codec_frame_unref (pFrame);
    return GST_FLOW_ERROR;
  }

  pImg = &decOut.hImg;
  pMeta->v4l2BufferIdx = decOut.dispIdx;
  pMeta->pNxVideoDec = pNxVideoDec;

  pGstbuf = gst_buffer_new ();
  if (!pGstbuf) {
    GST_ERROR_OBJECT (pNxVideoDec, "failed to gst_buffer_new");
    gst_video_codec_frame_unref (pFrame);
    goto HANDLE_ERROR;
  }

  n_planes = 3;
  nxvideodec_get_offset_stride (pNxVideoDec->pNxVideoDecHandle->width,
      pNxVideoDec->pNxVideoDecHandle->height, (guint8 *) pImg->pBuffer[0],
      offset, stride);

  pState = gst_video_decoder_get_output_state (pDecoder);

  pTmem = gst_tizen_allocator_alloc_surface (pNxVideoDec->allocator, &pState->info, pImg->surface, pMeta, nxvideodec_buffer_finalize);

  pVmeta =
      gst_buffer_add_video_meta_full (pGstbuf, GST_VIDEO_FRAME_FLAG_NONE,
      GST_VIDEO_INFO_FORMAT (&pState->info),
      GST_VIDEO_INFO_WIDTH (&pState->info),
      GST_VIDEO_INFO_HEIGHT (&pState->info), n_planes, offset, stride);

  if (!pVmeta) {
    GST_ERROR_OBJECT (pNxVideoDec, "failed to gst_buffer_add_video_meta_full");
    gst_video_codec_state_unref (pState);
    gst_video_codec_frame_unref (pFrame);
    goto HANDLE_ERROR;
  }
  gst_buffer_append_memory (pGstbuf, pTmem);

#ifdef CODEC_DEC_OUTPUT_DUMP /* for decoder output dump */
  decoder_output_dump (pGstbuf);
#endif
  pFrame->output_buffer = pGstbuf;

  if (-1 == GetTimeStamp (pNxVideoDec->pNxVideoDecHandle, &timeStamp)) {
    GST_DEBUG_OBJECT (pNxVideoDec, "Cannot Found Time Stamp!!!");
  }
  pFrame->pts = timeStamp;
  GST_BUFFER_PTS (pFrame->output_buffer) = timeStamp;

  gst_video_codec_state_unref (pState);

  ret = gst_video_decoder_finish_frame (pDecoder, pFrame);

  FUNC_OUT ();

  return ret;

HANDLE_ERROR:
  if (pGstbuf) {
    g_free (pGstbuf);
  }

  if (pMeta) {
    nxvideodec_buffer_finalize (pMeta);
  }

  return GST_FLOW_ERROR;
}
#else
static GstFlowReturn
gst_nxvideodec_handle_frame (GstVideoDecoder * pDecoder,
    GstVideoCodecFrame * pFrame)
{
  GstNxVideoDec *pNxVideoDec = GST_NXVIDEODEC (pDecoder);
  NX_V4L2DEC_OUT decOut;
  gint64 timeStamp = 0;
  GstMapInfo mapInfo;
  gint ret = 0;
  gboolean bKeyFrame = FALSE;
  GstMemory *pGstmem = NULL;
  GstBuffer *pGstbuf = NULL;
  struct video_meta_mmap_buffer *pMeta = NULL;

  FUNC_IN ();

  if (!gst_buffer_map (pFrame->input_buffer, &mapInfo, GST_MAP_READ)) {
    GST_ERROR ("Cannot map input buffer!");
    gst_video_codec_frame_unref (pFrame);
    return GST_FLOW_ERROR;
  }

  bKeyFrame = GST_VIDEO_CODEC_FRAME_IS_SYNC_POINT (pFrame);

  ret =
      VideoDecodeFrame (pNxVideoDec->pNxVideoDecHandle, pFrame->input_buffer,
      &decOut, bKeyFrame);

  gst_buffer_unmap (pFrame->input_buffer, &mapInfo);
  if (DEC_ERR == ret) {
    GetTimeStamp (pNxVideoDec->pNxVideoDecHandle, &timeStamp);
    return gst_video_decoder_drop_frame (pDecoder, pFrame);
  } else if (DEC_INIT_ERR == ret) {
    return GST_FLOW_ERROR;
  }

  if (decOut.dispIdx < 0) {
    return GST_FLOW_OK;
  }

  GST_DEBUG_OBJECT (pNxVideoDec, " decOut.dispIdx: %d\n", decOut.dispIdx);

  if (BUFFER_TYPE_GEM == pNxVideoDec->bufferType) {
    pGstbuf = gst_buffer_new ();
    if (!pGstbuf) {
      GST_ERROR_OBJECT (pNxVideoDec, "failed to gst_buffer_new");
      gst_video_codec_frame_unref (pFrame);
      goto HANDLE_ERROR;
    }

    pMeta =
        (struct video_meta_mmap_buffer *) g_malloc (sizeof (struct
            video_meta_mmap_buffer));
    if (!pMeta) {
      GST_ERROR_OBJECT (pNxVideoDec, "failed to malloc for meta");
      gst_video_codec_frame_unref (pFrame);
      return GST_FLOW_ERROR;
    }
    pMeta->v4l2BufferIdx = decOut.dispIdx;
    pMeta->pNxVideoDec = pNxVideoDec;
    pGstmem = gst_memory_new_wrapped (GST_MEMORY_FLAG_READONLY,
        pMeta,
        sizeof (struct video_meta_mmap_buffer),
        0,
        sizeof (struct video_meta_mmap_buffer),
        pMeta, nxvideodec_buffer_finalize);
    if (!pGstmem) {
      GST_ERROR_OBJECT (pNxVideoDec,
          "failed to gst_memory_new_wrapped for mmap buffer");
      gst_video_codec_frame_unref (pFrame);
      goto HANDLE_ERROR;
    }
    gst_buffer_append_memory (pGstbuf, pGstmem);

    gst_buffer_add_mmvideobuffer_meta (pGstbuf, 0);

    pFrame->output_buffer = pGstbuf;

    if (-1 == GetTimeStamp (pNxVideoDec->pNxVideoDecHandle, &timeStamp)) {
      GST_DEBUG_OBJECT (pNxVideoDec, "Cannot Found Time Stamp!!!");
    }
    pFrame->pts = timeStamp;
    GST_BUFFER_PTS (pFrame->output_buffer) = timeStamp;
  } else {
    GstVideoFrame videoFrame;
    NX_VID_MEMORY_INFO *pImg = NULL;
    guint8 *pPtr = NULL;;
    GstVideoCodecState *pState = NULL;
    GstFlowReturn flowRet;
    guint8 *plu = NULL;
    guint8 *pcb = NULL;
    guint8 *pcr = NULL;
    gint luStride = 0;
    gint luVStride = 0;
    gint cStride = 0;
    gint cVStride = 0;

    flowRet = gst_video_decoder_allocate_output_frame (pDecoder, pFrame);
    pState = gst_video_decoder_get_output_state (pDecoder);
    if (flowRet != GST_FLOW_OK) {
      gst_video_codec_state_unref (pState);
      gst_video_codec_frame_unref (pFrame);
      return flowRet;
    }

    if (!gst_video_frame_map (&videoFrame, &pState->info, pFrame->output_buffer,
            GST_MAP_WRITE)) {
      GST_ERROR ("Cannot video frame map!\n");
      gst_video_codec_state_unref (pState);
      gst_video_codec_frame_unref (pFrame);
      return GST_FLOW_ERROR;
    }

    if (-1 == GetTimeStamp (pNxVideoDec->pNxVideoDecHandle, &timeStamp)) {
      GST_DEBUG_OBJECT (pNxVideoDec, "Cannot Found Time Stamp!!!");
    }
    pFrame->pts = timeStamp;
    GST_BUFFER_PTS (pFrame->output_buffer) = timeStamp;

    pImg = &decOut.hImg;
    pPtr = GST_VIDEO_FRAME_COMP_DATA (&videoFrame, 0);

    luStride = ALIGN (pNxVideoDec->pNxVideoDecHandle->width, 32);
    luVStride = ALIGN (pNxVideoDec->pNxVideoDecHandle->height, 16);
    cStride = luStride / 2;
    cVStride = ALIGN (pNxVideoDec->pNxVideoDecHandle->height / 2, 16);
    plu = (guint8 *) pImg->pBuffer[0];
    pcb = plu + luStride * luVStride;
    pcr = pcb + cStride * cVStride;

    CopyImageToBufferYV12 ((guint8 *) plu, (guint8 *) pcb, (guint8 *) pcr,
        pPtr, luStride, cStride, pNxVideoDec->pNxVideoDecHandle->width,
        pNxVideoDec->pNxVideoDecHandle->height);

    DisplayDone (pNxVideoDec->pNxVideoDecHandle, decOut.dispIdx);

    gst_video_frame_unmap (&videoFrame);
    gst_video_codec_state_unref (pState);
  }

  ret = gst_video_decoder_finish_frame (pDecoder, pFrame);

  FUNC_OUT ();

  return ret;

HANDLE_ERROR:
  if (pGstbuf) {
    g_free (pGstbuf);
  }
  if (pMeta) {
    nxvideodec_buffer_finalize (pMeta);
  }

  return GST_FLOW_ERROR;
}
#endif

static void
nxvideodec_buffer_finalize (gpointer pData)
{
  gint ret = 0;

  FUNC_IN ();

  struct video_meta_mmap_buffer *pMeta =
      (struct video_meta_mmap_buffer *) pData;

  if (!pMeta) {
    GST_ERROR ("Error: pData is null !");
    return;
  }

  if ((pMeta->pNxVideoDec) && (pMeta->pNxVideoDec->pNxVideoDecHandle)) {
    pthread_mutex_lock (&pMeta->pNxVideoDec->mutex);
    if (PLAY == pMeta->pNxVideoDec->isState) {
      GST_DEBUG_OBJECT (pMeta->pNxVideoDec, "v4l2BufferIdx: %d\n",
          pMeta->v4l2BufferIdx);
      ret =
          DisplayDone (pMeta->pNxVideoDec->pNxVideoDecHandle,
          pMeta->v4l2BufferIdx);
      if (ret) {
        GST_ERROR ("Fail: DisplayDone !");
      }
    }
    pthread_mutex_unlock (&pMeta->pNxVideoDec->mutex);
  } else {
    GST_ERROR ("Error: hCodec is null !");
  }

  if (pMeta) {
    g_free (pMeta);
  }
}

static gboolean
plugin_init (GstPlugin * plugin)
{
  gboolean ret;

  FUNC_IN ();

  /* FIXME Remember to set the rank if it's an element that is meant
     to be autoplugged by decodebin. */
  ret = gst_element_register (plugin, "nxvideodec", GST_RANK_PRIMARY + 1,
      GST_TYPE_NXVIDEODEC);
  FUNC_OUT ();

  return ret;
}

#ifndef VERSION
#define VERSION "0.1.0"
#endif
#ifndef PACKAGE
#define PACKAGE "S5P6818 GStreamer PlugIn"
#endif
#ifndef PACKAGE_NAME
#define PACKAGE_NAME "S5P6818 GStreamer PlugIn"
#endif
#ifndef GST_PACKAGE_ORIGIN
#define GST_PACKAGE_ORIGIN "http://www.nexell.co.kr"
#endif

GST_PLUGIN_DEFINE (GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    nxvideodec,
    "Nexell H/W Video Decoder for S5P6818",
    plugin_init, VERSION, "LGPL", PACKAGE_NAME, GST_PACKAGE_ORIGIN)