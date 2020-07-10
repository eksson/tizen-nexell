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

#ifndef __GST_NXVIDEODEC_H__
#define __GST_NXVIDEODEC_H__

#include <gst/gst.h>
#include <gst/video/video.h>
#include <gst/video/gstvideodecoder.h>

/* Debug Flags */
#define	DBG_FUNCTION		0

#if	DBG_FUNCTION
#define	FUNC_IN()			GST_LOG("%s() In", __func__)
#define	FUNC_OUT()		GST_LOG("%s() Out", __func__)
#else
#define	FUNC_IN()			do{}while(0)
#define	FUNC_OUT()		do{}while(0)
#endif //      DBG_FUNCTION

G_BEGIN_DECLS

#define GST_TYPE_NXVIDEODEC (gst_nxvideodec_get_type())
#define GST_NXVIDEODEC(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_NXVIDEODEC, GstNxVideoDec))
#define GST_NXVIDEODEC_CLASS(klass) (G_TYPE_CHECK_CLASS_CAST ((klass), GST_TYPE_NXVIDEODEC, GstNxVideoDecClass))
#define GST_IS_NXVIDEODEC(obj) (G_TYPE_CHECK_INSTANCE_TYPE((obj), GST_TYPE_NXVIDEODEC))
#define GST_IS_NXVIDEODEC_CLASS(obj) (G_TYPE_CHECK_CLASS_TYPE((klass), GST_TYPE_NXVIDEODEC))

#define USE_NATIVE_DRM_BUFFER

typedef struct _GstNxVideoDec GstNxVideoDec;
typedef struct _GstNxVideoDecClass GstNxVideoDecClass;
typedef struct _GstNxDecOutBuffer GstNxDecOutBuffer;

#include <mm_types.h>
#include "decoder.h"

struct _GstNxDecOutBuffer
{
  GstBuffer *pGstBuffer;
  gint v4l2BufferIdx;
  GstNxVideoDec *pNxVideoDec;
};

struct _GstNxVideoDec
{
  GstVideoDecoder base_nxvideodec;
  NX_VIDEO_DEC_STRUCT *pNxVideoDecHandle;
  gint bufferType;
  gint bDisableVideoOutReorder;
  // video state
  GstVideoCodecState *pInputState;
  gint isState;
  pthread_mutex_t mutex;

	gint negoWidth;
	gint negoHeight;
	gint bIsCodecData;
	gint bIsInitVideoDec;
	gint bIsNegotiate;

#ifdef TIZEN_FEATURE_ARTIK530
  GstAllocator *allocator;
#endif
};

struct _GstNxVideoDecClass
{
  GstVideoDecoderClass base_nxvideodec_class;
  GstPadTemplate *pSinktempl;
};

GType gst_nxvideodec_get_type (void);

G_END_DECLS
#endif // __GST_NXVIDEODEC_H__
