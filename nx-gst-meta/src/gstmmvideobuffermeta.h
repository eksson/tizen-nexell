/* GStreamer
 * Copyright (C) 2016 Biela.Jo <doriya@nexell.co.kr>
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
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 */

#ifndef __GST_MMVIDEOBUFFERMETA_H__
#define __GST_MMVIDEOBUFFERMETA_H__

#include <gst/gst.h>

typedef struct _GstMMVideoBufferMeta GstMMVideoBufferMeta;

struct _GstMMVideoBufferMeta
{
  GstMeta meta;
  gint memory_index;
};

GType gst_mmvideobuffer_meta_api_get_type (void);
#define GST_MMVIDEOBUFFER_META_API_TYPE (gst_mmvideobuffer_meta_api_get_type())

#define gst_buffer_get_mmvideobuffer_meta(b) \
	((GstMMVideoBufferMeta*)gst_buffer_get_meta( (b),GST_MMVIDEOBUFFER_META_API_TYPE) )

/* implementation */
const GstMetaInfo *gst_mmvideobuffer_meta_get_info (void);
#define GST_MMVIDEOBUFFER_META_INFO (gst_mmvideobuffer_meta_get_info())

GstMMVideoBufferMeta *gst_buffer_add_mmvideobuffer_meta (GstBuffer * buffer,
    gint memory_index);

#endif // __GST_MMVIDEOBUFFERMETA_H__
