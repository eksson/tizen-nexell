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

#include "gstmmvideobuffermeta.h"

GType
gst_mmvideobuffer_meta_api_get_type (void)
{
  static volatile GType type;
  static const gchar *tags[] = { "MMVideoBuffer", NULL };

  if (g_once_init_enter (&type)) {
    GType _type = gst_meta_api_type_register ("GstMMVideoBufferMetaAPI", tags);
    g_once_init_leave (&type, _type);
  }

  return type;
}

static gboolean
gst_mmvideobuffer_meta_init (GstMeta * meta, gpointer params,
    GstBuffer * buffer)
{
  GstMMVideoBufferMeta *emeta = (GstMMVideoBufferMeta *) meta;

  emeta->memory_index = -1;

  return TRUE;
}

static gboolean
gst_mmvideobuffer_meta_transform (GstBuffer * transbuf, GstMeta * meta,
    GstBuffer * buffer, GQuark type, gpointer data)
{
  GstMMVideoBufferMeta *emeta = (GstMMVideoBufferMeta *) meta;

  /* we always copy no matter what transform */
  gst_buffer_add_mmvideobuffer_meta (transbuf, emeta->memory_index);

  return TRUE;
}

static void
gst_mmvideobuffer_meta_free (GstMeta * meta, GstBuffer * buffer)
{
  GstMMVideoBufferMeta *emeta = (GstMMVideoBufferMeta *) meta;

  emeta->memory_index = -1;
}

const GstMetaInfo *
gst_mmvideobuffer_meta_get_info (void)
{
  static const GstMetaInfo *meta_info = NULL;

  if (g_once_init_enter (&meta_info)) {
    const GstMetaInfo *mi = gst_meta_register (GST_MMVIDEOBUFFER_META_API_TYPE,
        "GstMMVideoBufferMeta",
        sizeof (GstMMVideoBufferMeta),
        gst_mmvideobuffer_meta_init,
        gst_mmvideobuffer_meta_free,
        gst_mmvideobuffer_meta_transform);

    g_once_init_leave (&meta_info, mi);
  }
  return meta_info;
}

GstMMVideoBufferMeta *
gst_buffer_add_mmvideobuffer_meta (GstBuffer * buffer, gint memory_index)
{
  GstMMVideoBufferMeta *meta;

  g_return_val_if_fail (GST_IS_BUFFER (buffer), NULL);

  meta =
      (GstMMVideoBufferMeta *) gst_buffer_add_meta (buffer,
      GST_MMVIDEOBUFFER_META_INFO, NULL);
  meta->memory_index = memory_index;

  return meta;
}
