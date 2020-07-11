/*
 *	Copyright (C) 2016 Nexell Co. All Rights Reserved
 *	Nexell Co. Proprietary & Confidential
 *
 *	NEXELL INFORMS THAT THIS CODE AND INFORMATION IS PROVIDED "AS IS" BASE
 *  AND	WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING
 *  BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS
 *  FOR A PARTICULAR PURPOSE.
 *
 *	File		: nx_video_api.c
 *	Brief		: V4L2 Video Decoder
 *	Author		: SungWon Jo (doriya@nexell.co.kr)
 *	History		: 2016.04.25 : Create
 */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>

#include <sys/stat.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>
#include <linux/videodev2_nxp_media.h>

#include <nx_video_alloc.h>
#include <nx_video_api.h>
#include "nx_video_log.h"


/*----------------------------------------------------------------------------*/
#define NX_V4L2_DEC_NAME		"nx-vpu-dec"
#define VIDEODEV_MINOR_MAX		63
#define STREAM_BUFFER_NUM		  1


struct NX_V4L2DEC_INFO
{
  int fd;
  uint32_t codecType;
  int32_t width;
  int32_t height;

  int32_t useExternalFrameBuffer;
  int32_t numFrameBuffers;
  NX_VID_MEMORY_HANDLE hImage[MAX_FRAME_BUFFER_NUM];

  NX_MEMORY_HANDLE hStream[STREAM_BUFFER_NUM];

  /* Initialize Output Information        */
  uint8_t pSeqData[1024];       /* SPS PPS (H.264) or Decoder Specific Information(for MPEG4) */
  int32_t seqDataSize;

  IMG_DISP_INFO dispInfo;

  int32_t planesNum;

  int32_t frameCnt;
 
	int32_t iRemainStrmSize;

	int vopTimeBits;		/* For MPEG4 */
	int32_t iInterlace;		/* For VC1 */

#ifdef TIZEN_FEATURE_ARTIK530
  /* buffer manager */
  void *bufmgr;
#endif
};


/*
 *		Find Device Node
 */

/*----------------------------------------------------------------------------*/
static int32_t
V4l2DecOpen (void)
{
  int fd = -1;

  bool found = false;
  struct stat s;
  FILE *stream_fd;
  char filename[64];
  char name[64];
  int32_t i = 0;

  while (!found && (i <= VIDEODEV_MINOR_MAX)) {
    /* video device node */
    snprintf (filename, 64, "/dev/video%d", i);

    /* if the node is video device */
    if ((lstat (filename, &s) == 0) && S_ISCHR (s.st_mode)
        && ((int) ((unsigned short) (s.st_rdev) >> 8) == 81)) {
      /* open sysfs entry */
      snprintf (filename, 64, "/sys/class/video4linux/video%d/name", i);
      stream_fd = fopen (filename, "r");
      if (stream_fd == NULL) {
        _D ("failed to open sysfs entry for videodev \n");
        i++;
        continue;
      }

      /* read sysfs entry for device name */
      if (fgets (name, sizeof (name), stream_fd) == 0) {
        _D ("failed to read sysfs entry for videodev\n");
      } else {
        if (strncmp (name, NX_V4L2_DEC_NAME, strlen (NX_V4L2_DEC_NAME)) == 0) {
          _D ("node found for device %s: /dev/video%d \n", NX_V4L2_DEC_NAME,
              i);
          found = true;
        }
      }

      fclose (stream_fd);
    }

    i++;
  }

  if (found) {
    snprintf (filename, 64, "/dev/video%d", i - 1);
    fd = open (filename, O_RDWR);
  }

  return fd;
}

#ifndef MKTAG
#define MKTAG(a,b,c,d) (a | (b << 8) | (c << 16) | (d << 24))
#endif

#ifndef PUT_LE32
#define PUT_LE32(_p, _var) \
	*_p++ = (uint8_t)((_var) >> 0); \
	*_p++ = (uint8_t)((_var) >> 8); \
	*_p++ = (uint8_t)((_var) >> 16); \
	*_p++ = (uint8_t)((_var) >> 24);
#endif

#ifndef PUT_BE32
#define PUT_BE32(_p, _var) \
	*_p++ = (uint8_t)((_var) >> 24); \
	*_p++ = (uint8_t)((_var) >> 16); \
	*_p++ = (uint8_t)((_var) >> 8); \
	*_p++ = (uint8_t)((_var) >> 0);
#endif

#ifndef PUT_LE16
#define PUT_LE16(_p, _var) \
	*_p++ = (uint8_t)((_var) >> 0); \
	*_p++ = (uint8_t)((_var) >> 8);
#endif

#ifndef PUT_BE16
#define PUT_BE16(_p, _var) \
	*_p++ = (uint8_t)((_var) >> 8); \
	*_p++ = (uint8_t)((_var) >> 0);
#endif

typedef struct
{
  uint32_t dwUsedBits;
  uint8_t *pbyStart;
  uint32_t dwPktSize;
} VLD_STREAM;

static int32_t
vld_count_leading_zero (uint32_t dwWord)
{
  int32_t iLZ = 0;

  if ((dwWord >> (32 - 16)) == 0)
    iLZ = 16;
  if ((dwWord >> (32 - 8 - iLZ)) == 0)
    iLZ += 8;
  if ((dwWord >> (32 - 4 - iLZ)) == 0)
    iLZ += 4;
  if ((dwWord >> (32 - 2 - iLZ)) == 0)
    iLZ += 2;
  if ((dwWord >> (32 - 1 - iLZ)) == 0)
    iLZ += 1;

  return iLZ;
}

static uint32_t
vld_show_bits (VLD_STREAM * pstVldStm, int32_t iBits)
{
  uint32_t dwUsedBits = pstVldStm->dwUsedBits;
  int32_t iBitCnt = 8 - (dwUsedBits & 0x7);
  uint8_t *pbyRead = (uint8_t *) pstVldStm->pbyStart + (dwUsedBits >> 3);
  uint32_t dwRead;

  dwRead = *pbyRead++ << 24;
  if (iBits > iBitCnt) {
    dwRead += *pbyRead++ << 16;
    if (iBits > iBitCnt + 8) {
      dwRead += *pbyRead++ << 8;
      if (iBits > iBitCnt + 16)
        dwRead += *pbyRead++;
    }
  }

  return (dwRead << (8 - iBitCnt)) >> (32 - iBits);
}

static uint32_t
vld_get_bits (VLD_STREAM * pstVldStm, int32_t iBits)
{
  uint32_t dwUsedBits = pstVldStm->dwUsedBits;
  int32_t iBitCnt = 8 - (dwUsedBits & 0x7);
  uint8_t *pbyRead = (uint8_t *) pstVldStm->pbyStart + (dwUsedBits >> 3);
  uint32_t dwRead;

  pstVldStm->dwUsedBits += iBits;

  dwRead = *pbyRead++ << 24;
  if (iBits > iBitCnt) {
    dwRead += *pbyRead++ << 16;
    if (iBits > iBitCnt + 8) {
      dwRead += *pbyRead++ << 8;
      if (iBits > iBitCnt + 16)
        dwRead += *pbyRead++;
    }
  }

  return (dwRead << (8 - iBitCnt)) >> (32 - iBits);
}

static void
vld_flush_bits (VLD_STREAM * pstVldStm, int iBits)
{
  pstVldStm->dwUsedBits += iBits;
}

static uint32_t
vld_get_uev (VLD_STREAM * pstVldStm)
{
  int32_t iLZ = vld_count_leading_zero (vld_show_bits (pstVldStm, 32));

  vld_flush_bits (pstVldStm, iLZ);
  return (vld_get_bits (pstVldStm, iLZ + 1) - 1);
}

static void
Mp4DecParseVideoCfg (NX_V4L2DEC_HANDLE hDec, uint8_t * pbyStream,
    int32_t iStreamSize)
{
  uint8_t *pbyStrm = pbyStream;
  uint32_t uPreFourByte = (uint32_t) - 1;

  hDec->vopTimeBits = 0;

  do {
    if (pbyStrm >= (pbyStream + iStreamSize))
      break;

    uPreFourByte = (uPreFourByte << 8) + *pbyStrm++;

    if (uPreFourByte >= 0x00000120 && uPreFourByte <= 0x0000012F) {
      VLD_STREAM stStrm = { 0, pbyStrm, iStreamSize };
      int32_t i;

      vld_flush_bits (&stStrm, 1 + 8);  /* random_accessible_vol, video_object_type_indication */
      if (vld_get_bits (&stStrm, 1))    /* is_object_layer_identifier */
        vld_flush_bits (&stStrm, 4 + 3);        /* video_object_layer_verid, video_object_layer_priority */

      if (vld_get_bits (&stStrm, 4) == 0xF)     /* aspect_ratio_info */
        vld_flush_bits (&stStrm, 8 + 8);        /* par_width, par_height */

      if (vld_get_bits (&stStrm, 1)) {  /* vol_control_parameters */
        if (vld_get_bits (&stStrm, 2 + 1 + 1) & 1) {    /* chroma_format, low_delay, vbv_parameters */
          vld_flush_bits (&stStrm, 15 + 1);     /* first_half_bit_rate, marker_bit */
          vld_flush_bits (&stStrm, 15 + 1);     /* latter_half_bit_rate, marker_bit */
          vld_flush_bits (&stStrm, 15 + 1);     /* first_half_vbv_buffer_size, marker_bit */
          vld_flush_bits (&stStrm, 3 + 11 + 1); /* latter_half_vbv_buffer_size, first_half_vbv_occupancy, marker_bit */
          vld_flush_bits (&stStrm, 15 + 1);     /* latter_half_vbv_occupancy, marker_bit */
        }
      }

      vld_flush_bits (&stStrm, 2 + 1);  /* video_object_layer_shape, marker_bit */

      for (i = 0; i < 16; i++)  /* vop_time_increment_resolution */
        if (vld_get_bits (&stStrm, 1))
          break;
      hDec->vopTimeBits = 16 - i;
      break;
    }
  } while (1);
}

static int32_t
Mp4DecParseFrameHeader (NX_V4L2DEC_HANDLE hDec, uint8_t * pbyStream,
    int32_t iStreamSize)
{
  VLD_STREAM stStrm = { 0, pbyStream, iStreamSize };
  int32_t iSize = iStreamSize;

  if (vld_get_bits (&stStrm, 32) == 0x000001B6) {
    vld_flush_bits (&stStrm, 2);        /* vop_coding_type */

    do {
      if (vld_get_bits (&stStrm, 1) == 0)
        break;
    } while (stStrm.dwUsedBits < ((uint32_t) iStreamSize << 3));

    vld_flush_bits (&stStrm, 1 + hDec->vopTimeBits + 1);        /* marker_bits, vop_time_increment, marker_bits */

    if (vld_get_bits (&stStrm, 1) == 0) /* vop_coded */
      iSize = 0;
  }

  return iSize;
}

static int32_t
GetSequenceHeader (NX_V4L2DEC_HANDLE hDec, NX_V4L2DEC_SEQ_IN * pSeqIn)
{
  uint8_t *pbySrc = pSeqIn->seqBuf;
  uint8_t *pbyDst = (uint8_t *) hDec->hStream[0]->pBuffer;
  int32_t iSize = pSeqIn->seqSize;

  switch (hDec->codecType) {
    case V4L2_PIX_FMT_H264:
      if (pSeqIn->seqSize > 0) {
        memcpy (pbyDst, pbySrc, pSeqIn->seqSize);

        if ((pbySrc[2] == 0) && (pbySrc[7] > 51))
          pbyDst[7] = 51;
        else if ((pbySrc[2] == 1) && (pbySrc[6] > 51))
          pbyDst[6] = 51;
        break;
      } else {
        return -1;
      }

    case V4L2_PIX_FMT_DIV3:
    case V4L2_PIX_FMT_WMV9:
    case V4L2_PIX_FMT_RV8:
    case V4L2_PIX_FMT_RV9:
    case V4L2_PIX_FMT_VP8:
      memcpy(pbyDst, pbySrc, pSeqIn->seqSize);
      break;
    case V4L2_PIX_FMT_XVID:
    case V4L2_PIX_FMT_DIVX:
    case V4L2_PIX_FMT_DIV4:
    case V4L2_PIX_FMT_DIV5:
    case V4L2_PIX_FMT_DIV6:
    case V4L2_PIX_FMT_MPEG4:
    	if (V4L2_PIX_FMT_DIV5 == hDec->codecType) {
			  if (pSeqIn->seqSize >= 4) {
				  // sequence start code
				  if( (pbySrc[0] != 0x00) || (pbySrc[1] != 0x00) ||
					    (pbySrc[2] != 0x01) || (pbySrc[3] != 0xb0) ) {
					  return -1;
				  }
			  }
		  }
      Mp4DecParseVideoCfg (hDec, pbySrc, pSeqIn->seqSize);

    default:
      if (pSeqIn->seqSize > 0)
        memcpy (pbyDst, pbySrc, pSeqIn->seqSize);
      else
        return -1;
  }

  return iSize;
}

static int32_t
GetFrameStream (NX_V4L2DEC_HANDLE hDec, NX_V4L2DEC_IN * pDecIn, int32_t * idx)
{
  int32_t iSize = pDecIn->strmSize;
  uint8_t *pbySrc = pDecIn->strmBuf;
  uint8_t *pbyDst;

  if (iSize <= 0)
    return 0;

  *idx = hDec->frameCnt % STREAM_BUFFER_NUM;
  pbyDst = (uint8_t *) hDec->hStream[*idx]->pBuffer;

  switch (hDec->codecType) {
    case V4L2_PIX_FMT_H264:
      memcpy (pbyDst, pbySrc, iSize);
      if (iSize > 8) {
        if ((pbySrc[2] == 0) && ((pbySrc[4] & 0x1F) == 7) && (pbySrc[7] > 51))
          pbyDst[7] = 51;
        else if ((pbySrc[2] == 1) && ((pbySrc[3] & 0x1F) == 7)
            && (pbySrc[6] > 51))
          pbyDst[6] = 51;
      }
      break;

    case V4L2_PIX_FMT_WVC1:
      /* check start code as prefix (0x00, 0x00, 0x01) */
      if (pbySrc[0] != 0 || pbySrc[1] != 0 || pbySrc[2] != 1) {
        *pbyDst++ = 0x00;
        *pbyDst++ = 0x00;
        *pbyDst++ = 0x01;
        *pbyDst++ = 0x0D;
        memcpy (pbyDst, pbySrc, iSize);
        iSize += 4;
      } else {
        /* no extra header size, there is start code in input stream */
        memcpy (pbyDst, pbySrc, iSize);
      }
      break;

    case V4L2_PIX_FMT_WMV9:
    case V4L2_PIX_FMT_RV8:
    case V4L2_PIX_FMT_RV9:
    case V4L2_PIX_FMT_DIV3:
    case V4L2_PIX_FMT_VP8:
      memcpy(pbyDst, pbySrc, iSize);
      break;
    case V4L2_PIX_FMT_XVID:
    case V4L2_PIX_FMT_DIVX:
    case V4L2_PIX_FMT_DIV4:
    case V4L2_PIX_FMT_DIV5:
    case V4L2_PIX_FMT_DIV6:
    case V4L2_PIX_FMT_MPEG4:
      /* For PB Frame */
      if (hDec->vopTimeBits > 0) {
        iSize = Mp4DecParseFrameHeader (hDec, pbySrc, iSize);
      }

    default:
      memcpy ((void *) pbyDst, (void *) pbySrc, iSize);
  }

  return iSize;
}


/*
 *		V4L2 Decoder
 */

/*----------------------------------------------------------------------------*/
NX_V4L2DEC_HANDLE
NX_V4l2DecOpen (uint32_t codecType)
{
  NX_V4L2DEC_HANDLE hDec =
      (NX_V4L2DEC_HANDLE) malloc (sizeof (struct NX_V4L2DEC_INFO));

  memset (hDec, 0, sizeof (struct NX_V4L2DEC_INFO));

  hDec->fd = V4l2DecOpen ();
  if (hDec->fd <= 0) {
    _E ("Failed to open video decoder device\n");
    goto ERROR_EXIT;
  }

  /* Query capabilities of Device */
  {
    struct v4l2_capability cap;

    memset (&cap, 0, sizeof (cap));

    if (ioctl (hDec->fd, VIDIOC_QUERYCAP, &cap) != 0) {
      _E ("failed to ioctl: VIDIOC_QUERYCAP\n");
      goto ERROR_EXIT;
    }
  }

  hDec->codecType = codecType;

  return hDec;

ERROR_EXIT:
  free (hDec);

  return NULL;
}

/*----------------------------------------------------------------------------*/
int32_t
NX_V4l2DecClose (NX_V4L2DEC_HANDLE hDec)
{
  int32_t ret = 0;
  int32_t i;

  if (NULL == hDec) {
    _E ("Fail, Invalid Handle.\n");
    return -1;
  }

  {
    enum v4l2_buf_type type;

    type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    if (ioctl (hDec->fd, VIDIOC_STREAMOFF, &type) != 0) {
      _E ("failed to ioctl: VIDIOC_STREAMOFF(Stream)\n");
      return -1;
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl (hDec->fd, VIDIOC_STREAMOFF, &type) != 0) {
      _E ("failed to ioctl: VIDIOC_STREAMOFF(Image)\n");
      return -1;
    }

    for (i = 0; i < STREAM_BUFFER_NUM; i++)
      NX_FreeMemory (hDec->hStream[i]);

    close (hDec->fd);
  }

  if (hDec->useExternalFrameBuffer == 0) {
    for (i = 0; i < hDec->numFrameBuffers; i++) {
      if (hDec->hImage[i]) {
        NX_FreeVideoMemory (hDec->hImage[i]);
        hDec->hImage[i] = NULL;
      }
    }
  }

#ifdef TIZEN_FEATURE_ARTIK530
    if (hDec->bufmgr)
      tbm_bufmgr_deinit (hDec->bufmgr);
#endif

  free (hDec);

  return ret;
}

/*----------------------------------------------------------------------------*/
int32_t
NX_V4l2DecParseVideoCfg (NX_V4L2DEC_HANDLE hDec, NX_V4L2DEC_SEQ_IN * pSeqIn,
    NX_V4L2DEC_SEQ_OUT * pSeqOut)
{
  int32_t imgWidth = pSeqIn->width;
  int32_t imgHeight = pSeqIn->height;

  memset (pSeqOut, 0, sizeof (NX_V4L2DEC_SEQ_OUT));

  if (NULL == hDec) {
    _E ("Fail, Invalid Handle.\n");
    return -1;
  }

  hDec->seqDataSize = (pSeqIn->seqSize < 1024) ?
                      pSeqIn->seqSize : sizeof(hDec->pSeqData);
  memcpy (hDec->pSeqData, pSeqIn->seqBuf, hDec->seqDataSize);

  /* Set Stream Formet */
  {
    struct v4l2_format fmt;

    memset (&fmt, 0, sizeof (fmt));

    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    fmt.fmt.pix_mp.pixelformat = hDec->codecType;

    if ((imgWidth == 0) || (imgHeight == 0))
      fmt.fmt.pix_mp.plane_fmt[0].sizeimage =
          MAX_IMAGE_WIDTH * MAX_IMAGE_HEIGHT * 3 / 4;
    else
      fmt.fmt.pix_mp.plane_fmt[0].sizeimage = imgWidth * imgHeight * 3 / 4;

    fmt.fmt.pix_mp.width = imgWidth;
    fmt.fmt.pix_mp.height = imgHeight;
    fmt.fmt.pix_mp.num_planes = 1;

    if (ioctl (hDec->fd, VIDIOC_S_FMT, &fmt) != 0) {
      _E ("Failed to ioctx : VIDIOC_S_FMT(Input Stream)\n");
      return -1;
    }
  }

  /* Malloc Stream Buffer */
  {
    struct v4l2_requestbuffers req;
    int32_t buffCnt = STREAM_BUFFER_NUM;
    int32_t i;

    /* IOCTL : VIDIOC_REQBUFS For Input Stream */
    memset (&req, 0, sizeof (req));
    req.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    req.count = buffCnt;
    req.memory = V4L2_MEMORY_DMABUF;

    if (ioctl (hDec->fd, VIDIOC_REQBUFS, &req) != 0) {
      _E ("failed to ioctl: VIDIOC_REQBUFS(Input Stream)\n");
      return -1;
    }

    for (i = 0; i < buffCnt; i++) {
      hDec->hStream[i] =
          NX_AllocateMemory (MAX_IMAGE_WIDTH * MAX_IMAGE_HEIGHT * 3 / 4, 4096);
      if (hDec->hStream[i] == NULL) {
        _E ("Failed to allocate stream buffer(%d, %d)\n", i,
            MAX_IMAGE_WIDTH * MAX_IMAGE_HEIGHT * 3 / 4);
        return -1;
      }

      if (NX_MapMemory (hDec->hStream[i]) != 0) {
        _E ("Stream memory Mapping Failed\n");
        return -1;
      }
    }
  }

  /* Set Parameter */
  {
    if (hDec->codecType == V4L2_PIX_FMT_MJPEG) {
      struct v4l2_control ctrl;

      ctrl.id = V4L2_CID_MPEG_VIDEO_THUMBNAIL_MODE;
      ctrl.value = pSeqIn->thumbnailMode;

      if (ioctl (hDec->fd, VIDIOC_S_CTRL, &ctrl) != 0) {
        _E ("failed to ioctl: Set Thumbnail Mode\n");
        return -1;
      }
    }

    if (pSeqIn->disableVideoOutReorder) {
			struct v4l2_control ctrl;

			ctrl.id = V4L2_CID_DISABLE_VIDEO_OUT_REORDER;
			ctrl.value = pSeqIn->disableVideoOutReorder;

			if (ioctl(hDec->fd, VIDIOC_S_CTRL, &ctrl) != 0) {
				_E ("failed to ioctl: Set DisableVideoOutReorder\n");
				return -1;
			}
		}
  }

  /* Parser Sequence Header */
  {
    struct v4l2_plane planes[1];
    struct v4l2_buffer buf;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    int32_t iSeqSize = GetSequenceHeader (hDec, pSeqIn);

    if (iSeqSize <= 0) {
      _E ("Fail, input data has error!!");
      return -1;
    }

    memset (&buf, 0, sizeof (buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    buf.m.planes = planes;
    buf.length = 1;
    buf.memory = V4L2_MEMORY_DMABUF;
    buf.index = 0;

    buf.m.planes[0].m.userptr = (unsigned long) hDec->hStream[0]->pBuffer;
    buf.m.planes[0].m.fd = hDec->hStream[0]->dmaFd;
    buf.m.planes[0].length = hDec->hStream[0]->size;
    buf.m.planes[0].bytesused = iSeqSize;
    buf.m.planes[0].data_offset = 0;

    buf.timestamp.tv_sec = pSeqIn->timeStamp / 1000;
    buf.timestamp.tv_usec = (pSeqIn->timeStamp % 1000) * 1000;

    if (ioctl (hDec->fd, VIDIOC_QBUF, &buf) != 0) {
      _E ("failed to ioctl: VIDIOC_QBUF(Header Stream)\n");
      return -1;
    }

    if (ioctl (hDec->fd, VIDIOC_STREAMON, &type) != 0) {
      _E ("Fail, ioctl(): VIDIOC_STREAMON. (Input)\n");
      return -1;
    }

    memset (&buf, 0, sizeof (buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    buf.m.planes = planes;
    buf.length = 1;
    buf.memory = V4L2_MEMORY_DMABUF;

    if (ioctl (hDec->fd, VIDIOC_DQBUF, &buf) != 0) {
      _E ("failed to ioctl: VIDIOC_DQBUF(Header Stream)\n");
      return -1;
    }

    pSeqOut->usedByte = buf.bytesused;

    if (buf.field == V4L2_FIELD_NONE)
      pSeqOut->interlace = NONE_FIELD;
    else if (V4L2_FIELD_INTERLACED)
      pSeqOut->interlace = FIELD_INTERLACED;

    hDec->iInterlace = pSeqOut->interlace;
    hDec->iRemainStrmSize -= buf.bytesused;
  }

  /* Get Image Information */
  {
    struct v4l2_format fmt;
    struct v4l2_crop crop;

    memset (&fmt, 0, sizeof (fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

    if (ioctl (hDec->fd, VIDIOC_G_FMT, &fmt) != 0) {
      _E ("Fail, ioctl(): VIDIOC_G_FMT.\n");
      return -1;
    }

#ifdef TIZEN_FEATURE_ARTIK530
    pSeqOut->imgFourCC = fmt.fmt.pix_mp.pixelformat;
#endif

    pSeqOut->width = fmt.fmt.pix_mp.width;
    pSeqOut->height = fmt.fmt.pix_mp.height;
    pSeqOut->minBuffers = fmt.fmt.pix_mp.reserved[1];
    hDec->numFrameBuffers = pSeqOut->minBuffers;

    memset (&crop, 0, sizeof (crop));
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

    if (ioctl (hDec->fd, VIDIOC_G_CROP, &crop) != 0) {
      _E ("Fail, ioctl(): VIDIOC_G_CROP\n");
      return -1;
    }

    pSeqOut->dispInfo.dispLeft = crop.c.left;
    pSeqOut->dispInfo.dispTop = crop.c.top;
    pSeqOut->dispInfo.dispRight = crop.c.left + crop.c.width;
    pSeqOut->dispInfo.dispBottom = crop.c.top + crop.c.height;
  }

  return 0;
}

/*----------------------------------------------------------------------------*/
int32_t
NX_V4l2DecInit (NX_V4L2DEC_HANDLE hDec, NX_V4L2DEC_SEQ_IN * pSeqIn)
{
	struct v4l2_format fmt;
	struct v4l2_requestbuffers req;
	struct v4l2_plane planes[3];
	struct v4l2_buffer buf;
	enum v4l2_buf_type type;
	int32_t imgBuffCnt;
  int32_t i;
  int32_t j;

  /* Set Output Image */
  {
    memset (&fmt, 0, sizeof (fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.pixelformat = pSeqIn->imgFormat;
    fmt.fmt.pix_mp.width = pSeqIn->width;
    fmt.fmt.pix_mp.height = pSeqIn->height;
    fmt.fmt.pix_mp.num_planes = pSeqIn->imgPlaneNum;

#ifdef TIZEN_FEATURE_ARTIK530
#else
    // This code is make a error in Tizen,
    // but, it's on nexell branch at github of NEXELLCORP
  	for (i=0 ; i<(int32_t)pSeqIn->imgPlaneNum ; i++) {
	  	fmt.fmt.pix_mp.plane_fmt[i].sizeimage =
          hDec->hImage[0]->size[i];
  		fmt.fmt.pix_mp.plane_fmt[i].bytesperline =
          hDec->hImage[0]->stride[i];
  	}
#endif

    if (ioctl (hDec->fd, VIDIOC_S_FMT, &fmt) != 0) {
      _E ("failed to ioctl: VIDIOC_S_FMT(Output Yuv)\n");
      return -1;
    }

    hDec->planesNum = pSeqIn->imgPlaneNum;
  }

  /* Malloc Output Image */
  {
    /* Calculate Buffer Number */
    if (pSeqIn->pMemHandle == NULL) {
      hDec->useExternalFrameBuffer = false;
      imgBuffCnt = pSeqIn->numBuffers;
    } else {
      hDec->useExternalFrameBuffer = true;
      if (2 > pSeqIn->numBuffers - hDec->numFrameBuffers) {
        _D ("External Buffer too small.(min=%d, buffers=%d)\n",
            hDec->numFrameBuffers, pSeqIn->numBuffers);
        return -1;
      }

      imgBuffCnt = pSeqIn->numBuffers;
    }
    hDec->numFrameBuffers = imgBuffCnt;

    /* Request Output Buffer */
    memset (&req, 0, sizeof (req));
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.count = imgBuffCnt;
    req.memory = V4L2_MEMORY_DMABUF;

    if (ioctl (hDec->fd, VIDIOC_REQBUFS, &req) != 0) {
      _E ("failed to ioctl: VIDIOC_REQBUFS(Output YUV)\n");
      return -1;
    }

    memset (&buf, 0, sizeof (buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.m.planes = planes;
    buf.length = pSeqIn->imgPlaneNum;
    buf.memory = V4L2_MEMORY_DMABUF;

#ifdef TIZEN_FEATURE_ARTIK530
  hDec->bufmgr = tbm_bufmgr_init (-1);
  _D ("bufmgr = %p", hDec->bufmgr);
  if (!hDec->bufmgr) {
    _E ("failed to initialize tbm_init");
    return -1;
  }
#endif

    /* Allocate Buffer */
    for (i = 0; i < imgBuffCnt; i++) {
      if (true == hDec->useExternalFrameBuffer) {
        hDec->hImage[i] = pSeqIn->pMemHandle[i];
      } else {
        hDec->hImage[i] =
            NX_AllocateVideoMemory (hDec->bufmgr, pSeqIn->width,
              pSeqIn->height, pSeqIn->imgPlaneNum, pSeqIn->imgFormat,
              4096);
        if (hDec->hImage[i] == NULL) {
          _E ("Failed to allocate image buffer(%d, %d, %d)\n", i,
              pSeqIn->width, pSeqIn->height);
          return -1;
        }

        if (NX_MapVideoMemory (hDec->hImage[i]) != 0) {
          _E ("Video Memory Mapping Failed\n");
          return -1;
        }
      }
    }

    /* Allocate Buffer(Internal or External) */
    for (i = 0; i < imgBuffCnt; i++) {
      buf.index = i;

      for (j = 0; j < (int32_t) pSeqIn->imgPlaneNum; j++) {
        buf.m.planes[j].m.fd = hDec->hImage[i]->dmaFd[j];
        buf.m.planes[j].length = hDec->hImage[i]->size[j];
      }

      if (ioctl (hDec->fd, VIDIOC_QBUF, &buf) != 0) {
        _E ("failed to ioctl: VIDIOC_QBUF(Output YUV - %d)\n", i);
        return -1;
      }
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl (hDec->fd, VIDIOC_STREAMON, &type) != 0) {
      _E ("failed to ioctl: VIDIOC_STREAMON\n");
      return -1;
    }
  }

  return 0;
}

/*----------------------------------------------------------------------------*/
int32_t
NX_V4l2DecDecodeFrame (NX_V4L2DEC_HANDLE hDec, NX_V4L2DEC_IN * pDecIn,
    NX_V4L2DEC_OUT * pDecOut)
{
  struct v4l2_buffer buf;
  struct v4l2_plane planes[3];
  int idx;
  int32_t iStrmSize;
  int32_t frameType;

  uint32_t iDecodedUsed = 0;
  uint32_t iDisplayUsed = 0;

  if (NULL == hDec) {
    _E ("Fail, Invalid Handle.\n");
    return -1;
  }

#ifdef TIZEN_FEATURE_ARTIK530
#else
  // This code is make a error in Tizen,
  // but, it's on nexell branch at github of NEXELLCORP
	memset (pDecOut, 0x00, sizeof(NX_V4L2DEC_OUT));
#endif
	pDecOut->decIdx = -1;
	pDecOut->dispIdx = -1;

  iStrmSize = GetFrameStream (hDec, pDecIn, &idx);

	hDec->iRemainStrmSize += iStrmSize;

  /* Queue Input Buffer */
  memset (&buf, 0, sizeof (buf));
  buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  buf.m.planes = planes;
  buf.length = 1;
  buf.memory = V4L2_MEMORY_DMABUF;
  buf.index = idx;
  buf.timestamp.tv_sec = pDecIn->timeStamp / 1000;
  buf.timestamp.tv_usec = (pDecIn->timeStamp % 1000) * 1000;
  buf.flags = pDecIn->eos ? 1 : 0;

  /* buf.m.planes[0].m.userptr = (unsigned long)hStream->pBuffer; */
  buf.m.planes[0].m.fd = hDec->hStream[idx]->dmaFd;
  buf.m.planes[0].length = hDec->hStream[idx]->size;
  buf.m.planes[0].bytesused = iStrmSize;
  buf.m.planes[0].data_offset = 0;

  if (ioctl (hDec->fd, VIDIOC_QBUF, &buf) != 0) {
    _E ("Fail, ioctl(): VIDIOC_QBUF.(Input)\n");
    return -1;
  }

  if (iStrmSize > 0) {
    /* Dequeue Input ES Buffer -> Get Decoded Order Result */
    memset (&buf, 0, sizeof (buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    buf.m.planes = planes;
    buf.length = 1;
    buf.memory = V4L2_MEMORY_DMABUF;

    if (ioctl (hDec->fd, VIDIOC_DQBUF, &buf) != 0) {
      _E ("Fail, ioctl(): VIDIOC_DQBUF.(Input)\n");
      return -1;
    }

    pDecOut->decIdx = buf.index;
    pDecOut->usedByte = buf.bytesused;
    pDecOut->outFrmReliable_0_100[DECODED_FRAME] = buf.reserved;
    pDecOut->timeStamp[DECODED_FRAME] =
        ((uint64_t) buf.timestamp.tv_sec) * 1000 + buf.timestamp.tv_usec / 1000;
    frameType = buf.flags;

    if (frameType & V4L2_BUF_FLAG_KEYFRAME)
      pDecOut->picType[DECODED_FRAME] = PIC_TYPE_I;
    else if (frameType & V4L2_BUF_FLAG_PFRAME)
      pDecOut->picType[DECODED_FRAME] = PIC_TYPE_P;
    else if (frameType & V4L2_BUF_FLAG_BFRAME)
      pDecOut->picType[DECODED_FRAME] = PIC_TYPE_B;
    else
      pDecOut->picType[DECODED_FRAME] = PIC_TYPE_UNKNOWN;

    if (buf.field == V4L2_FIELD_NONE)
      pDecOut->interlace[DECODED_FRAME] = NONE_FIELD;
    else if (buf.field == V4L2_FIELD_SEQ_TB)
      pDecOut->interlace[DECODED_FRAME] = TOP_FIELD_FIRST;
    else if (buf.field == V4L2_FIELD_SEQ_BT)
      pDecOut->interlace[DECODED_FRAME] = BOTTOM_FIELD_FIRST;
  } else if (pDecIn->strmSize > 0) {
    pDecOut->usedByte = pDecIn->strmSize;
  }

  /* Dequeue Output YUV Buffer -> Get Display Order Result */
  memset (&buf, 0, sizeof (buf));
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  buf.m.planes = planes;
  buf.length = hDec->planesNum;
  buf.memory = V4L2_MEMORY_DMABUF;

  if (ioctl (hDec->fd, VIDIOC_DQBUF, &buf) != 0) {
    _E ("Fail, ioctl(): VIDIOC_DQBUF(Output)\n");
    return -100;
  }

  pDecOut->dispIdx = buf.index;
  // pDecOut->dispInfo = &hDec->dispInfo;         // TBD.

  if (pDecOut->dispIdx >= 0) {
    pDecOut->hImg = *hDec->hImage[buf.index];
    pDecOut->timeStamp[DISPLAY_FRAME] =
        ((uint64_t) buf.timestamp.tv_sec) * 1000 + buf.timestamp.tv_usec / 1000;
    iDisplayUsed = buf.bytesused;
    frameType = buf.flags;

    if (frameType & V4L2_BUF_FLAG_KEYFRAME)
      pDecOut->picType[DISPLAY_FRAME] = PIC_TYPE_I;
    else if (frameType & V4L2_BUF_FLAG_PFRAME)
      pDecOut->picType[DISPLAY_FRAME] = PIC_TYPE_P;
    else if (frameType & V4L2_BUF_FLAG_BFRAME)
      pDecOut->picType[DISPLAY_FRAME] = PIC_TYPE_B;
    else
      pDecOut->picType[DISPLAY_FRAME] = PIC_TYPE_UNKNOWN;

    if (buf.field == V4L2_FIELD_NONE)
      pDecOut->interlace[DISPLAY_FRAME] = NONE_FIELD;
    else if (buf.field == V4L2_FIELD_SEQ_TB)
      pDecOut->interlace[DISPLAY_FRAME] = TOP_FIELD_FIRST;
    else if (buf.field == V4L2_FIELD_SEQ_BT)
      pDecOut->interlace[DISPLAY_FRAME] = BOTTOM_FIELD_FIRST;
  }

	pDecOut->usedByte = iDecodedUsed ? iDecodedUsed : iDisplayUsed;

	hDec->iRemainStrmSize -= pDecOut->usedByte;
	pDecOut->remainByte = hDec->iRemainStrmSize;

  hDec->frameCnt++;

  if (pDecOut->dispIdx == -1)
    return -1;

  return 0;
}

/*----------------------------------------------------------------------------*/
int32_t
NX_V4l2DecClrDspFlag (NX_V4L2DEC_HANDLE hDec, NX_VID_MEMORY_HANDLE hFrameBuf,
    int32_t iFrameIdx)
{
  struct v4l2_buffer buf;
  struct v4l2_plane planes[3];
  int32_t index = -1;
  int32_t i;

  if (NULL == hDec) {
    _E ("Fail, Invalid Handle.\n");
    return -1;
  }

  if (iFrameIdx >= 0) {
    index = iFrameIdx;
  } else {
    /* Search Buffer Index */
    if (hFrameBuf != NULL) {
      for (i = 0; i < hDec->numFrameBuffers; i++) {
        if (hFrameBuf == hDec->hImage[i]) {
          index = i;
          break;
        }
      }
    }
  }

  if (index < 0) {
    _E ("Fail, Invalid FrameBuffer or FrameIndex.\n");
    return -1;
  }

  memset (&buf, 0, sizeof (buf));
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  buf.index = index;
  buf.m.planes = planes;
  buf.length = hDec->planesNum;
  buf.memory = V4L2_MEMORY_DMABUF;

  for (i = 0; i < hDec->planesNum; i++) {
    buf.m.planes[i].m.fd = hDec->hImage[index]->dmaFd[i];
    buf.m.planes[i].length = hDec->hImage[index]->size[i];
  }

  /* Queue Output Buffer */
  if (ioctl (hDec->fd, VIDIOC_QBUF, &buf) != 0) {
    _E ("Fail, ioctl(): VIDIOC_QBUF.(Clear Display Index, index = %d)\n",
        index);
    return -1;
  }

  return 0;
}

/*----------------------------------------------------------------------------*/
int32_t
NX_V4l2DecFlush (NX_V4L2DEC_HANDLE hDec)
{
  enum v4l2_buf_type type;

  if (NULL == hDec) {
    _E ("Fail, Invalid Handle.\n");
    return -1;
  }

  type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  if (ioctl (hDec->fd, VIDIOC_STREAMOFF, &type) != 0) {
    _E ("failed to ioctl: VIDIOC_STREAMOFF(Stream)\n");
    return -1;
  }

  type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  if (ioctl (hDec->fd, VIDIOC_STREAMOFF, &type) != 0) {
    _E ("failed to ioctl: VIDIOC_STREAMOFF(Image)\n");
    return -1;
  }

  type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  if (ioctl (hDec->fd, VIDIOC_STREAMON, &type) != 0) {
    _E ("Fail, ioctl(): VIDIOC_STREAMON. (Input)\n");
    return -1;
  }

  type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  if (ioctl (hDec->fd, VIDIOC_STREAMON, &type) != 0) {
    _E ("failed to ioctl: VIDIOC_STREAMON\n");
    return -1;
  }

  {
    struct v4l2_plane planes[3];
    struct v4l2_buffer buf;
    int32_t i;
    int32_t j;

    memset (&buf, 0, sizeof (buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.m.planes = planes;
    buf.length = hDec->planesNum;
    buf.memory = V4L2_MEMORY_DMABUF;

    for (i = 0; i < hDec->numFrameBuffers; i++) {
      buf.index = i;

      for (j = 0; j < (int32_t) hDec->planesNum; j++) {
        buf.m.planes[j].m.fd = hDec->hImage[i]->dmaFd[j];
        buf.m.planes[j].length = hDec->hImage[i]->size[j];
      }

      if (ioctl (hDec->fd, VIDIOC_QBUF, &buf) != 0) {
        _E ("failed to ioctl: VIDIOC_QBUF(Output YUV - %d)\n", i);
        return -1;
      }
    }
  }

  return 0;
}

/* Optional Function */
int32_t
NX_DecGetFrameType (NX_V4L2DEC_HANDLE hDec, NX_V4L2DEC_IN * pDecIn,
    uint32_t codecType, int32_t * piFrameType)
{
  uint8_t *pbyStrm = pDecIn->strmBuf;
  uint32_t uPreFourByte = (uint32_t) - 1;
  int32_t iFrmType = PIC_TYPE_UNKNOWN;

  if ((pbyStrm == NULL) || (piFrameType == NULL))
    return -1;

  if (!codecType)
    codecType = hDec->codecType;

  switch (codecType) {
    case V4L2_PIX_FMT_H264:
      do {
        if (pbyStrm >= (pDecIn->strmBuf + pDecIn->strmSize))
          return -1;

        uPreFourByte = (uPreFourByte << 8) + *pbyStrm++;

        if ((uPreFourByte == 0x00000001) || (uPreFourByte << 8 == 0x00000100)) {
          int32_t iNaluType = pbyStrm[0] & 0x1F;

          /* Slice start code */
          if (iNaluType == 5) {
            iFrmType = PIC_TYPE_IDR;
            break;
          } else if (iNaluType == 1) {
            VLD_STREAM stStrm = { 8, pbyStrm, pDecIn->strmSize };

            vld_get_uev (&stStrm);      /* First_mb_in_slice */
            iFrmType = vld_get_uev (&stStrm);   /* Slice type */

            if (iFrmType == 0 || iFrmType == 5)
              iFrmType = PIC_TYPE_P;
            else if (iFrmType == 1 || iFrmType == 6)
              iFrmType = PIC_TYPE_B;
            else if (iFrmType == 2 || iFrmType == 7)
              iFrmType = PIC_TYPE_I;
            break;
          }
        }
      } while (1);
      break;

    case V4L2_PIX_FMT_MPEG2:
      do {
        if (pbyStrm >= (pDecIn->strmBuf + pDecIn->strmSize))
          return -1;

        uPreFourByte = (uPreFourByte << 8) + *pbyStrm++;

        /* Picture start code */
        if (uPreFourByte == 0x00000100) {
          VLD_STREAM stStrm = { 0, pbyStrm, pDecIn->strmSize };

          vld_flush_bits (&stStrm, 10); /* temporal_reference */
          iFrmType = vld_get_bits (&stStrm, 3); /* picture_coding_type */

          if (iFrmType == 1)
            iFrmType = PIC_TYPE_I;
          else if (iFrmType == 2)
            iFrmType = PIC_TYPE_P;
          else if (iFrmType == 3)
            iFrmType = PIC_TYPE_B;
          break;
        }
      } while (1);
      break;

    case V4L2_PIX_FMT_WVC1:
      if (hDec == NULL || hDec->seqDataSize == 0)
        return -1;

      {
        VLD_STREAM stStrm = { 0, pbyStrm, pDecIn->strmSize };

        if (hDec->iInterlace != NONE_FIELD) {
          /* FCM */
          if (vld_get_bits (&stStrm, 1) == 1)
            vld_flush_bits (&stStrm, 1);
        }

        iFrmType = vld_get_bits (&stStrm, 1);
        if (iFrmType == 0) {
          iFrmType = PIC_TYPE_P;
        } else {
          iFrmType = vld_get_bits (&stStrm, 1);
          if (iFrmType == 0) {
            iFrmType = PIC_TYPE_B;
          } else {
            iFrmType = vld_get_bits (&stStrm, 1);
            if (iFrmType == 0) {
              iFrmType = PIC_TYPE_I;
            } else {
              iFrmType = vld_get_bits (&stStrm, 1);
              if (iFrmType == 0)
                iFrmType = PIC_TYPE_VC1_BI;
              else
                iFrmType = PIC_TYPE_SKIP;
            }
          }
        }
      }
      break;

    case V4L2_PIX_FMT_WMV9:
      if (hDec == NULL || hDec->seqDataSize == 0)
        return -1;

      {
        int32_t rangeRed;
        int32_t fInterPFlag;
        int32_t maxBframes;
        VLD_STREAM stStrm = { 24, hDec->pSeqData, hDec->seqDataSize };

        /* Parse Sequece Header */
        rangeRed = vld_get_bits (&stStrm, 1);
        maxBframes = vld_get_bits (&stStrm, 3);
        vld_flush_bits (&stStrm, 2);
        fInterPFlag = vld_get_bits (&stStrm, 1);

        /* Parse Frame Header */
        stStrm.dwUsedBits = 0;
        stStrm.pbyStart = pbyStrm;
        stStrm.dwPktSize = pDecIn->strmSize;

        if (fInterPFlag == 1)
          vld_flush_bits (&stStrm, 1);  /* INTERPFRM */

        vld_flush_bits (&stStrm, 2);    /* FRMCNT */

        if (rangeRed == 1)
          vld_flush_bits (&stStrm, 1);  /* RANGEREDFRM */

        iFrmType = vld_get_bits (&stStrm, 1);
        if (maxBframes > 0) {
          if (iFrmType == 1) {
            iFrmType = PIC_TYPE_P;
          } else {
            iFrmType = vld_get_bits (&stStrm, 1);
            if (iFrmType == 1)
              iFrmType = PIC_TYPE_I;
            else if (iFrmType == 0)
              iFrmType = PIC_TYPE_B;    /* or BI */
          }
        } else {
          if (iFrmType == 0)
            iFrmType = PIC_TYPE_I;
          else if (iFrmType == 1)
            iFrmType = PIC_TYPE_P;
        }

      }
      break;

    default:
      return -1;
  }

  *piFrameType = iFrmType;

  return 0;
}
