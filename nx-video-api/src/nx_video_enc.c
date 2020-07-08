/*
 *	Copyright (C) 2016 Nexell Co. All Rights Reserved
 *	Nexell Co. Proprietary & Confidential
 *
 *	NEXELL INFORMS THAT THIS CODE AND INFORMATION IS PROVIDED "AS IS" BASE
 *  AND	WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING
 *  BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS
 *  FOR A PARTICULAR PURPOSE.
 *
 *	File		: nx_video_enc.c
 *	Brief		: V4L2 Video Encoder
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
#define NX_V4L2_ENC_NAME		"nx-vpu-enc"
#define VIDEODEV_MINOR_MAX		63
#define MAX_CTRL_NUM 			32
#define STREAM_BUFFER_NUM		1


struct NX_V4L2ENC_INFO
{
  int fd;
  uint32_t codecType;
  int32_t planesNum;
  int32_t outBuffCnt;
  int32_t frameCnt;
  uint8_t *pSeqBuf;
  int32_t seqSize;
  NX_MEMORY_HANDLE hBitStream[STREAM_BUFFER_NUM];
  NX_VID_MEMORY_HANDLE hImg[MAX_FRAME_BUFFER_NUM];
};


/*
 *		Find Device Node
 */

/*----------------------------------------------------------------------------*/
static int
V4l2EncOpen (void)
{
  int fd = -1;

  bool found = false;
  struct stat s;
  FILE *stream_fd;
  char filename[64], name[64];
  int i = 0;

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
        if (strncmp (name, NX_V4L2_ENC_NAME, strlen (NX_V4L2_ENC_NAME)) == 0) {
          _D ("node found for device %s: /dev/video%d\n", NX_V4L2_ENC_NAME,
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

/*
 *	V4L2 Encoder
 */

/*----------------------------------------------------------------------------*/
NX_V4L2ENC_HANDLE
NX_V4l2EncOpen (uint32_t codecType)
{
  NX_V4L2ENC_HANDLE hEnc =
      (NX_V4L2ENC_HANDLE) malloc (sizeof (struct NX_V4L2ENC_INFO));

  memset (hEnc, 0, sizeof (struct NX_V4L2ENC_INFO));

  hEnc->fd = V4l2EncOpen ();
  if (hEnc->fd <= 0) {
    _E ("failed to open video encoder device\n");
    goto ERROR_EXIT;
  }

  /* Query capabilities of Device */
  {
    struct v4l2_capability cap;

    memset (&cap, 0, sizeof (cap));

    if (ioctl (hEnc->fd, VIDIOC_QUERYCAP, &cap) != 0) {
      _E ("failed to ioctl: VIDIOC_QUERYCAP\n");
      goto ERROR_EXIT;
    }
  }

  hEnc->codecType = codecType;

  return hEnc;

ERROR_EXIT:
  free (hEnc);

  return NULL;
}

/*----------------------------------------------------------------------------*/
int32_t
NX_V4l2EncClose (NX_V4L2ENC_HANDLE hEnc)
{
  enum v4l2_buf_type type;
  int32_t ret = 0;
  int i;

  if (NULL == hEnc) {
    _E ("Fail, Invalid Handle.\n");
    return -1;
  }

  type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  if (ioctl (hEnc->fd, VIDIOC_STREAMOFF, &type) != 0) {
    _E ("Fail, ioctl(): VIDIOC_STREAMOFF - Stream\n");
    ret = -1;
  }

  type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  if (ioctl (hEnc->fd, VIDIOC_STREAMOFF, &type) != 0) {
    _E ("Fail, ioctl(): VIDIOC_STREAMOFF - Image\n");
    ret = -1;
  }

  for (i = 0; i < hEnc->outBuffCnt; i++)
    NX_FreeMemory (hEnc->hBitStream[i]);

  if (hEnc->pSeqBuf)
    free (hEnc->pSeqBuf);

  close (hEnc->fd);

  free (hEnc);

  return ret;
}

/*----------------------------------------------------------------------------*/
int32_t
NX_V4l2EncInit (NX_V4L2ENC_HANDLE hEnc, NX_V4L2ENC_PARA * pEncPara)
{
  int inWidth = pEncPara->width;
  int inHeight = pEncPara->height;
  int i;

  if (NULL == hEnc) {
    _E ("Fail, Invalid Handle.\n");
    return -1;
  }

  /* Set Stream Format */
  {
    struct v4l2_format fmt;

    memset (&fmt, 0, sizeof (fmt));

    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    fmt.fmt.pix_mp.pixelformat = hEnc->codecType;
    fmt.fmt.pix_mp.plane_fmt[0].sizeimage = inWidth * inHeight * 3 / 4;

    if (ioctl (hEnc->fd, VIDIOC_S_FMT, &fmt) != 0) {
      _E ("failed to ioctl: VIDIOC_S_FMT(Stream)\n");
      return -1;
    }
  }

  /* Set Image Format */
  {
    struct v4l2_format fmt;

    memset (&fmt, 0, sizeof (fmt));

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.pixelformat = pEncPara->imgFormat;
    fmt.fmt.pix_mp.width = inWidth;
    fmt.fmt.pix_mp.height = inHeight;
    fmt.fmt.pix_mp.num_planes = pEncPara->imgPlaneNum;

    if (ioctl (hEnc->fd, VIDIOC_S_FMT, &fmt) != 0) {
      _E ("Failed to s_fmt : YUV \n");
      return -1;
    }

    hEnc->planesNum = pEncPara->imgPlaneNum;
  }

  /* Set Encoder Parameter */
  {
    if (hEnc->codecType != V4L2_PIX_FMT_MJPEG) {
      struct v4l2_ext_control ext_ctrl[MAX_CTRL_NUM];
      struct v4l2_ext_controls ext_ctrls;

      ext_ctrl[0].id = V4L2_CID_MPEG_VIDEO_FPS_NUM;
      ext_ctrl[0].value = pEncPara->fpsNum;
      ext_ctrl[1].id = V4L2_CID_MPEG_VIDEO_FPS_DEN;
      ext_ctrl[1].value = pEncPara->fpsDen;
      ext_ctrl[2].id = V4L2_CID_MPEG_VIDEO_GOP_SIZE;
      ext_ctrl[2].value = pEncPara->keyFrmInterval;

      ext_ctrl[3].id = V4L2_CID_MPEG_VIDEO_CYCLIC_INTRA_REFRESH_MB;
      ext_ctrl[3].value = pEncPara->numIntraRefreshMbs;
      ext_ctrl[4].id = V4L2_CID_MPEG_VIDEO_SEARCH_RANGE;
      ext_ctrl[4].value = pEncPara->searchRange;

      ext_ctrl[5].id = V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE;
      ext_ctrl[5].value = (pEncPara->bitrate) ? (1) : (0);
      ext_ctrl[6].id = V4L2_CID_MPEG_VIDEO_BITRATE;
      ext_ctrl[6].value = pEncPara->bitrate;
      ext_ctrl[7].id = V4L2_CID_MPEG_VIDEO_VBV_SIZE;
      ext_ctrl[7].value = pEncPara->rcVbvSize;
      ext_ctrl[8].id = V4L2_CID_MPEG_VIDEO_RC_DELAY;
      ext_ctrl[8].value = pEncPara->RCDelay;
      ext_ctrl[9].id = V4L2_CID_MPEG_VIDEO_RC_GAMMA_FACTOR;
      ext_ctrl[9].value = pEncPara->gammaFactor;
      ext_ctrl[10].id = V4L2_CID_MPEG_VIDEO_FRAME_SKIP_MODE;
      ext_ctrl[10].value = pEncPara->disableSkip;

      if (hEnc->codecType == V4L2_PIX_FMT_H264) {
        ext_ctrl[11].id = V4L2_CID_MPEG_VIDEO_H264_AUD_INSERT;
        ext_ctrl[11].value = pEncPara->enableAUDelimiter;
        ext_ctrl[12].id = V4L2_CID_MPEG_VIDEO_H264_MAX_QP;
        ext_ctrl[12].value = pEncPara->maximumQp;

        ext_ctrls.count = 13;

        if ((pEncPara->bitrate == 0) || (pEncPara->initialQp > 0)) {
          ext_ctrl[13].id = V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP;
          ext_ctrl[13].value = pEncPara->initialQp;
          ext_ctrl[14].id = V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP;
          ext_ctrl[14].value = pEncPara->initialQp;

          ext_ctrls.count += 2;
        }
        //ext_ctrl[15].id = V4L2_CID_MPEG_VIDEO_H264_PROFILE;
        //ext_ctrl[15].value = ;
      } else if (hEnc->codecType == V4L2_PIX_FMT_MPEG4) {
        ext_ctrl[11].id = V4L2_CID_MPEG_VIDEO_MPEG4_MAX_QP;
        ext_ctrl[11].value = pEncPara->maximumQp;

        ext_ctrls.count = 12;

        if ((pEncPara->bitrate == 0) || (pEncPara->initialQp > 0)) {
          ext_ctrl[12].id = V4L2_CID_MPEG_VIDEO_MPEG4_I_FRAME_QP;
          ext_ctrl[12].value = pEncPara->initialQp;
          ext_ctrl[13].id = V4L2_CID_MPEG_VIDEO_MPEG4_P_FRAME_QP;
          ext_ctrl[13].value = pEncPara->initialQp;

          ext_ctrls.count += 2;
        }
      } else if (hEnc->codecType == V4L2_PIX_FMT_H263) {
        ext_ctrl[11].id = V4L2_CID_MPEG_VIDEO_H263_PROFILE;
        ext_ctrl[11].value = pEncPara->profile;
        ext_ctrl[12].id = V4L2_CID_MPEG_VIDEO_H263_MAX_QP;
        ext_ctrl[12].value = pEncPara->maximumQp;

        ext_ctrls.count = 13;

        if ((pEncPara->bitrate == 0) || (pEncPara->initialQp > 0)) {
          ext_ctrl[13].id = V4L2_CID_MPEG_VIDEO_H263_I_FRAME_QP;
          ext_ctrl[13].value = pEncPara->initialQp;
          ext_ctrl[14].id = V4L2_CID_MPEG_VIDEO_H263_P_FRAME_QP;
          ext_ctrl[14].value = pEncPara->initialQp;

          ext_ctrls.count += 2;
        }
      }

      ext_ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;
      ext_ctrls.controls = ext_ctrl;

      if (ioctl (hEnc->fd, VIDIOC_S_EXT_CTRLS, &ext_ctrls) != 0) {
        _E ("Fail, ioctl(): VIDIOC_S_EXT_CTRLS\n");
        return -1;
      }
    } else {
      struct v4l2_control ctrl;

      ctrl.id = V4L2_CID_JPEG_COMPRESSION_QUALITY;
      ctrl.value = pEncPara->jpgQuality;

      if (ioctl (hEnc->fd, VIDIOC_S_CTRL, &ctrl) != 0) {
        _E ("Fail, ioctl(): VIDIOC_S_CTRL\n");
        return -1;
      }
    }
  }

  /* Malloc Input Image Buffer */
  {
    struct v4l2_requestbuffers req;
    int32_t bufferCount = pEncPara->imgBufferNum;

    /* IOCTL : VIDIOC_REQBUFS For Input Yuv */
    memset (&req, 0, sizeof (req));
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.count = bufferCount;
    req.memory = V4L2_MEMORY_DMABUF;    /* V4L2_MEMORY_USERPTR, V4L2_MEMORY_DMABUF, V4L2_MEMORY_MMAP */

    if (ioctl (hEnc->fd, VIDIOC_REQBUFS, &req) != 0) {
      _E ("failed to ioctl: VIDIOC_REQBUFS(Input YUV)\n");
      return -1;
    }
  }

  /* Malloc Output Stream Buffer */
  {
    struct v4l2_requestbuffers req;
    int bufferCount = STREAM_BUFFER_NUM;

    /* IOCTL : VIDIOC_REQBUFS For Output Stream */
    memset (&req, 0, sizeof (req));
    req.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    req.count = bufferCount;
    req.memory = V4L2_MEMORY_DMABUF;

    if (ioctl (hEnc->fd, VIDIOC_REQBUFS, &req) != 0) {
      _E ("failed to ioctl: VIDIOC_REQBUFS(Output ES)\n");
      return -1;
    }

    /* Allocate Output Buffer */
    for (i = 0; i < bufferCount; i++) {
      hEnc->hBitStream[i] =
          NX_AllocateMemory (inWidth * inHeight * 3 / 4, 4096);
      if (hEnc->hBitStream[i] == NULL) {
        _E ("Failed to allocate stream buffer(%d, %d)\n", i,
            inWidth * inHeight * 3 / 4);
        return -1;
      }

      if (NX_MapMemory (hEnc->hBitStream[i]) != 0) {
        _E ("Stream memory Mapping Failed\n");
        return -1;
      }
    }

    hEnc->outBuffCnt = bufferCount;
  }

  /* Get Sequence header */
  {
    struct v4l2_plane planes[1];
    struct v4l2_buffer buf;
    enum v4l2_buf_type type;

    memset (&buf, 0, sizeof (buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    buf.m.planes = planes;
    buf.length = 1;
    buf.memory = V4L2_MEMORY_DMABUF;
    buf.index = 0;

    buf.m.planes[0].m.fd = hEnc->hBitStream[0]->dmaFd;
    //buf.m.planes[0].m.userptr = (unsigned long)hEnc->hBitStream[0]->pBuffer;
    buf.m.planes[0].length = hEnc->hBitStream[0]->size;
    buf.m.planes[0].bytesused = hEnc->hBitStream[0]->size;
    buf.m.planes[0].data_offset = 0;

    if (ioctl (hEnc->fd, VIDIOC_QBUF, &buf) != 0) {
      _E ("failed to ioctl: VIDIOC_QBUF(Header)\n");
      return -1;
    }

    type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    if (ioctl (hEnc->fd, VIDIOC_STREAMON, &type) != 0) {
      _E ("failed to ioctl: VIDIOC_STREAMON\n");
      return -1;
    }

    if (ioctl (hEnc->fd, VIDIOC_DQBUF, &buf) != 0) {
      _E ("failed to ioctl: VIDIOC_DQBUF(Header)\n");
      return -1;
    }

    if (buf.m.planes[0].bytesused > 0) {
      hEnc->seqSize = buf.m.planes[0].bytesused;
      hEnc->pSeqBuf = (uint8_t *) malloc (hEnc->seqSize);
      memcpy (hEnc->pSeqBuf, (void *) hEnc->hBitStream[buf.index]->pBuffer,
          hEnc->seqSize);
    }
  }

  /* Input Image Stream On */
  {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

    if (ioctl (hEnc->fd, VIDIOC_STREAMON, &type) != 0) {
      _E ("Fail, ioctl(): VIDIOC_STREAMON(Image)\n");
      return -1;
    }
  }

  return 0;
}

/*----------------------------------------------------------------------------*/
int32_t
NX_V4l2EncGetSeqInfo (NX_V4L2ENC_HANDLE hEnc, uint8_t ** ppSeqBuf,
    int32_t * iSeqSize)
{
  if (hEnc == NULL) {
    _E ("Fail, Invalid Handle.\n");
    return -1;
  }

  *ppSeqBuf = hEnc->pSeqBuf;
  *iSeqSize = hEnc->seqSize;

  return 0;
}

/*----------------------------------------------------------------------------*/
int32_t
NX_V4l2EncEncodeFrame (NX_V4L2ENC_HANDLE hEnc, NX_V4L2ENC_IN * pEncIn,
    NX_V4L2ENC_OUT * pEncOut)
{
  struct v4l2_plane planes[3];
  struct v4l2_buffer buf;
  int i;

  if (hEnc == NULL) {
    _E ("Fail, Invalid Handle.\n");
    return -1;
  }

  memset (pEncOut, 0, sizeof (NX_V4L2ENC_OUT));

  /* Set Encode Parameter */
  if (hEnc->codecType != V4L2_PIX_FMT_MJPEG) {
    struct v4l2_control ctrl;

    ctrl.id = V4L2_CID_MPEG_VIDEO_FORCE_I_FRAME;
    ctrl.value = pEncIn->forcedIFrame;

    if (ioctl (hEnc->fd, VIDIOC_S_CTRL, &ctrl) != 0) {
      _E ("failed to ioctl: Forced Intra Frame\n");
      return -1;
    }

    ctrl.id = V4L2_CID_MPEG_VIDEO_FORCE_SKIP_FRAME;
    ctrl.value = pEncIn->forcedSkipFrame;

    if (ioctl (hEnc->fd, VIDIOC_S_CTRL, &ctrl) != 0) {
      _E ("failed to ioctl: Forced Skip Frame\n");
      return -1;
    }

    if (hEnc->codecType == V4L2_PIX_FMT_H264)
      ctrl.id = V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP;
    else if (hEnc->codecType == V4L2_PIX_FMT_MPEG4)
      ctrl.id = V4L2_CID_MPEG_VIDEO_MPEG4_P_FRAME_QP;
    else if (hEnc->codecType == V4L2_PIX_FMT_H263)
      ctrl.id = V4L2_CID_MPEG_VIDEO_H263_P_FRAME_QP;

    ctrl.value = pEncIn->quantParam;

    if (ioctl (hEnc->fd, VIDIOC_S_CTRL, &ctrl) != 0) {
      _E ("failed to ioctl: Forced QP\n");
      return -1;
    }
  }

  /* Queue Input Buffer */
  memset (&buf, 0, sizeof (buf));
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  buf.m.planes = planes;
  buf.length = hEnc->planesNum;
  buf.memory = V4L2_MEMORY_DMABUF;
  buf.index = pEncIn->imgIndex;

  for (i = 0; i < hEnc->planesNum; i++) {
    buf.m.planes[i].m.fd = pEncIn->pImage->dmaFd[i];
    buf.m.planes[i].length = pEncIn->pImage->size[i];
  }

  if (ioctl (hEnc->fd, VIDIOC_QBUF, &buf) != 0) {
    _E ("Fail, ioctl(): VIDIOC_QBUF(Image)\n");
    return -1;
  }

  /* Queue Output Bitstream Buffer */
  {
    int idx = hEnc->frameCnt % hEnc->outBuffCnt;

    memset (&buf, 0, sizeof (buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    buf.m.planes = planes;
    buf.length = 1;
    buf.memory = V4L2_MEMORY_DMABUF;
    buf.index = idx;

    buf.m.planes[0].m.fd = hEnc->hBitStream[idx]->dmaFd;
    //buf.m.planes[0].m.userptr = (unsigned long)hBitStream[idx]->pBuffer;
    buf.m.planes[0].length = hEnc->hBitStream[idx]->size;
    buf.m.planes[0].bytesused = hEnc->hBitStream[idx]->size;
    buf.m.planes[0].data_offset = 0;

    if (ioctl (hEnc->fd, VIDIOC_QBUF, &buf) != 0) {
      _E ("failed to ioctl: VIDIOC_QBUF(Stream)\n");
      return -1;
    }
  }

  /* Dequeue Input Image Buffer */
  {
    memset (&buf, 0, sizeof (buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.m.planes = planes;
    buf.length = hEnc->planesNum;
    buf.memory = V4L2_MEMORY_DMABUF;

    if (ioctl (hEnc->fd, VIDIOC_DQBUF, &buf) != 0) {
      _E ("Fail, ioctl(): VIDIOC_DQBUF(Input Image)\n");
      return -1;
    }
  }

  /* Dequeue Output Stream Buffer */
  {
    memset (&buf, 0, sizeof (buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    buf.m.planes = planes;
    buf.length = 1;
    buf.memory = V4L2_MEMORY_DMABUF;    //V4L2_MEMORY_USERPTR;

    if (ioctl (hEnc->fd, VIDIOC_DQBUF, &buf) != 0) {
      _E ("Fail, ioctl(): VIDIOC_DQBUF(Compressed Stream)\n");
      return -1;
    }

    pEncOut->strmBuf = (uint8_t *) hEnc->hBitStream[buf.index]->pBuffer;
    pEncOut->strmSize = buf.m.planes[0].bytesused;

    if (buf.flags & V4L2_BUF_FLAG_KEYFRAME)
      pEncOut->frameType = PIC_TYPE_I;
    else if (buf.flags & V4L2_BUF_FLAG_PFRAME)
      pEncOut->frameType =
          (pEncIn->forcedSkipFrame == 0) ? (PIC_TYPE_P) : (PIC_TYPE_SKIP);
    else if (buf.flags & V4L2_BUF_FLAG_BFRAME)
      pEncOut->frameType = PIC_TYPE_B;
    else
      pEncOut->frameType = PIC_TYPE_UNKNOWN;
  }

  hEnc->frameCnt++;

  return 0;
}

int32_t
NX_V4l2EncChangeParameter (NX_V4L2ENC_HANDLE hEnc,
    NX_V4L2ENC_CHG_PARA * pChgPara)
{
  struct v4l2_control ctrl;

  if (hEnc == NULL) {
    _E ("Fail, Invalid Handle.\n");
    return -1;
  }

  if (pChgPara->chgFlg & VID_CHG_KEYFRAME) {
    ctrl.id = V4L2_CID_MPEG_VIDEO_GOP_SIZE;
    ctrl.value = pChgPara->keyFrmInterval;

    if (ioctl (hEnc->fd, VIDIOC_S_CTRL, &ctrl) != 0) {
      _E ("failed to ioctl: Change Key Frame Interval\n");
      return -1;
    }
  }

  if (pChgPara->chgFlg & VID_CHG_BITRATE) {
    ctrl.id = V4L2_CID_MPEG_VIDEO_BITRATE;
    ctrl.value = pChgPara->bitrate;

    if (ioctl (hEnc->fd, VIDIOC_S_CTRL, &ctrl) != 0) {
      _E ("failed to ioctl: Change Bitrate\n");
      return -1;
    }
  }

  if (pChgPara->chgFlg & VID_CHG_FRAMERATE) {
    ctrl.id = V4L2_CID_MPEG_VIDEO_FPS_NUM;
    ctrl.value = pChgPara->fpsNum;

    if (ioctl (hEnc->fd, VIDIOC_S_CTRL, &ctrl) != 0) {
      _E ("failed to ioctl: Change Fps Number\n");
      return -1;
    }

    ctrl.id = V4L2_CID_MPEG_VIDEO_FPS_DEN;
    ctrl.value = pChgPara->fpsDen;

    if (ioctl (hEnc->fd, VIDIOC_S_CTRL, &ctrl) != 0) {
      _E ("failed to ioctl: Change Fps Density\n");
      return -1;
    }
  }

  if (pChgPara->chgFlg & VID_CHG_INTRARF) {
    ctrl.id = V4L2_CID_MPEG_VIDEO_CYCLIC_INTRA_REFRESH_MB;
    ctrl.value = pChgPara->numIntraRefreshMbs;

    if (ioctl (hEnc->fd, VIDIOC_S_CTRL, &ctrl) != 0) {
      _E ("failed to ioctl: Change Intra Refresh Mbs\n");
      return -1;
    }
  }

  return 0;
}
