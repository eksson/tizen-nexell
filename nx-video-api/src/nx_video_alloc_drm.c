/*
 * Copyright (C) 2016  Nexell Co., Ltd.
 * Author: SeongO, Park <ray@nexell.co.kr>
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <errno.h>
#include <stdarg.h>
#include <ctype.h>
#include <string.h>
#include <strings.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>           //      PROT_READ/PROT_WRITE/MAP_SHARED/mmap/munmap

#include <nexell/nexell_drm.h>
#include <nx_video_alloc.h>

#include <libdrm/drm_fourcc.h>
#include <linux/videodev2.h>
#include <linux/videodev2_nxp_media.h>
#ifdef TIZEN_FEATURE_ARTIK530
#include <tbm_bufmgr.h>
#endif
#include "nx_video_log.h"

#define DRM_DEVICE_NAME "/dev/dri/renderD128"

#define DRM_IOCTL_NR(n)         _IOC_NR(n)
#define DRM_IOC_VOID            _IOC_NONE
#define DRM_IOC_READ            _IOC_READ
#define DRM_IOC_WRITE           _IOC_WRITE
#define DRM_IOC_READWRITE       _IOC_READ|_IOC_WRITE
#define DRM_IOC(dir, group, nr, size) _IOC(dir, group, nr, size)

#ifndef ALIGN
#define	ALIGN(X,N)	( (X+N-1) & (~(N-1)) )
#endif

#define	ALIGNED16(X)	ALIGN(X,16)

static int
drm_ioctl (int32_t drm_fd, uint32_t request, void *arg)
{
  int ret;

  do {
    ret = ioctl (drm_fd, request, arg);
  } while (ret == -1 && (errno == EINTR || errno == EAGAIN));
  return ret;
}

static int
drm_command_write_read (int fd, uint32_t command_index,
    void *data, uint32_t size)
{
  uint32_t request;

  request = DRM_IOC (DRM_IOC_READ | DRM_IOC_WRITE, DRM_IOCTL_BASE,
      DRM_COMMAND_BASE + command_index, size);
  if (drm_ioctl (fd, request, data))
    return -errno;
  return 0;
}

/**
 * return gem_fd
 */
static int
alloc_gem (int drm_fd, int size, int flags)
{
  struct nx_drm_gem_create arg = { 0, };
  int ret;

  arg.size = (uint32_t) size;
  arg.flags = flags;

  ret = drm_command_write_read (drm_fd, DRM_NX_GEM_CREATE, &arg, sizeof (arg));
  if (ret) {
    _E ("drm_command_write_read(errno=%d, flags=%d)\n", -ret, flags);
    return ret;
  }
  //_D ("[DRM ALLOC] gem %d, size %d, flags 0x%x\n", arg.handle, size, flags);

  return arg.handle;
}

static void
free_gem (int drm_fd, int gem)
{
  struct drm_gem_close arg = { 0, };

  arg.handle = gem;
  drm_ioctl (drm_fd, DRM_IOCTL_GEM_CLOSE, &arg);
}

/**
 * return dmabuf fd
 */
static int
gem_to_dmafd (int drm_fd, int gem_fd)
{
  int ret;
  struct drm_prime_handle arg = { 0, };

  arg.handle = gem_fd;
  arg.flags = O_CLOEXEC | O_RDWR;
  ret = drm_ioctl (drm_fd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &arg);
  if (0 != ret) {
    _E ("fail : get fd from gem:%d (DRM_IOCTL_PRIME_HANDLE_TO_FD)\n", ret);
    return -1;
  }
  return arg.fd;
}

static uint32_t
get_flink_name (int fd, int gem)
{
  struct drm_gem_flink arg = { 0, };

  arg.handle = gem;
  if (drm_ioctl (fd, DRM_IOCTL_GEM_FLINK, &arg)) {
    _E ("fail : get flink from gem:%d (DRM_IOCTL_GEM_FLINK)\n", gem);
    return 0;
  }
  return arg.name;
}

static uint32_t
gem_from_flink (int fd, uint32_t flink_name)
{
  struct drm_gem_open arg = { 0, };
  /* struct nx_drm_gem_info info = { 0, }; */

  arg.name = flink_name;
  if (drm_ioctl (fd, DRM_IOCTL_GEM_OPEN, &arg)) {
    _E ("fail : cannot open gem name=%d\n", flink_name);
    return -EINVAL;
  }
  return arg.handle;
}

static int32_t
IsContinuousPlanes( uint32_t fourcc )
{
	switch (fourcc)	{
	case V4L2_PIX_FMT_YUV420M:
	case V4L2_PIX_FMT_YVU420M:
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV21M:
	case V4L2_PIX_FMT_YUV422M:
//	case V4L2_PIX_FMT_NV16M:
//	case V4L2_PIX_FMT_NV61M:
	case V4L2_PIX_FMT_YUV444M:
	case V4L2_PIX_FMT_NV24M:
	case V4L2_PIX_FMT_NV42M:
		return 0;
	}

	return 1;
}

//
//	Nexell Memory Allocator Wrapper
//
NX_MEMORY_INFO *
NX_AllocateMemory (int size, int align)
{
  int gemFd = -1;
  int dmaFd = -1;
  int32_t flags = 0;
  NX_MEMORY_INFO *pMem;

  int drmFd = open (DRM_DEVICE_NAME, O_RDWR);
  if (drmFd < 0)
    return NULL;

  drmDropMaster (drmFd);

  gemFd = alloc_gem (drmFd, size, flags);
  if (gemFd < 0)
    goto ErrorExit;

  dmaFd = gem_to_dmafd (drmFd, gemFd);
  if (dmaFd < 0)
    goto ErrorExit;

  pMem = (NX_MEMORY_INFO *) calloc (1, sizeof (NX_MEMORY_INFO));
  if (!pMem)
    goto ErrorExit;

  pMem->drmFd = drmFd;
  pMem->dmaFd = dmaFd;
  pMem->gemFd = gemFd;
  pMem->size = size;
  pMem->align = align;
  return pMem;

ErrorExit:
  if (gemFd > 0) {
    free_gem (drmFd, gemFd);
  }
  if (drmFd > 0)
    close (drmFd);
  return NULL;
}

void
NX_FreeMemory (NX_MEMORY_INFO * pMem)
{
  if (pMem) {
    if (pMem->pBuffer) {
      munmap (pMem->pBuffer, pMem->size);
    }

    free_gem (pMem->drmFd, pMem->gemFd);
    close (pMem->dmaFd);
    close (pMem->drmFd);
    free (pMem);
  }
}


//      Video Specific Allocator Wrapper
//
//      Suport Format & Planes
//              YUV420 Format :
//                      1 Plane : I420, NV12
//                      2 Plane : NV12
//                      3 Plane : I420
//
NX_VID_MEMORY_INFO *
NX_AllocateVideoMemory (void *bufmgr, int width, int height, int32_t planes,
    uint32_t format, int align)
{
  int gemFd[NX_MAX_PLANES] = { 0, };
  int dmaFd[NX_MAX_PLANES] = { 0, };
#ifdef TIZEN_FEATURE_ARTIK530
  int32_t flags = NEXELL_BO_DMA;
#else
  int32_t flags = 0;
#endif
  int32_t i = 0;
  int32_t luStride;
  int32_t cStride;
  int32_t luVStride;
  int32_t cVStride;
  int32_t stride[NX_MAX_PLANES];
  int32_t size[NX_MAX_PLANES];
  uint32_t flink[NX_MAX_PLANES];
#ifdef TIZEN_FEATURE_ARTIK530
  tbm_surface_info_s info;
#endif

  NX_VID_MEMORY_INFO *pVidMem = NULL;

  int drmFd = open (DRM_DEVICE_NAME, O_RDWR);
  if (drmFd < 0)
    return NULL;

  drmDropMaster (drmFd);

  //      Luma
  luStride = ALIGN (width, 32);
  luVStride = ALIGN (height, 16);

  //      Chroma
  switch (format) {
  	case V4L2_PIX_FMT_NV12:
  	case V4L2_PIX_FMT_NV21:
  	case V4L2_PIX_FMT_YUV420:	// DRM_FORMAT_YUV420
  	case V4L2_PIX_FMT_YVU420:	// DRM_FORMAT_YVU420
  	case V4L2_PIX_FMT_YUV420M:
  	case V4L2_PIX_FMT_YVU420M:
  	case V4L2_PIX_FMT_NV12M:
  	case V4L2_PIX_FMT_NV21M:
	  	cStride = luStride / 2;
		  cVStride = ALIGN (height / 2, 16);
  		break;

  	case DRM_FORMAT_YUV422:
  	case DRM_FORMAT_YVU422:
  	case DRM_FORMAT_NV16:
  	case DRM_FORMAT_NV61:
  	case V4L2_PIX_FMT_YUV422P:
  	case V4L2_PIX_FMT_YUV422M:
//  	case V4L2_PIX_FMT_NV16M:
//  	case V4L2_PIX_FMT_NV61M:
  		cStride = luStride / 2;
  		cVStride = luVStride;
  		break;

  	case DRM_FORMAT_YUV444:
  	case DRM_FORMAT_YVU444:
  	// case DRM_FORMAT_NV24:
  	// case DRM_FORMAT_NV42:
  	case V4L2_PIX_FMT_YUV444:
  	case V4L2_PIX_FMT_YUV444M:
  	case V4L2_PIX_FMT_NV24M:
  	case V4L2_PIX_FMT_NV42M:
  		cStride = luStride;
  		cVStride = luVStride;
  		break;

	  case V4L2_PIX_FMT_GREY:
  		cStride = 0;
  		cVStride = 0;
  		break;

	  default:
		  _E ("Unknown format type\n");
		  goto ErrorExit;
  }

  //      Decide Memory Size
  switch (planes) {
    case 1:
      size[0] = luStride * luVStride + cStride * cVStride * 2;
      stride[0] = luStride;

      gemFd[0] = alloc_gem (drmFd, size[0], flags);
      if (gemFd[0] < 0)
        goto ErrorExit;

      dmaFd[0] = gem_to_dmafd (drmFd, gemFd[0]);
      if (dmaFd[0] < 0)
        goto ErrorExit;
      break;

    case 2:
      size[0] = luStride * luVStride;
      stride[0] = luStride;

      size[1] = cStride * cVStride * 2;
      stride[1] = cStride * 2;

      if (IsContinuousPlanes (format)) {
        gemFd[0] =
        gemFd[1] = alloc_gem (drmFd, size[0] + size[1], flags);
        if (gemFd[0] < 0)
          goto ErrorExit;

        dmaFd[0] =
        dmaFd[1] = gem_to_dmafd (drmFd, gemFd[0]);
        if (dmaFd[0] < 0)
          goto ErrorExit;

        flink[0] =
			  flink[1] = get_flink_name (drmFd, gemFd[1]);
      } else {
		  	gemFd[0] = alloc_gem (drmFd, size[0], flags);
		  	if (gemFd[0] < 0)
          goto ErrorExit;

		  	dmaFd[0] = gem_to_dmafd (drmFd, gemFd[0]);
		  	if (dmaFd[0] < 0)
          goto ErrorExit;

		  	flink[0] = get_flink_name (drmFd, gemFd[0]);

		  	gemFd[1] = alloc_gem (drmFd, size[1], flags);
		  	if (gemFd[1] < 0)
          goto ErrorExit;

		  	dmaFd[1] = gem_to_dmafd (drmFd, gemFd[1]);
		  	if (dmaFd[1] < 0)
          goto ErrorExit;

		  	flink[1] = get_flink_name (drmFd, gemFd[1]);
      }
      break;
    case 3:
      size[0] = luStride * luVStride;
      stride[0] = luStride;

      size[1] = cStride * cVStride;
      stride[1] = cStride;

      size[2] = cStride * cVStride;
      stride[2] = cStride;

      if (IsContinuousPlanes (format)) {
        gemFd[0] = gemFd[1] = gemFd[2] =
            alloc_gem (drmFd, size[0] + size[1] + size[2], flags);
        if (gemFd[0] < 0)
          goto ErrorExit;

        dmaFd[0] = dmaFd[1] = dmaFd[2] =
              gem_to_dmafd (drmFd, gemFd[0]);
          if (dmaFd[0] < 0)
            goto ErrorExit;

			  flink[0] = flink[1] = flink[2] =
              get_flink_name (drmFd, gemFd[0]);
      } else {
			  gemFd[0] = alloc_gem (drmFd, size[0], flags);
			  if (gemFd[0] < 0)
          goto ErrorExit;

			  dmaFd[0] = gem_to_dmafd (drmFd, gemFd[0]);
			  if (dmaFd[0] < 0)
          goto ErrorExit;

			  flink[0] = get_flink_name (drmFd, gemFd[0]);

			  gemFd[1] = alloc_gem (drmFd, size[1], flags);
  			if (gemFd[1] < 0)
          goto ErrorExit;

	  		dmaFd[1] = gem_to_dmafd (drmFd, gemFd[1]);
		  	if (dmaFd[1] < 0)
          goto ErrorExit;

			  flink[1] = get_flink_name (drmFd, gemFd[1]);

  			gemFd[2] = alloc_gem (drmFd, size[2], flags);
	  		if (gemFd[2] < 0)
          goto ErrorExit;

		  	dmaFd[2] = gem_to_dmafd (drmFd, gemFd[2]);
			  if (dmaFd[2] < 0)
          goto ErrorExit;

			  flink[2] = get_flink_name (drmFd, gemFd[2]);
      }
      break;
  }

  pVidMem = (NX_VID_MEMORY_INFO *) calloc (1, sizeof (NX_VID_MEMORY_INFO));
  //if (!pVidMem)
  //  goto ErrorExit;

  pVidMem->width = width;
  pVidMem->height = height;
  pVidMem->align = align;
  pVidMem->planes = planes;
  pVidMem->format = format;
  pVidMem->drmFd = drmFd;

#ifdef TIZEN_FEATURE_ARTIK530
  info.width = width;
  info.height = height;
  info.format = TBM_FORMAT_YUV420;
  info.size = size[0];
  info.num_planes = 3;
  info.planes[0].stride = luStride;
  info.planes[0].size = luStride * luVStride;
  info.planes[0].offset = 0;
  info.planes[1].stride = cStride;
  info.planes[1].size = cStride * cVStride;
  info.planes[1].offset = info.planes[0].size;
  info.planes[2].stride = cStride;
  info.planes[2].size = cStride * cVStride;
  info.planes[2].offset = info.planes[1].offset + info.planes[1].size;
#endif

  for (i = 0; i < planes; i++) {
    pVidMem->dmaFd[i] = dmaFd[i];
    pVidMem->gemFd[i] = gemFd[i];
    pVidMem->size[i] = size[i];
    pVidMem->stride[i] = stride[i];
    pVidMem->flink[i] = flink[i];

#ifdef TIZEN_FEATURE_ARTIK530
    pVidMem->bo[i] = tbm_bo_import_fd (bufmgr, dmaFd[i]);
    if (!pVidMem->bo[i])
      goto alloc_bo_fail;
#endif
  }

#ifdef TIZEN_FEATURE_ARTIK530
  pVidMem->surface = tbm_surface_internal_create_with_bos (&info, pVidMem->bo, planes);
  if (!pVidMem->surface)
    goto alloc_bo_fail;
#endif

  return pVidMem;

#ifdef TIZEN_FEATURE_ARTIK530
alloc_bo_fail:
  for (i = 0; i < planes; i++) {
    if (pVidMem->bo[i])
      tbm_bo_unref (pVidMem->bo[i]);
  }
#endif

ErrorExit:
  for (i = 0; i < planes; i++) {
    if (gemFd[i] > 0) {
      free_gem (drmFd, gemFd[i]);
    }

    if (dmaFd[i] > 0) {
      close (dmaFd[i]);
    }
  }

  if (drmFd > 0)
    close (drmFd);

  return NULL;
}

void
NX_FreeVideoMemory (NX_VID_MEMORY_INFO * pMem)
{
  int32_t i;

  if (pMem) {

#ifdef TIZEN_FEATURE_ARTIK530
    if (pMem->bo[i])
      tbm_bo_unref (pMem->bo[i]);
#endif

	  NX_UnmapVideoMemory (pMem);

    for (i = 0; i < pMem->planes; i++) {
      free_gem (pMem->drmFd, pMem->gemFd[i]);
      close (pMem->dmaFd[i]);
    }

#ifdef TIZEN_FEATURE_ARTIK530
  if (pMem->surface)
    tbm_surface_internal_destroy (pMem->surface);
#endif

    close (pMem->drmFd);
    free (pMem);
  }
}

//
//              Memory Mapping/Unmapping Memory
//
int
NX_MapMemory (NX_MEMORY_INFO * pMem)
{
  char *pBuf;

  if (!pMem)
    return -1;

  //      Already Mapped
  if (pMem->pBuffer)
    return -1;

  pBuf = (char *) mmap (0, pMem->size, PROT_READ | PROT_WRITE,
          MAP_SHARED, pMem->dmaFd, 0);

  if (pBuf == MAP_FAILED) {
    _E ("Map failed : size %d, fd %d", pMem->size, pMem->dmaFd);
    return -1;
  }

  pMem->pBuffer = pBuf;
  return 0;
}

int
NX_UnmapMemory (NX_MEMORY_INFO * pMem)
{
  if (!pMem)
    return -1;

  if (!pMem->pBuffer)
    return -1;

  if (0 != munmap (pMem->pBuffer, pMem->size))
    return -1;

  pMem->pBuffer = NULL;

  return 0;
}

int
NX_MapVideoMemory (NX_VID_MEMORY_INFO * pMem)
{
  int32_t i;
  int32_t size = 0;
  void *pBuf;

  if (!pMem)
    return -1;

  if (IsContinuousPlanes (pMem->format)) {
		for (i = 0; i < pMem->planes; i++) {
			size += pMem->size[i];
		}

		for (i = 0; i < pMem->planes; i++) {
			pBuf = (i < 1 ) ?
				mmap (0, size, PROT_READ|PROT_WRITE, MAP_SHARED,
              pMem->dmaFd[i], 0) :
				pMem->pBuffer[i-1] + pMem->size[i-1];

			if (pBuf == MAP_FAILED) {
				return -1;
			}

			pMem->pBuffer[i] = pBuf;
		}
  } else {
		for (i = 0; i < pMem->planes; i++) {
			pBuf = mmap (0, pMem->size[i], PROT_READ|PROT_WRITE,
                  MAP_SHARED, pMem->dmaFd[i], 0);

			if (pBuf == MAP_FAILED) {
				return -1;
			}

			pMem->pBuffer[i] = pBuf;
		}
  }

  return 0;
}

int
NX_UnmapVideoMemory (NX_VID_MEMORY_INFO * pMem)
{
  int32_t i;
  int32_t size = 0;

  if (!pMem)
    return -1;

	if (IsContinuousPlanes (pMem->format)) {
		for (i = 0; i < pMem->planes; i++) {
			size += pMem->size[i];
		}

		if (pMem->pBuffer[0]) {
			munmap( pMem->pBuffer[0], size );
		} else {
			return -1;
		}
	} else {
		for (i = 0; i < pMem->planes; i++) {
			if (pMem->pBuffer[i]) {
				munmap( pMem->pBuffer[i], pMem->size[i] );
			} else {
				return -1;
			}
		}
	}
  return 0;
}

int
NX_GetGEMHandles (int drmFd, NX_VID_MEMORY_INFO * pMem,
    uint32_t handles[NX_MAX_PLANES])
{
  int32_t i;

  memset (handles, 0, sizeof (uint32_t) * NX_MAX_PLANES);

  for (i = 0; i < pMem->planes; i++) {
    handles[i] = gem_from_flink (drmFd, pMem->flink[i]);
    if (0 > (int) handles[i]) {
      return -1;
    }
  }
  return 0;
}

int
NX_GetGemHandle (int drmFd, NX_VID_MEMORY_INFO * pMem, int32_t plane)
{
  if (plane >= NX_MAX_PLANES || plane < 0)
    return -1;

  return gem_from_flink (drmFd, pMem->flink[plane]);
}
