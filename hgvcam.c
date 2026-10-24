// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Josemar Simao <josemarsimao@gmail.com>
 * Co-authored with Claude Opus 5.5 (Anthropic) <noreply@anthropic.com>
 *
 * hgvcam - userspace-fed V4L2 virtual cameras.
 *
 * Loading the module creates only the control device, /dev/hgvcam.  Each
 * producer (the player) creates its own camera there with HGVCAM_ADD:
 * an OUTPUT node it publishes frames to and a CAPTURE node for ordinary
 * camera clients, numbered "Hitgalut Virtual Camera N".  The camera is removed
 * when the producer closes the control device (or exits), or with
 * HGVCAM_REMOVE.  The producer owns video decoding, scaling and encoding;
 * this module only transports already-prepared frames.
 *
 * Each camera behaves like a webcam: clients pick a format, resolution and
 * frame rate from fixed lists (which include the video's native size and
 * rate).  The choice is reported to the producer (see hgvcam.h), which
 * then publishes frames of that format.  Frames whose format does not match
 * the camera's are dropped, never delivered.
 */
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/math64.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/videodev2.h>
#include <linux/vmalloc.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

#include "hgvcam.h"

#define HGVCAM_NAME          "hgvcam"
/* How many cameras may exist at the same time. */
#define HGVCAM_MAX_CAMERAS   15
#define HGVCAM_MAX_FRAME     (HGVCAM_MAX_WIDTH * HGVCAM_MAX_HEIGHT * 2U)
/* Smallest MJPEG buffer: tiny native sizes still need room for the JPEG headers. */
#define HGVCAM_MIN_JPEG      (128U * 1024U)
#define HGVCAM_MAX_SIZES     6
#define HGVCAM_MAX_INTERVALS 6

struct hgvcam_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

struct hgvcam_size {
	u32 width;
	u32 height;
};

struct hgvcam {
	struct v4l2_device v4l2_dev;
	struct video_device capture_vdev;
	struct video_device output_vdev;
	struct vb2_queue capture_q;
	struct vb2_queue output_q;
	struct v4l2_ctrl_handler ctrl_handler;
	unsigned int number;
	/* Control-device file that created the camera; protected by hgvcam_cameras_lock. */
	struct file *owner;
	/* Queue locks serialize each node's ioctls; state_lock protects shared state. */
	struct mutex capture_lock;
	struct mutex output_lock;
	struct mutex state_lock;
	struct list_head capture_buffers;
	void *frame;
	size_t frame_bytes;
	/* Advertised sizes: the standard ones plus the video's native size. */
	struct hgvcam_size sizes[HGVCAM_MAX_SIZES];
	unsigned int num_sizes;
	/* What the producer writes, and what camera clients negotiated. */
	struct v4l2_pix_format out_format;
	struct v4l2_pix_format cap_format;
	/* Frame rate of the producer's source, and the camera's frame rate. */
	struct v4l2_fract source_interval;
	struct v4l2_fract cap_interval;
	u32 sequence;
	bool frame_valid;
	bool capture_streaming;
	bool output_streaming;
};

static DEFINE_MUTEX(hgvcam_cameras_lock);
static struct hgvcam *hgvcam_cameras[HGVCAM_MAX_CAMERAS];

/* Formats of both nodes; the producer prepares (or encodes) each of them. */
static const u32 hgvcam_formats[] = {
	V4L2_PIX_FMT_YUYV,
	V4L2_PIX_FMT_MJPEG,
	V4L2_PIX_FMT_SBGGR8,
	V4L2_PIX_FMT_SGBRG8,
};

/* Standard resolutions and frame rates of the camera node. */
static const struct hgvcam_size hgvcam_standard_sizes[] = {
	{ 320, 240 },
	{ 640, 360 },
	{ 640, 480 },
	{ 1280, 720 },
	{ 1920, 1080 },
};

static const u32 hgvcam_camera_fps[] = { 60, 30, 25, 24, 15 };

static struct hgvcam_buffer *to_hgvcam_buffer(struct vb2_buffer *vb)
{
	return container_of(vb, struct hgvcam_buffer, vb.vb2_buf);
}

static bool hgvcam_supported_format(u32 pixelformat)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(hgvcam_formats); i++)
		if (hgvcam_formats[i] == pixelformat)
			return true;
	return false;
}

static void hgvcam_try_format(struct v4l2_pix_format *pix)
{
	if (!hgvcam_supported_format(pix->pixelformat))
		pix->pixelformat = V4L2_PIX_FMT_YUYV;
	pix->width = clamp_t(u32, pix->width ?: 640, 16, HGVCAM_MAX_WIDTH) & ~1U;
	pix->height = clamp_t(u32, pix->height ?: 480, 16, HGVCAM_MAX_HEIGHT) & ~1U;
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_DEFAULT;
	pix->xfer_func = V4L2_XFER_FUNC_DEFAULT;

	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
		pix->bytesperline = pix->width * 2;
		pix->sizeimage = pix->bytesperline * pix->height;
		break;
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
		pix->bytesperline = pix->width;
		pix->sizeimage = pix->width * pix->height;
		break;
	default:
		/* MJPEG frames vary in size; they never exceed the YUYV size. */
		pix->bytesperline = 0;
		pix->sizeimage = max(pix->width * pix->height * 2, HGVCAM_MIN_JPEG);
		break;
	}
}

static void hgvcam_camera_try_format(const struct hgvcam *dev, struct v4l2_pix_format *pix)
{
	const struct hgvcam_size *size;

	size = v4l2_find_nearest_size(dev->sizes, dev->num_sizes, width, height,
				      pix->width, pix->height);
	pix->width = size->width;
	pix->height = size->height;
	hgvcam_try_format(pix);
}

static bool hgvcam_known_size(const struct hgvcam *dev, u32 width, u32 height)
{
	unsigned int i;

	for (i = 0; i < dev->num_sizes; i++)
		if (dev->sizes[i].width == width && dev->sizes[i].height == height)
			return true;
	return false;
}

/* Builds the size list, smallest first, with the native size inserted in order. */
static void hgvcam_init_sizes(struct hgvcam *dev, u32 width, u32 height)
{
	struct v4l2_pix_format native = { .width = width, .height = height };
	unsigned int i;
	bool inserted = false;

	hgvcam_try_format(&native);
	for (i = 0; i < ARRAY_SIZE(hgvcam_standard_sizes); i++) {
		const struct hgvcam_size *standard = &hgvcam_standard_sizes[i];

		if (!inserted && (u64)native.width * native.height <=
				 (u64)standard->width * standard->height) {
			if (native.width != standard->width || native.height != standard->height)
				dev->sizes[dev->num_sizes++] =
					(struct hgvcam_size){ native.width, native.height };
			inserted = true;
		}
		dev->sizes[dev->num_sizes++] = *standard;
	}
	if (!inserted)
		dev->sizes[dev->num_sizes++] = (struct hgvcam_size){ native.width, native.height };
}

static bool hgvcam_same_format(const struct v4l2_pix_format *a,
			      const struct v4l2_pix_format *b)
{
	return a->pixelformat == b->pixelformat && a->width == b->width &&
	       a->height == b->height && a->sizeimage == b->sizeimage;
}

/* Compares frame rates: negative, zero or positive when a is slower, equal, faster. */
static int hgvcam_compare_fps(const struct v4l2_fract *a, const struct v4l2_fract *b)
{
	u64 lhs = (u64)a->denominator * b->numerator;
	u64 rhs = (u64)b->denominator * a->numerator;

	return lhs < rhs ? -1 : lhs > rhs;
}

/* Fills the advertised intervals, fastest first: the standard rates plus the source's. */
static unsigned int hgvcam_intervals(const struct hgvcam *dev, struct v4l2_fract *list)
{
	bool source_listed = !dev->source_interval.numerator;
	unsigned int i, count = 0;

	for (i = 0; i < ARRAY_SIZE(hgvcam_camera_fps); i++) {
		struct v4l2_fract standard = { 1, hgvcam_camera_fps[i] };

		if (!source_listed) {
			int cmp = hgvcam_compare_fps(&dev->source_interval, &standard);

			if (cmp >= 0) {
				if (cmp > 0)
					list[count++] = dev->source_interval;
				source_listed = true;
			}
		}
		list[count++] = standard;
	}
	if (!source_listed)
		list[count++] = dev->source_interval;
	return count;
}

static struct v4l2_fract hgvcam_nearest_interval(const struct hgvcam *dev,
						 const struct v4l2_fract *wanted)
{
	struct v4l2_fract list[HGVCAM_MAX_INTERVALS];
	unsigned int i, count = hgvcam_intervals(dev, list), best = 0;
	u64 wanted_mfps = div_u64((u64)wanted->denominator * 1000, wanted->numerator);
	u64 best_diff = U64_MAX;

	for (i = 0; i < count; i++) {
		u64 mfps = div_u64((u64)list[i].denominator * 1000, list[i].numerator);
		u64 diff = mfps > wanted_mfps ? mfps - wanted_mfps : wanted_mfps - mfps;

		if (diff < best_diff) {
			best_diff = diff;
			best = i;
		}
	}
	return list[best];
}

static void hgvcam_camera_info_locked(const struct hgvcam *dev,
				     struct hgvcam_camera *camera)
{
	memset(camera, 0, sizeof(*camera));
	camera->width = dev->cap_format.width;
	camera->height = dev->cap_format.height;
	camera->pixelformat = dev->cap_format.pixelformat;
	camera->interval_numerator = dev->cap_interval.numerator;
	camera->interval_denominator = dev->cap_interval.denominator;
	if (dev->capture_streaming)
		camera->flags |= HGVCAM_CAMERA_STREAMING;
}

/* Tells the producer that the camera's format, rate or capture state changed. */
static void hgvcam_notify_camera_locked(struct hgvcam *dev)
{
	struct v4l2_event event = { .type = HGVCAM_EVENT_CAMERA };
	struct hgvcam_camera camera;

	BUILD_BUG_ON(sizeof(camera) > sizeof(event.u.data));
	hgvcam_camera_info_locked(dev, &camera);
	memcpy(event.u.data, &camera, sizeof(camera));
	v4l2_event_queue(&dev->output_vdev, &event);
}

static void hgvcam_deliver_locked(struct hgvcam *dev)
{
	struct hgvcam_buffer *buf;
	void *dst;

	if (!dev->frame_valid || !dev->capture_streaming ||
	    list_empty(&dev->capture_buffers) ||
	    !hgvcam_same_format(&dev->out_format, &dev->cap_format))
		return;

	buf = list_first_entry(&dev->capture_buffers, struct hgvcam_buffer, list);
	list_del_init(&buf->list);
	dst = vb2_plane_vaddr(&buf->vb.vb2_buf, 0);
	if (!dst || dev->frame_bytes > vb2_plane_size(&buf->vb.vb2_buf, 0)) {
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		return;
	}
	memcpy(dst, dev->frame, dev->frame_bytes);
	vb2_set_plane_payload(&buf->vb.vb2_buf, 0, dev->frame_bytes);
	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	buf->vb.sequence = dev->sequence++;
	buf->vb.field = V4L2_FIELD_NONE;
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
}

static const struct v4l2_pix_format *hgvcam_queue_format(struct hgvcam *dev,
							struct vb2_queue *vq)
{
	return vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE ? &dev->cap_format : &dev->out_format;
}

static int hgvcam_queue_setup(struct vb2_queue *vq, unsigned int *nbufs,
			     unsigned int *nplanes, unsigned int sizes[],
			     struct device *alloc_devs[])
{
	struct hgvcam *dev = vb2_get_drv_priv(vq);
	u32 sizeimage = hgvcam_queue_format(dev, vq)->sizeimage;

	if (*nplanes)
		return sizes[0] < sizeimage ? -EINVAL : 0;
	*nplanes = 1;
	sizes[0] = sizeimage;
	return 0;
}

static int hgvcam_buf_prepare(struct vb2_buffer *vb)
{
	struct hgvcam *dev = vb2_get_drv_priv(vb->vb2_queue);

	return vb2_plane_size(vb, 0) < hgvcam_queue_format(dev, vb->vb2_queue)->sizeimage ?
	       -EINVAL : 0;
}

static void hgvcam_buf_queue(struct vb2_buffer *vb)
{
	struct hgvcam *dev = vb2_get_drv_priv(vb->vb2_queue);
	struct hgvcam_buffer *buf = to_hgvcam_buffer(vb);
	void *src;
	size_t bytes;

	mutex_lock(&dev->state_lock);
	if (vb->vb2_queue->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		/*
		 * Only park the buffer: it is filled when the producer publishes its
		 * next frame, so capture clients follow the producer's cadence instead
		 * of receiving the same frame again as fast as they can requeue.
		 */
		list_add_tail(&buf->list, &dev->capture_buffers);
		mutex_unlock(&dev->state_lock);
		return;
	}

	bytes = vb2_get_plane_payload(vb, 0);
	if (!bytes)
		bytes = dev->out_format.sizeimage;
	src = vb2_plane_vaddr(vb, 0);
	if (!src || bytes > HGVCAM_MAX_FRAME) {
		mutex_unlock(&dev->state_lock);
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		return;
	}
	memcpy(dev->frame, src, bytes);
	dev->frame_bytes = bytes;
	dev->frame_valid = true;
	hgvcam_deliver_locked(dev);
	mutex_unlock(&dev->state_lock);
	vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
}

static int hgvcam_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct hgvcam *dev = vb2_get_drv_priv(vq);

	mutex_lock(&dev->state_lock);
	if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		dev->capture_streaming = true;
		/* The producer works only while someone is capturing. */
		hgvcam_notify_camera_locked(dev);
	} else {
		dev->output_streaming = true;
	}
	mutex_unlock(&dev->state_lock);
	return 0;
}

static void hgvcam_return_capture_buffers_locked(struct hgvcam *dev)
{
	struct hgvcam_buffer *buf, *next;

	list_for_each_entry_safe(buf, next, &dev->capture_buffers, list) {
		list_del_init(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
}

static void hgvcam_stop_streaming(struct vb2_queue *vq)
{
	struct hgvcam *dev = vb2_get_drv_priv(vq);

	mutex_lock(&dev->state_lock);
	if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		dev->capture_streaming = false;
		hgvcam_return_capture_buffers_locked(dev);
		hgvcam_notify_camera_locked(dev);
	} else {
		dev->output_streaming = false;
	}
	mutex_unlock(&dev->state_lock);
}

static const struct vb2_ops hgvcam_qops = {
	.queue_setup = hgvcam_queue_setup,
	.buf_prepare = hgvcam_buf_prepare,
	.buf_queue = hgvcam_buf_queue,
	.start_streaming = hgvcam_start_streaming,
	.stop_streaming = hgvcam_stop_streaming,
};

static int hgvcam_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	struct video_device *vdev = video_devdata(file);
	struct hgvcam *dev = video_drvdata(file);

	strscpy(cap->driver, HGVCAM_NAME, sizeof(cap->driver));
	strscpy(cap->card, dev->capture_vdev.name, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s-%u",
		 HGVCAM_NAME, dev->number);
	cap->device_caps = vdev->device_caps;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int hgvcam_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(hgvcam_formats))
		return -EINVAL;
	f->pixelformat = hgvcam_formats[f->index];
	return 0;
}

/* Camera (capture) node. */

static int hgvcam_g_fmt_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	struct hgvcam *dev = video_drvdata(file);

	mutex_lock(&dev->state_lock);
	f->fmt.pix = dev->cap_format;
	mutex_unlock(&dev->state_lock);
	return 0;
}

static int hgvcam_try_fmt_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	hgvcam_camera_try_format(video_drvdata(file), &f->fmt.pix);
	return 0;
}

static int hgvcam_s_fmt_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	struct hgvcam *dev = video_drvdata(file);
	int ret = 0;

	hgvcam_camera_try_format(dev, &f->fmt.pix);
	mutex_lock(&dev->state_lock);
	if (!hgvcam_same_format(&f->fmt.pix, &dev->cap_format)) {
		if (vb2_is_busy(&dev->capture_q)) {
			ret = -EBUSY;
		} else {
			dev->cap_format = f->fmt.pix;
			hgvcam_notify_camera_locked(dev);
		}
	}
	mutex_unlock(&dev->state_lock);
	return ret;
}

static int hgvcam_enum_framesizes(struct file *file, void *priv,
				 struct v4l2_frmsizeenum *fsize)
{
	struct hgvcam *dev = video_drvdata(file);

	if (fsize->index >= dev->num_sizes || !hgvcam_supported_format(fsize->pixel_format))
		return -EINVAL;
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = dev->sizes[fsize->index].width;
	fsize->discrete.height = dev->sizes[fsize->index].height;
	return 0;
}

static int hgvcam_enum_frameintervals(struct file *file, void *priv,
				     struct v4l2_frmivalenum *fival)
{
	struct hgvcam *dev = video_drvdata(file);
	struct v4l2_fract list[HGVCAM_MAX_INTERVALS];
	unsigned int count;

	if (!hgvcam_known_size(dev, fival->width, fival->height) ||
	    !hgvcam_supported_format(fival->pixel_format))
		return -EINVAL;
	count = hgvcam_intervals(dev, list);
	if (fival->index >= count)
		return -EINVAL;
	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete = list[fival->index];
	return 0;
}

static int hgvcam_g_parm_cap(struct file *file, void *priv, struct v4l2_streamparm *parm)
{
	struct hgvcam *dev = video_drvdata(file);

	mutex_lock(&dev->state_lock);
	parm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	parm->parm.capture.timeperframe = dev->cap_interval;
	parm->parm.capture.readbuffers = 2;
	mutex_unlock(&dev->state_lock);
	return 0;
}

static int hgvcam_s_parm_cap(struct file *file, void *priv, struct v4l2_streamparm *parm)
{
	struct hgvcam *dev = video_drvdata(file);
	struct v4l2_fract *wanted = &parm->parm.capture.timeperframe;

	if (wanted->numerator && wanted->denominator) {
		struct v4l2_fract interval = hgvcam_nearest_interval(dev, wanted);

		mutex_lock(&dev->state_lock);
		if (interval.numerator != dev->cap_interval.numerator ||
		    interval.denominator != dev->cap_interval.denominator) {
			dev->cap_interval = interval;
			hgvcam_notify_camera_locked(dev);
		}
		mutex_unlock(&dev->state_lock);
	}
	return hgvcam_g_parm_cap(file, priv, parm);
}

/*
 * Producer (output) node.  While a producer owns the queue (it is writing
 * frames), any other file may only query: S_FMT and S_PARM fail with EBUSY.
 */

static int hgvcam_g_fmt_out(struct file *file, void *priv, struct v4l2_format *f)
{
	struct hgvcam *dev = video_drvdata(file);

	mutex_lock(&dev->state_lock);
	f->fmt.pix = dev->out_format;
	mutex_unlock(&dev->state_lock);
	return 0;
}

static int hgvcam_try_fmt_out(struct file *file, void *priv, struct v4l2_format *f)
{
	hgvcam_try_format(&f->fmt.pix);
	return 0;
}

static int hgvcam_s_fmt_out(struct file *file, void *priv, struct v4l2_format *f)
{
	struct hgvcam *dev = video_drvdata(file);
	int ret = 0;

	if (vb2_queue_is_busy(&dev->output_q, file))
		return -EBUSY;
	hgvcam_try_format(&f->fmt.pix);
	mutex_lock(&dev->state_lock);
	if (hgvcam_same_format(&f->fmt.pix, &dev->out_format)) {
		f->fmt.pix = dev->out_format;
	} else if (vb2_is_busy(&dev->output_q)) {
		/* Output buffers were sized for the current format. */
		ret = -EBUSY;
	} else {
		dev->out_format = f->fmt.pix;
		dev->frame_valid = false;
	}
	mutex_unlock(&dev->state_lock);
	return ret;
}

static int hgvcam_g_parm_out(struct file *file, void *priv, struct v4l2_streamparm *parm)
{
	struct hgvcam *dev = video_drvdata(file);

	mutex_lock(&dev->state_lock);
	parm->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
	parm->parm.output.timeperframe = dev->cap_interval;
	parm->parm.output.writebuffers = 2;
	mutex_unlock(&dev->state_lock);
	return 0;
}

/* The source's rate is fixed by HGVCAM_ADD; the camera's rate belongs to its clients. */
static int hgvcam_s_parm_out(struct file *file, void *priv, struct v4l2_streamparm *parm)
{
	struct hgvcam *dev = video_drvdata(file);

	if (vb2_queue_is_busy(&dev->output_q, file))
		return -EBUSY;
	return hgvcam_g_parm_out(file, priv, parm);
}

static int hgvcam_subscribe_event_out(struct v4l2_fh *fh,
				     const struct v4l2_event_subscription *sub)
{
	if (sub->type == HGVCAM_EVENT_CAMERA)
		return v4l2_event_subscribe(fh, sub, 4, NULL);
	return v4l2_ctrl_subscribe_event(fh, sub);
}

static long hgvcam_default_out(struct file *file, void *priv, bool valid_prio,
			      unsigned int cmd, void *arg)
{
	struct hgvcam *dev = video_drvdata(file);

	if (cmd != HGVCAM_G_CAMERA)
		return -ENOTTY;
	mutex_lock(&dev->state_lock);
	hgvcam_camera_info_locked(dev, arg);
	mutex_unlock(&dev->state_lock);
	return 0;
}

/* Controls: the module keeps the values; the producer applies them to the picture. */

static int hgvcam_s_ctrl(struct v4l2_ctrl *ctrl)
{
	return 0;
}

static const struct v4l2_ctrl_ops hgvcam_ctrl_ops = {
	.s_ctrl = hgvcam_s_ctrl,
};

static const struct v4l2_ctrl_config hgvcam_pause_ctrl = {
	.ops = &hgvcam_ctrl_ops,
	.id = HGVCAM_CID_PAUSE,
	.name = "Pause virtual camera",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
};

static const struct v4l2_ctrl_config hgvcam_black_ctrl = {
	.ops = &hgvcam_ctrl_ops,
	.id = HGVCAM_CID_BLACK,
	.name = "Black virtual camera",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
};

static int hgvcam_init_controls(struct v4l2_ctrl_handler *hdl)
{
	v4l2_ctrl_handler_init(hdl, 11);
	v4l2_ctrl_new_custom(hdl, &hgvcam_pause_ctrl, NULL);
	v4l2_ctrl_new_custom(hdl, &hgvcam_black_ctrl, NULL);
	v4l2_ctrl_new_std(hdl, &hgvcam_ctrl_ops, V4L2_CID_BRIGHTNESS,
			  HGVCAM_BRIGHTNESS_MIN, HGVCAM_BRIGHTNESS_MAX, 1, 0);
	v4l2_ctrl_new_std(hdl, &hgvcam_ctrl_ops, V4L2_CID_CONTRAST,
			  0, HGVCAM_PERCENT_MAX, 1, 100);
	v4l2_ctrl_new_std(hdl, &hgvcam_ctrl_ops, V4L2_CID_SATURATION,
			  0, HGVCAM_PERCENT_MAX, 1, 100);
	v4l2_ctrl_new_std(hdl, &hgvcam_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(hdl, &hgvcam_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(hdl, &hgvcam_ctrl_ops, V4L2_CID_ZOOM_ABSOLUTE,
			  HGVCAM_ZOOM_MIN, HGVCAM_ZOOM_MAX, HGVCAM_ZOOM_STEP,
			  HGVCAM_ZOOM_MIN);
	v4l2_ctrl_new_std(hdl, &hgvcam_ctrl_ops, V4L2_CID_PAN_ABSOLUTE,
			  -HGVCAM_PAN_MAX, HGVCAM_PAN_MAX, 1, 0);
	v4l2_ctrl_new_std(hdl, &hgvcam_ctrl_ops, V4L2_CID_TILT_ABSOLUTE,
			  -HGVCAM_PAN_MAX, HGVCAM_PAN_MAX, 1, 0);
	return hdl->error;
}

static const struct v4l2_ioctl_ops hgvcam_capture_ioctl_ops = {
	.vidioc_querycap = hgvcam_querycap,
	.vidioc_enum_fmt_vid_cap = hgvcam_enum_fmt,
	.vidioc_g_fmt_vid_cap = hgvcam_g_fmt_cap,
	.vidioc_try_fmt_vid_cap = hgvcam_try_fmt_cap,
	.vidioc_s_fmt_vid_cap = hgvcam_s_fmt_cap,
	.vidioc_enum_framesizes = hgvcam_enum_framesizes,
	.vidioc_enum_frameintervals = hgvcam_enum_frameintervals,
	.vidioc_g_parm = hgvcam_g_parm_cap,
	.vidioc_s_parm = hgvcam_s_parm_cap,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct v4l2_ioctl_ops hgvcam_output_ioctl_ops = {
	.vidioc_querycap = hgvcam_querycap,
	.vidioc_enum_fmt_vid_out = hgvcam_enum_fmt,
	.vidioc_g_fmt_vid_out = hgvcam_g_fmt_out,
	.vidioc_try_fmt_vid_out = hgvcam_try_fmt_out,
	.vidioc_s_fmt_vid_out = hgvcam_s_fmt_out,
	.vidioc_g_parm = hgvcam_g_parm_out,
	.vidioc_s_parm = hgvcam_s_parm_out,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_subscribe_event = hgvcam_subscribe_event_out,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	.vidioc_default = hgvcam_default_out,
};

static const struct v4l2_file_operations hgvcam_capture_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.read = vb2_fop_read,
	.mmap = vb2_fop_mmap,
	.poll = vb2_fop_poll,
};

static const struct v4l2_file_operations hgvcam_output_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.write = vb2_fop_write,
	.mmap = vb2_fop_mmap,
	.poll = vb2_fop_poll,
};

static int hgvcam_init_queue(struct hgvcam *dev, struct vb2_queue *q,
			   enum v4l2_buf_type type, struct mutex *lock)
{
	q->type = type;
	/* read() emulation exists only for capture, write() only for output. */
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF |
		      (V4L2_TYPE_IS_OUTPUT(type) ? VB2_WRITE : VB2_READ);
	/* Each write() is one frame, so MJPEG frames of any size can be published. */
	q->fileio_write_immediately = V4L2_TYPE_IS_OUTPUT(type);
	q->drv_priv = dev;
	q->buf_struct_size = sizeof(struct hgvcam_buffer);
	q->ops = &hgvcam_qops;
	q->mem_ops = &vb2_vmalloc_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = lock;
	q->min_queued_buffers = 1;
	return vb2_queue_init(q);
}

static void hgvcam_init_vdev(struct video_device *vdev, struct hgvcam *dev,
			    const struct v4l2_file_operations *fops,
			    const struct v4l2_ioctl_ops *ioctl_ops, struct vb2_queue *q,
			    struct mutex *lock, u32 device_caps)
{
	/*
	 * The V4L2 core enables the *_vid_out ioctls only on transmitter nodes;
	 * with the default VFL_DIR_RX every format ioctl on the producer fails
	 * with ENOTTY.
	 */
	vdev->vfl_dir = (device_caps & V4L2_CAP_VIDEO_OUTPUT) ? VFL_DIR_TX : VFL_DIR_RX;
	vdev->v4l2_dev = &dev->v4l2_dev;
	vdev->fops = fops;
	vdev->ioctl_ops = ioctl_ops;
	/* The nodes live inside struct hgvcam, freed by hgvcam_release(). */
	vdev->release = video_device_release_empty;
	vdev->lock = lock;
	vdev->queue = q;
	vdev->ctrl_handler = &dev->ctrl_handler;
	vdev->device_caps = device_caps;
	video_set_drvdata(vdev, dev);
}

/* Camera lifetime. */

/* Runs when the last reference goes: removal done and every node file closed. */
static void hgvcam_release(struct v4l2_device *v4l2_dev)
{
	struct hgvcam *dev = container_of(v4l2_dev, struct hgvcam, v4l2_dev);

	v4l2_ctrl_handler_free(&dev->ctrl_handler);
	v4l2_device_unregister(&dev->v4l2_dev);
	vfree(dev->frame);
	kfree(dev);
}

static int hgvcam_create_locked(struct file *owner, struct hgvcam_add *add)
{
	struct hgvcam *dev;
	unsigned int slot;
	int ret;

	for (slot = 0; slot < HGVCAM_MAX_CAMERAS && hgvcam_cameras[slot]; slot++)
		;
	if (slot == HGVCAM_MAX_CAMERAS)
		return -ENOSPC;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	dev->frame = vzalloc(HGVCAM_MAX_FRAME);
	if (!dev->frame) {
		kfree(dev);
		return -ENOMEM;
	}
	mutex_init(&dev->capture_lock);
	mutex_init(&dev->output_lock);
	mutex_init(&dev->state_lock);
	INIT_LIST_HEAD(&dev->capture_buffers);
	dev->number = slot + 1;
	dev->owner = owner;

	/* The camera starts at the video's native size and frame rate. */
	hgvcam_init_sizes(dev, add->width, add->height);
	if (add->interval_numerator && add->interval_denominator)
		dev->source_interval = (struct v4l2_fract){ add->interval_numerator,
							    add->interval_denominator };
	dev->cap_interval = dev->source_interval.numerator ?
			    dev->source_interval : (struct v4l2_fract){ 1, 30 };
	dev->cap_format.width = add->width;
	dev->cap_format.height = add->height;
	dev->cap_format.pixelformat = V4L2_PIX_FMT_YUYV;
	hgvcam_camera_try_format(dev, &dev->cap_format);
	dev->out_format = dev->cap_format;

	snprintf(dev->v4l2_dev.name, sizeof(dev->v4l2_dev.name), "%s-%u",
		 HGVCAM_NAME, dev->number);
	ret = v4l2_device_register(NULL, &dev->v4l2_dev);
	if (ret) {
		vfree(dev->frame);
		kfree(dev);
		return ret;
	}
	/* From here on, the last v4l2_device_put() frees everything. */
	dev->v4l2_dev.release = hgvcam_release;
	ret = hgvcam_init_controls(&dev->ctrl_handler);
	if (ret)
		goto err;
	ret = hgvcam_init_queue(dev, &dev->capture_q, V4L2_BUF_TYPE_VIDEO_CAPTURE,
			       &dev->capture_lock);
	if (ret)
		goto err;
	ret = hgvcam_init_queue(dev, &dev->output_q, V4L2_BUF_TYPE_VIDEO_OUTPUT,
			       &dev->output_lock);
	if (ret)
		goto err;

	snprintf(dev->capture_vdev.name, sizeof(dev->capture_vdev.name),
		 "Hitgalut Virtual Camera %u", dev->number);
	hgvcam_init_vdev(&dev->capture_vdev, dev, &hgvcam_capture_fops,
			&hgvcam_capture_ioctl_ops, &dev->capture_q, &dev->capture_lock,
			V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_READWRITE);
	ret = video_register_device(&dev->capture_vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err;
	snprintf(dev->output_vdev.name, sizeof(dev->output_vdev.name),
		 "Hitgalut Virtual Camera Producer %u", dev->number);
	hgvcam_init_vdev(&dev->output_vdev, dev, &hgvcam_output_fops,
			&hgvcam_output_ioctl_ops, &dev->output_q, &dev->output_lock,
			V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING | V4L2_CAP_READWRITE);
	ret = video_register_device(&dev->output_vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err;

	hgvcam_cameras[slot] = dev;
	add->number = dev->number;
	add->capture_nr = dev->capture_vdev.num;
	add->output_nr = dev->output_vdev.num;
	pr_info(HGVCAM_NAME ": camera %u: /dev/video%d, producer /dev/video%d\n",
		dev->number, dev->capture_vdev.num, dev->output_vdev.num);
	return 0;
err:
	video_unregister_device(&dev->output_vdev);
	video_unregister_device(&dev->capture_vdev);
	v4l2_device_put(&dev->v4l2_dev);
	return ret;
}

/*
 * The nodes disappear at once; files still open on them (an application that
 * kept the camera open) see ENODEV, and the memory goes with the last close.
 */
static void hgvcam_remove_locked(struct hgvcam *dev)
{
	hgvcam_cameras[dev->number - 1] = NULL;
	pr_info(HGVCAM_NAME ": camera %u removed\n", dev->number);
	video_unregister_device(&dev->output_vdev);
	video_unregister_device(&dev->capture_vdev);
	v4l2_device_put(&dev->v4l2_dev);
}

/* Control device: /dev/hgvcam. */

static long hgvcam_control_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct hgvcam_add add;
	u32 number;
	int ret;

	switch (cmd) {
	case HGVCAM_ADD:
		if (copy_from_user(&add, argp, sizeof(add)))
			return -EFAULT;
		mutex_lock(&hgvcam_cameras_lock);
		ret = hgvcam_create_locked(file, &add);
		mutex_unlock(&hgvcam_cameras_lock);
		/* On a copy failure the camera still goes away with this file. */
		if (!ret && copy_to_user(argp, &add, sizeof(add)))
			ret = -EFAULT;
		return ret;
	case HGVCAM_REMOVE:
		if (get_user(number, (u32 __user *)argp))
			return -EFAULT;
		if (!number || number > HGVCAM_MAX_CAMERAS)
			return -ENOENT;
		mutex_lock(&hgvcam_cameras_lock);
		if (!hgvcam_cameras[number - 1]) {
			ret = -ENOENT;
		} else if (hgvcam_cameras[number - 1]->owner != file) {
			ret = -EPERM;
		} else {
			hgvcam_remove_locked(hgvcam_cameras[number - 1]);
			ret = 0;
		}
		mutex_unlock(&hgvcam_cameras_lock);
		return ret;
	default:
		return -ENOTTY;
	}
}

/* A producer that exits, however it exits, takes its cameras with it. */
static int hgvcam_control_release(struct inode *inode, struct file *file)
{
	unsigned int slot;

	mutex_lock(&hgvcam_cameras_lock);
	for (slot = 0; slot < HGVCAM_MAX_CAMERAS; slot++)
		if (hgvcam_cameras[slot] && hgvcam_cameras[slot]->owner == file)
			hgvcam_remove_locked(hgvcam_cameras[slot]);
	mutex_unlock(&hgvcam_cameras_lock);
	return 0;
}

static const struct file_operations hgvcam_control_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = hgvcam_control_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
	.release = hgvcam_control_release,
	.llseek = noop_llseek,
};

static struct miscdevice hgvcam_control_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = HGVCAM_NAME,
	.fops = &hgvcam_control_fops,
};

static int __init hgvcam_init(void)
{
	return misc_register(&hgvcam_control_device);
}

static void __exit hgvcam_exit(void)
{
	unsigned int slot;

	misc_deregister(&hgvcam_control_device);
	/* Every control file is closed by now; this only guards against leaks. */
	mutex_lock(&hgvcam_cameras_lock);
	for (slot = 0; slot < HGVCAM_MAX_CAMERAS; slot++)
		if (hgvcam_cameras[slot])
			hgvcam_remove_locked(hgvcam_cameras[slot]);
	mutex_unlock(&hgvcam_cameras_lock);
}

module_init(hgvcam_init);
module_exit(hgvcam_exit);

MODULE_AUTHOR("Josemar Simao <josemarsimao@gmail.com>");
MODULE_AUTHOR("Claude Opus 5.5 (Anthropic) <noreply@anthropic.com>");
MODULE_DESCRIPTION("V4L2 virtual cameras fed by userspace producers");
MODULE_LICENSE("GPL");
