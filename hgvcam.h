/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Copyright (C) 2026 Josemar Simao <josemarsimao@gmail.com>
 * Co-authored with Claude Opus 5.5 (Anthropic) <noreply@anthropic.com>
 *
 * Interface shared by the hgvcam module and its userspace producer.
 *
 * Cameras are created on demand: a producer opens the control device
 * (/dev/hgvcam) and issues HGVCAM_ADD, which creates a numbered pair
 * of V4L2 nodes, "Hitgalut Virtual Camera N" (capture, for applications) and
 * "Hitgalut Virtual Camera Producer N" (output, for the producer).  The camera
 * lives as long as the control-device file that created it stays open, or
 * until HGVCAM_REMOVE.
 *
 * The camera node chooses its own format and frame rate (from the lists the
 * module advertises).  The producer learns that choice through
 * HGVCAM_G_CAMERA and is told about later changes, including whether an
 * application is capturing, through the HGVCAM_EVENT_CAMERA event; it
 * then publishes frames of exactly that format.  Image controls are stored by
 * the module and applied by the producer.
 */
#ifndef HGVCAM_H
#define HGVCAM_H

#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/videodev2.h>

#define HGVCAM_CONTROL_DEVICE	"/dev/hgvcam"

#define HGVCAM_CID_PAUSE	(V4L2_CID_USER_BASE + 0x10f0)
#define HGVCAM_CID_BLACK	(V4L2_CID_USER_BASE + 0x10f1)

/* Image control ranges; the producer maps them onto the picture. */
#define HGVCAM_BRIGHTNESS_MIN	-100
#define HGVCAM_BRIGHTNESS_MAX	100
#define HGVCAM_PERCENT_MAX		200	/* contrast and saturation, 100 = unchanged */
#define HGVCAM_ZOOM_MIN		100	/* 1x */
#define HGVCAM_ZOOM_MAX		1000	/* 10x */
#define HGVCAM_ZOOM_STEP		10
#define HGVCAM_PAN_MAX		100	/* pan/tilt: -100..100, 0 = centre */

/* Largest frame size the module accepts (the video's native size is fitted to it). */
#define HGVCAM_MAX_WIDTH		1920
#define HGVCAM_MAX_HEIGHT		1080

/* Control device. */

struct hgvcam_add {
	/* in: the video's native size (fitted to the maximum) and frame rate */
	__u32 width;
	__u32 height;
	__u32 interval_numerator;
	__u32 interval_denominator;
	/* out */
	__u32 number;			/* N in the node names */
	__u32 capture_nr;		/* /dev/videoN of the camera */
	__u32 output_nr;		/* /dev/videoN of the producer */
	__u32 reserved[5];
};

#define HGVCAM_ADD		_IOWR('E', 1, struct hgvcam_add)
/* Removes camera number N; only the file that created it may do so. */
#define HGVCAM_REMOVE	_IOW('E', 2, __u32)

/* Producer node. */

#define HGVCAM_CAMERA_STREAMING	(1U << 0)	/* an application is capturing */

struct hgvcam_camera {
	__u32 width;
	__u32 height;
	__u32 pixelformat;
	__u32 interval_numerator;	/* camera frame interval, seconds */
	__u32 interval_denominator;
	__u32 flags;			/* HGVCAM_CAMERA_* */
	__u32 reserved[2];
};

/* Queued on the producer node; u.data holds a struct hgvcam_camera. */
#define HGVCAM_EVENT_CAMERA	(V4L2_EVENT_PRIVATE_START + 0x10)

/* Reads the camera's current format, frame rate and flags. */
#define HGVCAM_G_CAMERA	_IOR('V', BASE_VIDIOC_PRIVATE + 0, struct hgvcam_camera)

#endif /* HGVCAM_H */
