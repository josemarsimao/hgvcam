/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2026 Josemar Simao <josemarsimao@gmail.com>
 * Co-authored with Claude Opus 5.5 (Anthropic) <noreply@anthropic.com>
 *
 * FFmpeg-backed circular producer for hgvcam.
 *
 * Each player creates its own virtual camera through /dev/hgvcam and
 * removes it when it exits.  The camera's clients choose format, resolution
 * and frame rate; the module reports that choice (hgvcam.h) and this
 * player publishes frames of exactly that format, at that rate.  Each output
 * frame shows the video frame whose timestamp is due, so the video never runs
 * fast or slow.  While no application is capturing, the player does no work
 * and the video waits where it stopped.  Image controls (brightness,
 * contrast, saturation, flips, zoom, pan, tilt, pause, black) live in the
 * module and are applied here.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include "hgvcam.h"

#define RULE_FILE      "70-hgvcam.rules"
/* Frames decoded in one output tick before the media clock is resynchronised. */
#define MAX_CATCH_UP   120
/* MJPEG quality scale: 1 (best) .. 31 (smallest). */
#define JPEG_QSCALE    3

static volatile sig_atomic_t keep_running = 1;

struct settings {
	bool paused;
	bool black;
	int brightness;
	int contrast;
	int saturation;
	bool hflip;
	bool vflip;
	int zoom;
	int pan;
	int tilt;
};

/* Media time (seconds of video) advancing with the wall clock unless paused. */
struct media_clock {
	double base_media;
	double base_wall;
	double frozen;
	bool paused;
};

struct source {
	AVFormatContext *input;
	AVCodecContext *decoder;
	AVPacket *packet;
	int stream;
	AVRational rate;	/* nominal rate, for frames without timing */
	double loop_offset;	/* media time added to the current loop's timestamps */
	double end_time;	/* media time at which the latest frame ends */
	double last_time;
	bool have_time;
	bool draining;
	int64_t frames_in_loop;
	int64_t loop_count;
};

struct producer {
	char device[32];	/* producer node of this player's camera */
	int control_fd;		/* events, controls and format negotiation */
	int data_fd;		/* frames, via write(); reopened on format changes */
	struct hgvcam_camera camera;
	struct v4l2_pix_format pix;
	struct settings settings;
	struct media_clock clock;
	bool streaming;		/* an application is capturing */
	/* Composition: every format starts as a YUYV picture of the camera's size. */
	struct SwsContext *sws;
	AVFrame *cropped;
	AVFrame *scaled;
	uint8_t *canvas;
	int layout[4];		/* x, y, width, height of the picture in the canvas */
	uint8_t luma[256];
	uint8_t chroma[256];
	bool identity;
	/* Conversion of the canvas into the camera's format. */
	struct SwsContext *convert;
	AVFrame *converted;	/* RGB24 (Bayer) or YUVJ422P (MJPEG) */
	uint8_t *payload;	/* Bayer mosaic */
	AVCodecContext *encoder;
	AVPacket *jpeg;
	int64_t frame_count;
	bool warned_jpeg_size;
};

static const uint32_t subscribed_controls[] = {
	HGVCAM_CID_PAUSE,
	HGVCAM_CID_BLACK,
	V4L2_CID_BRIGHTNESS,
	V4L2_CID_CONTRAST,
	V4L2_CID_SATURATION,
	V4L2_CID_HFLIP,
	V4L2_CID_VFLIP,
	V4L2_CID_ZOOM_ABSOLUTE,
	V4L2_CID_PAN_ABSOLUTE,
	V4L2_CID_TILT_ABSOLUTE,
};

static void on_signal(int signal_number)
{
	(void)signal_number;
	keep_running = 0;
}

static void usage(const char *program)
{
	fprintf(stderr,
		"Usage: %s --file VIDEO [--module FILE] [--no-load]\n"
		"  --module FILE  hgvcam.ko to load (default: beside this executable)\n"
		"  --no-load      never load, reload or configure the module\n"
		"The player creates its own virtual camera and removes it when it exits.\n"
		"The module is loaded automatically when absent (or reloaded when the\n"
		"loaded build differs from FILE and nothing is using it), and the udev\n"
		"rule " RULE_FILE " beside this executable is installed; the\n"
		"administrator password is asked through sudo, or through pkexec when\n"
		"there is no terminal.\n",
		program);
}

static double now_seconds(void)
{
	struct timespec now;

	clock_gettime(CLOCK_MONOTONIC, &now);
	return now.tv_sec + now.tv_nsec / 1e9;
}

static double clock_time(const struct media_clock *clock)
{
	return clock->paused ? clock->frozen : clock->base_media + (now_seconds() - clock->base_wall);
}

static void clock_start(struct media_clock *clock, double media)
{
	clock->base_media = media;
	clock->base_wall = now_seconds();
	clock->frozen = media;
}

static void clock_set_paused(struct media_clock *clock, bool paused)
{
	if (paused == clock->paused)
		return;
	if (paused)
		clock->frozen = clock_time(clock);
	else
		clock_start(clock, clock->frozen);
	clock->paused = paused;
}

static const char *fourcc_name(uint32_t fourcc, char name[5])
{
	name[0] = fourcc & 0xff;
	name[1] = (fourcc >> 8) & 0xff;
	name[2] = (fourcc >> 16) & 0xff;
	name[3] = (fourcc >> 24) & 0xff;
	name[4] = '\0';
	return name;
}

static int read_text(const char *path, char *text, size_t size)
{
	FILE *file = fopen(path, "r");
	size_t length;

	if (!file)
		return -1;
	length = fread(text, 1, size - 1, file);
	fclose(file);
	text[length] = '\0';
	text[strcspn(text, "\n")] = '\0';
	return 0;
}

static int beside_executable(const char *name, char *path, size_t path_size)
{
	char executable[PATH_MAX];
	char *slash;
	ssize_t length = readlink("/proc/self/exe", executable, sizeof(executable) - 1);

	if (length < 0)
		return -1;
	executable[length] = '\0';
	slash = strrchr(executable, '/');
	if (!slash) {
		errno = ENOENT;
		return -1;
	}
	*slash = '\0';
	if (snprintf(path, path_size, "%s/%s", executable, name) >= (int)path_size) {
		errno = ENAMETOOLONG;
		return -1;
	}
	return 0;
}

/* Reads "srcversion=" from the .modinfo section of a .ko file. */
static int module_file_srcversion(const char *path, char *version, size_t size)
{
	static const char key[] = "srcversion=";
	FILE *file = fopen(path, "rb");
	size_t matched = 0, length = 0;
	int c;

	if (!file)
		return -1;
	while ((c = fgetc(file)) != EOF) {
		if (matched < sizeof(key) - 1) {
			matched = c == key[matched] ? matched + 1 : (c == key[0]);
			continue;
		}
		if (!c || length + 1 >= size)
			break;
		version[length++] = (char)c;
	}
	fclose(file);
	version[length] = '\0';
	return length ? 0 : -1;
}

static bool module_loaded(void)
{
	return access("/sys/module/hgvcam", F_OK) == 0;
}

static int run_command(char *const argv[])
{
	int status;
	pid_t pid = fork();

	if (pid < 0)
		return -1;
	if (!pid) {
		execvp(argv[0], argv);
		perror(argv[0]);
		_exit(127);
	}
	while (waitpid(pid, &status, 0) < 0)
		if (errno != EINTR)
			return -1;
	return WIFEXITED(status) && WEXITSTATUS(status) == 0 ? 0 : -1;
}

/*
 * The only privileged step: installs the udev rule (when it changed) and
 * loads, reloads or merely re-applies the rule to the control device.  The
 * player itself, and therefore FFmpeg, keeps running as the user.
 */
static int run_privileged(const char *rule, const char *module, const char *action,
			  const char *why)
{
	static const char script[] =
		"set -e\n"
		"rule=\"$1\" module=\"$2\" action=\"$3\"\n"
		"target=/etc/udev/rules.d/" RULE_FILE "\n"
		"if [ -n \"$rule\" ] && ! cmp -s \"$rule\" \"$target\"; then\n"
		"  install -m 0644 \"$rule\" \"$target\"\n"
		"  udevadm control --reload-rules\n"
		"fi\n"
		"case \"$action\" in\n"
		"reload)\n"
		"  if grep -q '^hgvcam ' /proc/modules; then /usr/sbin/rmmod hgvcam; fi\n"
		"  /usr/sbin/insmod \"$module\" ;;\n"
		"load)\n"
		/* Another player may have loaded it meanwhile ("File exists"). */
		"  grep -q '^hgvcam ' /proc/modules || /usr/sbin/insmod \"$module\" ||\n"
		"    grep -q '^hgvcam ' /proc/modules ;;\n"
		"*)\n"
		"  udevadm trigger --action=change --subsystem-match=misc --sysname-match=hgvcam ;;\n"
		"esac\n"
		"udevadm settle\n";
	char *argv[] = { NULL, "/bin/sh", "-c", (char *)script, "sh", (char *)rule,
			 (char *)module, (char *)action, NULL };

	if (geteuid() == 0)
		return run_command(argv + 1);
	argv[0] = isatty(STDIN_FILENO) ? "sudo" : "pkexec";
	fprintf(stderr, "%s: administrator password requested via %s\n", why, argv[0]);
	return run_command(argv);
}

static int prepare_module(const char *module_path, const char *rule)
{
	char loaded[64], wanted[64], refcnt[16];
	bool is_loaded = !read_text("/sys/module/hgvcam/srcversion", loaded, sizeof(loaded));

	if (is_loaded && (module_file_srcversion(module_path, wanted, sizeof(wanted)) ||
			  !strcmp(loaded, wanted)))
		return 0;
	if (is_loaded) {
		fprintf(stderr, "loaded hgvcam (%s) differs from %s (%s)\n",
			loaded, module_path, wanted);
		/* Something (another player, a camera client) holds it: rmmod would fail. */
		if (!read_text("/sys/module/hgvcam/refcnt", refcnt, sizeof(refcnt)) &&
		    atoi(refcnt) > 0) {
			fprintf(stderr, "it is in use, so the loaded module is kept\n");
			return 0;
		}
	}
	if (access(module_path, R_OK)) {
		fprintf(stderr, "cannot read '%s': %s\n", module_path, strerror(errno));
		return is_loaded ? 0 : -1;
	}
	if (run_privileged(rule, module_path, is_loaded ? "reload" : "load",
			   is_loaded ? "reloading hgvcam" : "loading hgvcam") == 0)
		return 0;
	fprintf(stderr, "cannot %s '%s'\n", is_loaded ? "reload" : "load", module_path);
	if (is_loaded) {
		fprintf(stderr, "close applications using the virtual cameras; "
			"continuing with the loaded module\n");
		return 0;
	}
	return -1;
}

/* Opens /dev/hgvcam, installing the udev rule if access is still denied. */
static int open_control_device(const char *rule, bool configure_allowed)
{
	bool rule_applied = false;
	unsigned int attempt;
	int fd, error = 0;

	/* udev needs a moment to create the node and grant access to it. */
	for (attempt = 0; attempt < 50; attempt++) {
		fd = open(HGVCAM_CONTROL_DEVICE, O_RDWR | O_CLOEXEC);
		if (fd >= 0)
			return fd;
		error = errno;
		if (error != ENOENT && error != EACCES)
			break;
		if (error == EACCES && attempt >= 10 && !rule_applied && configure_allowed &&
		    rule[0]) {
			rule_applied = true;
			if (run_privileged(rule, "", "rule", "granting access to "
					   HGVCAM_CONTROL_DEVICE) == 0)
				continue;
		}
		usleep(100000);
	}
	if (error == ENOENT && module_loaded())
		fprintf(stderr, "the loaded hgvcam is an older build without "
			HGVCAM_CONTROL_DEVICE "; close the applications using the "
			"virtual camera and run the player again to reload it\n");
	else if (error == ENOENT)
		fprintf(stderr, "hgvcam is not loaded\n");
	else if (error == EACCES)
		fprintf(stderr, "no permission to open " HGVCAM_CONTROL_DEVICE
			"; install " RULE_FILE " in /etc/udev/rules.d (see the README)\n");
	else
		fprintf(stderr, "%s: %s\n", HGVCAM_CONTROL_DEVICE, strerror(error));
	return -1;
}

/* Keeps the aspect ratio when the video exceeds the module's maximum size. */
static void fit_size(int width, int height, int *fit_width, int *fit_height)
{
	*fit_width = width;
	*fit_height = height;
	if (width > HGVCAM_MAX_WIDTH || height > HGVCAM_MAX_HEIGHT) {
		if ((int64_t)width * HGVCAM_MAX_HEIGHT > (int64_t)height * HGVCAM_MAX_WIDTH) {
			*fit_width = HGVCAM_MAX_WIDTH;
			*fit_height = (int)((int64_t)height * HGVCAM_MAX_WIDTH / width);
		} else {
			*fit_height = HGVCAM_MAX_HEIGHT;
			*fit_width = (int)((int64_t)width * HGVCAM_MAX_HEIGHT / height);
		}
	}
	*fit_width &= ~1;
	*fit_height &= ~1;
}

/* Video source: decodes frames in a loop and stamps them with a monotonic media time. */

static int source_open(struct source *src, const char *filename)
{
	const AVCodec *decoder;
	int ret;

	if ((ret = avformat_open_input(&src->input, filename, NULL, NULL)) < 0) {
		fprintf(stderr, "cannot open '%s': %s\n", filename, av_err2str(ret));
		return -1;
	}
	if ((ret = avformat_find_stream_info(src->input, NULL)) < 0) {
		fprintf(stderr, "cannot inspect '%s': %s\n", filename, av_err2str(ret));
		return -1;
	}
	src->stream = av_find_best_stream(src->input, AVMEDIA_TYPE_VIDEO, -1, -1, &decoder, 0);
	if (src->stream < 0) {
		fprintf(stderr, "no decodable video stream: %s\n", av_err2str(src->stream));
		return -1;
	}
	src->decoder = avcodec_alloc_context3(decoder);
	if (!src->decoder ||
	    avcodec_parameters_to_context(src->decoder,
					  src->input->streams[src->stream]->codecpar) < 0 ||
	    avcodec_open2(src->decoder, decoder, NULL) < 0) {
		fprintf(stderr, "cannot initialize the video decoder\n");
		return -1;
	}
	if (src->decoder->width <= 0 || src->decoder->height <= 0) {
		fprintf(stderr, "'%s' does not declare a frame size\n", filename);
		return -1;
	}
	src->packet = av_packet_alloc();
	if (!src->packet) {
		fprintf(stderr, "out of memory\n");
		return -1;
	}
	src->rate = av_guess_frame_rate(src->input, src->input->streams[src->stream], NULL);
	if (src->rate.num <= 0 || src->rate.den <= 0)
		src->rate = (AVRational){ 30, 1 };
	return 0;
}

/* The video's display size (pixel aspect applied), fitted to the module's maximum. */
static void source_native_size(const struct source *src, int *width, int *height)
{
	AVRational sar = src->decoder->sample_aspect_ratio;
	int64_t w = src->decoder->width, h = src->decoder->height;

	if (sar.num > 0 && sar.den > 0) {
		if (sar.num > sar.den)
			w = w * sar.num / sar.den;
		else
			h = h * sar.den / sar.num;
	}
	fit_size((int)w, (int)h, width, height);
}

static void source_close(struct source *src)
{
	av_packet_free(&src->packet);
	avcodec_free_context(&src->decoder);
	avformat_close_input(&src->input);
}

static double source_stamp(struct source *src, const AVFrame *frame)
{
	AVStream *stream = src->input->streams[src->stream];
	double nominal = av_q2d(av_inv_q(src->rate));
	double time_base = av_q2d(stream->time_base);
	int64_t start = stream->start_time != AV_NOPTS_VALUE ? stream->start_time : 0;
	int64_t ts = frame->best_effort_timestamp;
	double duration = frame->duration > 0 ? frame->duration * time_base : nominal;
	double time;

	if (ts == AV_NOPTS_VALUE)
		time = src->frames_in_loop ? src->last_time + nominal : src->loop_offset;
	else
		time = (ts - start) * time_base + src->loop_offset;
	if (src->have_time && time < src->last_time)
		time = src->last_time;
	src->have_time = true;
	src->last_time = time;
	if (time + duration > src->end_time)
		src->end_time = time + duration;
	src->frames_in_loop++;
	return time;
}

/* Decodes the next frame, rewinding at the end of the file; returns 0 or -1. */
static int source_next(struct source *src, AVFrame *frame, double *time)
{
	AVStream *stream = src->input->streams[src->stream];
	int ret;

	for (;;) {
		ret = avcodec_receive_frame(src->decoder, frame);
		if (ret >= 0) {
			*time = source_stamp(src, frame);
			return 0;
		}
		if (ret == AVERROR_EOF) {
			if (!src->frames_in_loop) {
				fprintf(stderr, "the file has no decodable video frames\n");
				return -1;
			}
			if (av_seek_frame(src->input, src->stream,
					  stream->start_time != AV_NOPTS_VALUE ? stream->start_time : 0,
					  AVSEEK_FLAG_BACKWARD) < 0) {
				fprintf(stderr, "cannot seek to the beginning of the file\n");
				return -1;
			}
			avcodec_flush_buffers(src->decoder);
			src->loop_offset = src->end_time;
			src->frames_in_loop = 0;
			src->draining = false;
			src->loop_count++;
			fprintf(stderr, "loop %lld\n", (long long)src->loop_count);
			continue;
		}
		if (ret != AVERROR(EAGAIN) || src->draining) {
			fprintf(stderr, "decoder failed: %s\n", av_err2str(ret));
			return -1;
		}
		ret = av_read_frame(src->input, src->packet);
		if (ret == AVERROR_EOF) {
			/* Flush the frames still buffered in the decoder before looping. */
			avcodec_send_packet(src->decoder, NULL);
			src->draining = true;
			continue;
		}
		if (ret < 0) {
			fprintf(stderr, "cannot read input: %s\n", av_err2str(ret));
			return -1;
		}
		if (src->packet->stream_index == src->stream) {
			ret = avcodec_send_packet(src->decoder, src->packet);
			if (ret < 0 && ret != AVERROR(EAGAIN)) {
				av_packet_unref(src->packet);
				fprintf(stderr, "decoder rejected packet: %s\n", av_err2str(ret));
				return -1;
			}
		}
		av_packet_unref(src->packet);
	}
}

/* Producer: negotiation with the module, controls and frame rendering. */

static void update_tables(struct producer *out)
{
	const struct settings *s = &out->settings;
	int offset = s->brightness * 110 / HGVCAM_BRIGHTNESS_MAX;
	int value;

	for (value = 0; value < 256; value++) {
		int y = (value - 128) * s->contrast / 100 + 128 + offset;
		int c = (value - 128) * s->saturation / 100 + 128;

		out->luma[value] = y < 16 ? 16 : y > 235 ? 235 : y;
		out->chroma[value] = c < 16 ? 16 : c > 240 ? 240 : c;
	}
	out->identity = !s->brightness && s->contrast == 100 && s->saturation == 100;
}

/* The video waits whenever it is paused or nobody is capturing. */
static void update_clock(struct producer *out)
{
	clock_set_paused(&out->clock, out->settings.paused || !out->streaming);
}

static double camera_interval(const struct hgvcam_camera *camera)
{
	if (!camera->interval_numerator || !camera->interval_denominator)
		return 1.0 / 30;
	return (double)camera->interval_numerator / camera->interval_denominator;
}

static void producer_release_format(struct producer *out)
{
	free(out->canvas);
	out->canvas = NULL;
	free(out->payload);
	out->payload = NULL;
	sws_freeContext(out->convert);
	out->convert = NULL;
	av_frame_free(&out->converted);
	avcodec_free_context(&out->encoder);
}

/* Allocates the frame that converts the canvas into an RGB24 or YUVJ422P picture. */
static int producer_alloc_conversion(struct producer *out, enum AVPixelFormat format)
{
	int width = out->pix.width, height = out->pix.height;

	out->converted = av_frame_alloc();
	if (!out->converted)
		return -1;
	out->converted->format = format;
	out->converted->width = width;
	out->converted->height = height;
	if (av_frame_get_buffer(out->converted, 32) < 0)
		return -1;
	out->convert = sws_getContext(width, height, AV_PIX_FMT_YUYV422, width, height, format,
				      SWS_POINT, NULL, NULL, NULL);
	return out->convert ? 0 : -1;
}

static int producer_open_encoder(struct producer *out)
{
	const AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_MJPEG);

	if (!codec || !(out->encoder = avcodec_alloc_context3(codec)))
		return -1;
	out->encoder->width = out->pix.width;
	out->encoder->height = out->pix.height;
	out->encoder->pix_fmt = AV_PIX_FMT_YUVJ422P;
	out->encoder->color_range = AVCOL_RANGE_JPEG;
	out->encoder->time_base = (AVRational){ 1, 30 };
	out->encoder->flags |= AV_CODEC_FLAG_QSCALE;
	out->encoder->global_quality = FF_QP2LAMBDA * JPEG_QSCALE;
	return avcodec_open2(out->encoder, codec, NULL) < 0 ? -1 : 0;
}

/* Applies the camera's format: the producer node is set to exactly that format. */
static int producer_configure(struct producer *out)
{
	struct v4l2_format format = {
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT,
	};
	uint32_t pixelformat = out->camera.pixelformat;
	char name[5];
	int ret;

	if (pixelformat != V4L2_PIX_FMT_YUYV && pixelformat != V4L2_PIX_FMT_MJPEG &&
	    pixelformat != V4L2_PIX_FMT_SBGGR8 && pixelformat != V4L2_PIX_FMT_SGBRG8) {
		fprintf(stderr, "the camera asks for an unsupported pixel format (%s)\n",
			fourcc_name(pixelformat, name));
		return -1;
	}
	/* The write() queue keeps the old buffer size until its file is closed. */
	if (out->data_fd >= 0) {
		close(out->data_fd);
		out->data_fd = -1;
	}
	format.fmt.pix.width = out->camera.width;
	format.fmt.pix.height = out->camera.height;
	format.fmt.pix.pixelformat = pixelformat;
	format.fmt.pix.field = V4L2_FIELD_NONE;
	if (ioctl(out->control_fd, VIDIOC_S_FMT, &format) < 0) {
		perror("VIDIOC_S_FMT");
		return -1;
	}
	if (format.fmt.pix.width != out->camera.width ||
	    format.fmt.pix.height != out->camera.height ||
	    format.fmt.pix.pixelformat != pixelformat) {
		fprintf(stderr, "producer node did not accept %ux%u %s\n", out->camera.width,
			out->camera.height, fourcc_name(pixelformat, name));
		return -1;
	}
	producer_release_format(out);
	out->pix = format.fmt.pix;
	out->canvas = malloc((size_t)out->pix.width * out->pix.height * 2);
	if (!out->canvas) {
		fprintf(stderr, "out of memory\n");
		return -1;
	}
	out->layout[2] = 0;	/* forces the black borders to be redrawn */
	switch (pixelformat) {
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
		out->payload = malloc((size_t)out->pix.width * out->pix.height);
		ret = !out->payload || producer_alloc_conversion(out, AV_PIX_FMT_RGB24);
		break;
	case V4L2_PIX_FMT_MJPEG:
		ret = producer_alloc_conversion(out, AV_PIX_FMT_YUVJ422P) ||
		      producer_open_encoder(out);
		break;
	default:
		ret = 0;
		break;
	}
	if (ret) {
		fprintf(stderr, "cannot prepare %s conversion\n", fourcc_name(pixelformat, name));
		return -1;
	}
	out->data_fd = open(out->device, O_WRONLY | O_CLOEXEC);
	if (out->data_fd < 0) {
		perror(out->device);
		return -1;
	}
	fprintf(stderr, "camera: %ux%u %s at %.3f fps\n", out->pix.width, out->pix.height,
		fourcc_name(pixelformat, name), 1.0 / camera_interval(&out->camera));
	return 0;
}

static void apply_streaming(struct producer *out, bool streaming)
{
	if (streaming == out->streaming)
		return;
	out->streaming = streaming;
	fprintf(stderr, streaming ? "capture started\n" :
		"no application is capturing: video paused\n");
	update_clock(out);
}

static int apply_camera(struct producer *out, const struct hgvcam_camera *camera)
{
	bool reformatted = camera->width != out->camera.width ||
			   camera->height != out->camera.height ||
			   camera->pixelformat != out->camera.pixelformat;
	bool retimed = camera->interval_numerator != out->camera.interval_numerator ||
		       camera->interval_denominator != out->camera.interval_denominator;

	out->camera = *camera;
	if (reformatted && producer_configure(out))
		return -1;
	if (retimed && !reformatted)
		fprintf(stderr, "camera: %.3f fps\n", 1.0 / camera_interval(camera));
	apply_streaming(out, camera->flags & HGVCAM_CAMERA_STREAMING);
	return 0;
}

static void apply_control(struct producer *out, uint32_t id, int32_t value)
{
	struct settings *s = &out->settings;

	switch (id) {
	case HGVCAM_CID_PAUSE:
		s->paused = value;
		update_clock(out);
		break;
	case HGVCAM_CID_BLACK: s->black = value; break;
	case V4L2_CID_BRIGHTNESS: s->brightness = value; break;
	case V4L2_CID_CONTRAST: s->contrast = value; break;
	case V4L2_CID_SATURATION: s->saturation = value; break;
	case V4L2_CID_HFLIP: s->hflip = value; break;
	case V4L2_CID_VFLIP: s->vflip = value; break;
	case V4L2_CID_ZOOM_ABSOLUTE:
		s->zoom = value < HGVCAM_ZOOM_MIN ? HGVCAM_ZOOM_MIN : value;
		break;
	case V4L2_CID_PAN_ABSOLUTE: s->pan = value; break;
	case V4L2_CID_TILT_ABSOLUTE: s->tilt = value; break;
	}
	update_tables(out);
}

/* Dequeues every pending event; returns 0, or -1 on a fatal error. */
static int handle_events(struct producer *out)
{
	struct v4l2_event event;

	for (;;) {
		if (ioctl(out->control_fd, VIDIOC_DQEVENT, &event) < 0) {
			if (errno == EINTR)
				continue;
			if (errno == ENOENT || errno == EAGAIN)
				return 0;
			perror("VIDIOC_DQEVENT");
			return -1;
		}
		if (event.type == HGVCAM_EVENT_CAMERA) {
			struct hgvcam_camera camera;

			memcpy(&camera, event.u.data, sizeof(camera));
			if (apply_camera(out, &camera))
				return -1;
		} else if (event.type == V4L2_EVENT_CTRL &&
			   (event.u.ctrl.changes & V4L2_EVENT_CTRL_CH_VALUE)) {
			apply_control(out, event.id, event.u.ctrl.value);
		}
	}
}

/* Opens the producer node of the camera just created and reads its state. */
static int producer_open(struct producer *out)
{
	struct v4l2_event_subscription subscription = { .type = HGVCAM_EVENT_CAMERA };
	struct v4l2_capability cap = { 0 };
	unsigned int attempt, i;

	/* udev needs a moment to create the node and grant access to it. */
	for (attempt = 0; attempt < 50; attempt++) {
		out->control_fd = open(out->device, O_RDWR | O_NONBLOCK | O_CLOEXEC);
		if (out->control_fd >= 0 || (errno != ENOENT && errno != EACCES))
			break;
		usleep(100000);
	}
	if (out->control_fd < 0) {
		perror(out->device);
		return -1;
	}
	if (ioctl(out->control_fd, VIDIOC_QUERYCAP, &cap) < 0 ||
	    strcmp((const char *)cap.driver, "hgvcam") ||
	    !(cap.device_caps & V4L2_CAP_VIDEO_OUTPUT)) {
		fprintf(stderr, "%s is not an hgvcam producer node\n", out->device);
		return -1;
	}
	/* Subscribe before reading the camera state, so no change is missed. */
	if (ioctl(out->control_fd, VIDIOC_SUBSCRIBE_EVENT, &subscription) < 0) {
		perror("subscribing to camera events");
		return -1;
	}
	for (i = 0; i < sizeof(subscribed_controls) / sizeof(subscribed_controls[0]); i++) {
		subscription = (struct v4l2_event_subscription){
			.type = V4L2_EVENT_CTRL,
			.id = subscribed_controls[i],
			.flags = V4L2_EVENT_SUB_FL_SEND_INITIAL,
		};
		if (ioctl(out->control_fd, VIDIOC_SUBSCRIBE_EVENT, &subscription) < 0)
			fprintf(stderr, "control 0x%08x is not available\n", subscribed_controls[i]);
	}
	if (ioctl(out->control_fd, HGVCAM_G_CAMERA, &out->camera) < 0) {
		perror("reading the camera format");
		return -1;
	}
	out->streaming = out->camera.flags & HGVCAM_CAMERA_STREAMING;
	return producer_configure(out);
}

static void producer_close(struct producer *out)
{
	producer_release_format(out);
	av_packet_free(&out->jpeg);
	av_frame_free(&out->cropped);
	av_frame_free(&out->scaled);
	sws_freeContext(out->sws);
	if (out->data_fd >= 0)
		close(out->data_fd);
	if (out->control_fd >= 0)
		close(out->control_fd);
}

static int write_all(int fd, const uint8_t *data, size_t length)
{
	while (length) {
		ssize_t written = write(fd, data, length);

		if (written < 0) {
			if (errno == EINTR)
				continue;
			perror("writing frame to V4L2 producer");
			return -1;
		}
		if (!written) {
			fprintf(stderr, "V4L2 producer accepted a zero-byte write\n");
			return -1;
		}
		data += written;
		length -= written;
	}
	return 0;
}

static void fill_black(uint8_t *data, size_t length)
{
	size_t i;

	for (i = 0; i + 3 < length; i += 4) {
		data[i] = 16;
		data[i + 1] = 128;
		data[i + 2] = 16;
		data[i + 3] = 128;
	}
}

/* Samples one colour per pixel from an RGB24 picture: B G / G R or G B / R G. */
static void mosaic(const AVFrame *rgb, uint8_t *dst, int width, int height, bool bggr)
{
	/* channel[row parity][column parity]; RGB24 offsets: 0 = R, 1 = G, 2 = B */
	static const int bggr_channel[2][2] = { { 2, 1 }, { 1, 0 } };
	static const int gbrg_channel[2][2] = { { 1, 2 }, { 0, 1 } };
	const int (*channel)[2] = bggr ? bggr_channel : gbrg_channel;
	int x, y;

	for (y = 0; y < height; y++) {
		const uint8_t *row = rgb->data[0] + (size_t)y * rgb->linesize[0];
		const int *pattern = channel[y & 1];
		uint8_t *line = dst + (size_t)y * width;

		for (x = 0; x < width; x++)
			line[x] = row[3 * x + pattern[x & 1]];
	}
}

/* Converts the YUYV canvas into the camera's format and writes it out. */
static int publish(struct producer *out)
{
	const uint8_t *canvas[4] = { out->canvas };
	const int stride[4] = { (int)out->pix.width * 2 };
	int width = out->pix.width, height = out->pix.height, ret;

	if (out->pix.pixelformat == V4L2_PIX_FMT_YUYV)
		return write_all(out->data_fd, out->canvas, out->pix.sizeimage);

	if (av_frame_make_writable(out->converted) < 0) {
		fprintf(stderr, "conversion frame is not writable\n");
		return -1;
	}
	sws_scale(out->convert, canvas, stride, 0, height, out->converted->data,
		  out->converted->linesize);
	if (out->pix.pixelformat != V4L2_PIX_FMT_MJPEG) {
		mosaic(out->converted, out->payload, width, height,
		       out->pix.pixelformat == V4L2_PIX_FMT_SBGGR8);
		return write_all(out->data_fd, out->payload, (size_t)width * height);
	}

	out->converted->pts = out->frame_count++;
	out->converted->quality = out->encoder->global_quality;
	if ((ret = avcodec_send_frame(out->encoder, out->converted)) < 0 ||
	    (ret = avcodec_receive_packet(out->encoder, out->jpeg)) < 0) {
		fprintf(stderr, "JPEG encoding failed: %s\n", av_err2str(ret));
		return -1;
	}
	if ((size_t)out->jpeg->size > out->pix.sizeimage) {
		/* Never happens at this quality; dropping beats overflowing the buffer. */
		if (!out->warned_jpeg_size)
			fprintf(stderr, "JPEG frame larger than the camera buffer: dropped\n");
		out->warned_jpeg_size = true;
		ret = 0;
	} else {
		ret = write_all(out->data_fd, out->jpeg->data, out->jpeg->size);
	}
	av_packet_unref(out->jpeg);
	return ret;
}

/* Copies one YUYV row, mirroring it and applying the colour tables when needed. */
static void copy_row(const struct producer *out, uint8_t *dst, const uint8_t *src, int pairs)
{
	bool hflip = out->settings.hflip;
	int i;

	if (!hflip && out->identity) {
		memcpy(dst, src, (size_t)pairs * 4);
		return;
	}
	for (i = 0; i < pairs; i++) {
		const uint8_t *p = src + 4 * i;
		uint8_t *q = dst + 4 * (hflip ? pairs - 1 - i : i);

		q[0] = out->luma[hflip ? p[2] : p[0]];
		q[1] = out->chroma[p[1]];
		q[2] = out->luma[hflip ? p[0] : p[2]];
		q[3] = out->chroma[p[3]];
	}
}

/*
 * Crops the zoom/pan/tilt window out of the video frame, scales it into the
 * canvas keeping its aspect ratio (black bars fill the rest) and publishes
 * the result in the camera's format.
 */
static int render(struct producer *out, const AVFrame *frame)
{
	const struct settings *s = &out->settings;
	const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(frame->format);
	int camera_w = out->pix.width, camera_h = out->pix.height;
	size_t canvas_stride = (size_t)camera_w * 2;
	int align_x, align_y, crop_w, crop_h, crop_x, crop_y;
	int pic_w, pic_h, pic_x, pic_y, row;
	AVRational sar = frame->sample_aspect_ratio;
	int64_t display_w, display_h;

	if (s->black) {
		fill_black(out->canvas, canvas_stride * camera_h);
		out->layout[2] = 0;
		return publish(out);
	}
	if (!desc) {
		fprintf(stderr, "unknown decoded pixel format\n");
		return -1;
	}
	align_x = 1 << desc->log2_chroma_w;
	align_y = 1 << desc->log2_chroma_h;
	crop_w = (int)((int64_t)frame->width * HGVCAM_ZOOM_MIN / s->zoom) & ~(align_x - 1);
	crop_h = (int)((int64_t)frame->height * HGVCAM_ZOOM_MIN / s->zoom) & ~(align_y - 1);
	if (crop_w < 2 * align_x)
		crop_w = frame->width < 2 * align_x ? frame->width : 2 * align_x;
	if (crop_h < 2 * align_y)
		crop_h = frame->height < 2 * align_y ? frame->height : 2 * align_y;
	/* Pan moves the window right, tilt moves it up; both stay inside the video. */
	crop_x = (int)((int64_t)(frame->width - crop_w) * (s->pan + HGVCAM_PAN_MAX) /
		       (2 * HGVCAM_PAN_MAX)) & ~(align_x - 1);
	crop_y = (int)((int64_t)(frame->height - crop_h) * (HGVCAM_PAN_MAX - s->tilt) /
		       (2 * HGVCAM_PAN_MAX)) & ~(align_y - 1);

	if (sar.num <= 0 || sar.den <= 0)
		sar = (AVRational){ 1, 1 };
	display_w = (int64_t)crop_w * sar.num;
	display_h = (int64_t)crop_h * sar.den;
	if ((int64_t)camera_w * display_h > (int64_t)camera_h * display_w) {
		pic_h = camera_h;
		pic_w = (int)(camera_h * display_w / display_h);
	} else {
		pic_w = camera_w;
		pic_h = (int)(camera_w * display_h / display_w);
	}
	pic_w &= ~1;
	pic_w = pic_w < 2 ? 2 : pic_w > camera_w ? camera_w : pic_w;
	pic_h = pic_h < 2 ? 2 : pic_h > camera_h ? camera_h : pic_h;
	pic_x = ((camera_w - pic_w) / 2) & ~1;
	pic_y = (camera_h - pic_h) / 2;

	if (out->layout[0] != pic_x || out->layout[1] != pic_y ||
	    out->layout[2] != pic_w || out->layout[3] != pic_h) {
		av_frame_unref(out->scaled);
		out->scaled->format = AV_PIX_FMT_YUYV422;
		out->scaled->width = pic_w;
		out->scaled->height = pic_h;
		if (av_frame_get_buffer(out->scaled, 32) < 0) {
			fprintf(stderr, "cannot allocate the scaling frame\n");
			return -1;
		}
		fill_black(out->canvas, canvas_stride * camera_h);
		out->layout[0] = pic_x;
		out->layout[1] = pic_y;
		out->layout[2] = pic_w;
		out->layout[3] = pic_h;
	}

	if (av_frame_ref(out->cropped, frame) < 0)
		return -1;
	out->cropped->crop_left = crop_x;
	out->cropped->crop_top = crop_y;
	out->cropped->crop_right = frame->width - crop_x - crop_w;
	out->cropped->crop_bottom = frame->height - crop_y - crop_h;
	if (av_frame_apply_cropping(out->cropped, AV_FRAME_CROP_UNALIGNED) < 0) {
		av_frame_unref(out->cropped);
		fprintf(stderr, "cannot crop %s frames\n", desc->name);
		return -1;
	}
	/* Cached per frame so mid-stream size or pixel-format changes are handled. */
	out->sws = sws_getCachedContext(out->sws, out->cropped->width, out->cropped->height,
					frame->format, pic_w, pic_h, AV_PIX_FMT_YUYV422,
					SWS_BICUBIC, NULL, NULL, NULL);
	if (!out->sws) {
		av_frame_unref(out->cropped);
		fprintf(stderr, "cannot convert %dx%d %s frames\n", frame->width,
			frame->height, desc->name);
		return -1;
	}
	sws_scale(out->sws, (const uint8_t * const *)out->cropped->data, out->cropped->linesize,
		  0, out->cropped->height, out->scaled->data, out->scaled->linesize);
	av_frame_unref(out->cropped);

	for (row = 0; row < pic_h; row++) {
		int dst_row = pic_y + (s->vflip ? pic_h - 1 - row : row);

		copy_row(out, out->canvas + (size_t)dst_row * canvas_stride + pic_x * 2,
			 out->scaled->data[0] + (size_t)row * out->scaled->linesize[0], pic_w / 2);
	}
	return publish(out);
}

/*
 * Sleeps until the deadline, handling events (format, rate, capture state,
 * controls) meanwhile.  A negative deadline waits for the next events only.
 */
static int wait_until(struct producer *out, double deadline)
{
	bool forever = deadline < 0;

	for (;;) {
		struct pollfd pfd = { .fd = out->control_fd, .events = POLLPRI };
		struct timespec timeout;
		int ret;

		if (!keep_running)
			return 0;
		if (!forever) {
			double remaining = deadline - now_seconds();

			if (remaining <= 0)
				return 0;
			timeout.tv_sec = (time_t)remaining;
			timeout.tv_nsec = (long)((remaining - timeout.tv_sec) * 1e9);
		}
		ret = ppoll(&pfd, 1, forever ? NULL : &timeout, NULL);
		if (ret < 0) {
			if (errno == EINTR)
				continue;
			perror("waiting for events");
			return -1;
		}
		if (!ret)
			continue;
		if (!(pfd.revents & POLLPRI)) {
			/* poll reports an error state: fall back to short sleeps. */
			struct timespec nap = { 0, 5000000 };

			nanosleep(&nap, NULL);
		}
		if (handle_events(out))
			return -1;
		if (forever)
			return 0;
	}
}

int main(int argc, char **argv)
{
	const char *filename = NULL, *module_path = NULL;
	struct source src = { 0 };
	struct producer out = {
		.control_fd = -1,
		.data_fd = -1,
		.settings = {
			.contrast = 100,
			.saturation = 100,
			.zoom = HGVCAM_ZOOM_MIN,
		},
	};
	struct hgvcam_add add = { 0 };
	AVFrame *current = NULL, *next = NULL;
	double current_time, next_time, deadline;
	int option, control = -1, native_w, native_h, status = EXIT_FAILURE;
	bool load_allowed = true, camera_created = false;
	char module_path_buffer[PATH_MAX], rule[PATH_MAX];
	static const struct option options[] = {
		{ "file", required_argument, NULL, 'f' },
		{ "load-module", no_argument, NULL, 'l' },
		{ "module", required_argument, NULL, 'm' },
		{ "no-load", no_argument, NULL, 'n' },
		{ "help", no_argument, NULL, 'h' },
		{ NULL, 0, NULL, 0 },
	};

	while ((option = getopt_long(argc, argv, "f:lm:nh", options, NULL)) != -1) {
		switch (option) {
		case 'f': filename = optarg; break;
		case 'l': break; /* kept for compatibility: loading is automatic now */
		case 'm': module_path = optarg; break;
		case 'n': load_allowed = false; break;
		default: usage(argv[0]); return option == 'h' ? EXIT_SUCCESS : EXIT_FAILURE;
		}
	}
	if (!filename) {
		usage(argv[0]);
		return EXIT_FAILURE;
	}
	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	update_tables(&out);

	/* The video is inspected first: the camera is created at its native size. */
	if (source_open(&src, filename))
		goto out;
	if (beside_executable(RULE_FILE, rule, sizeof(rule)) || access(rule, R_OK)) {
		fprintf(stderr, "note: %s not found beside the player; it will not be installed\n",
			RULE_FILE);
		rule[0] = '\0';
	}
	if (load_allowed) {
		if (!module_path) {
			if (beside_executable("hgvcam.ko", module_path_buffer,
					      sizeof(module_path_buffer))) {
				perror("locating hgvcam.ko");
				goto out;
			}
			module_path = module_path_buffer;
		}
		if (prepare_module(module_path, rule))
			goto out;
	}
	control = open_control_device(rule, load_allowed);
	if (control < 0)
		goto out;

	source_native_size(&src, &native_w, &native_h);
	add.width = native_w;
	add.height = native_h;
	add.interval_numerator = src.rate.den;
	add.interval_denominator = src.rate.num;
	if (ioctl(control, HGVCAM_ADD, &add) < 0) {
		if (errno == ENOSPC)
			fprintf(stderr, "every virtual camera is in use; close a player first\n");
		else
			perror("creating the virtual camera");
		goto out;
	}
	camera_created = true;
	snprintf(out.device, sizeof(out.device), "/dev/video%u", add.output_nr);
	fprintf(stderr, "virtual camera %u: /dev/video%u (producer %s)\n", add.number,
		add.capture_nr, out.device);

	out.cropped = av_frame_alloc();
	out.scaled = av_frame_alloc();
	out.jpeg = av_packet_alloc();
	current = av_frame_alloc();
	next = av_frame_alloc();
	if (!out.cropped || !out.scaled || !out.jpeg || !current || !next) {
		fprintf(stderr, "out of memory\n");
		goto out;
	}
	if (producer_open(&out) || handle_events(&out))
		goto out;
	if (source_next(&src, current, &current_time) || source_next(&src, next, &next_time))
		goto out;
	fprintf(stderr, "playing %s (%dx%d, %d/%d fps); Ctrl-C stops\n", filename,
		current->width, current->height, src.rate.num, src.rate.den);
	if (!out.streaming)
		fprintf(stderr, "waiting for an application to capture\n");

	clock_start(&out.clock, current_time);
	update_clock(&out);
	deadline = now_seconds();
	while (keep_running) {
		double media;
		int caught_up = 0;

		/* Nobody is capturing: no decoding or conversion until that changes. */
		if (!out.streaming) {
			if (wait_until(&out, -1))
				goto out;
			deadline = now_seconds();
			continue;
		}
		media = clock_time(&out.clock);
		/* Show the latest frame that is due; frames that came too late are skipped. */
		while (next_time <= media) {
			if (++caught_up > MAX_CATCH_UP) {
				/* Decoding cannot keep up: let the video slip instead of stalling. */
				clock_start(&out.clock, next_time);
				break;
			}
			av_frame_unref(current);
			av_frame_move_ref(current, next);
			if (source_next(&src, next, &next_time))
				goto out;
		}
		if (render(&out, current))
			goto out;
		deadline += camera_interval(&out.camera);
		if (deadline < now_seconds() - 0.25)
			deadline = now_seconds();
		if (wait_until(&out, deadline))
			goto out;
	}
	status = EXIT_SUCCESS;
out:
	av_frame_free(&current);
	av_frame_free(&next);
	producer_close(&out);
	source_close(&src);
	if (control >= 0) {
		/* Closing the control device would remove it too; this makes it explicit. */
		if (camera_created && ioctl(control, HGVCAM_REMOVE, &add.number) < 0)
			perror("removing the virtual camera");
		close(control);
	}
	return status;
}
