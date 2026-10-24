# Hitgalut Virtual Camera

`hgvcam` is a self-contained V4L2 kernel module. It deliberately does not
decode, scale or encode video in kernel space.

Loading the module creates only the control device, `/dev/hgvcam`. Each
player creates its own virtual camera there, so several players (even with
the same file) give several independent cameras. A camera is a pair of
nodes:

- **camera** ("Hitgalut Virtual Camera N"): `V4L2_CAP_VIDEO_CAPTURE`, for
  browser/OBS/Multiview/conferencing applications;
- **producer** ("Hitgalut Virtual Camera Producer N"): `V4L2_CAP_VIDEO_OUTPUT`,
  where that player publishes frames.

The camera disappears when its player exits, however it exits. At most 15
cameras exist at a time (`HGVCAM_MAX_CAMERAS` in `hgvcam.c`).

Each camera behaves like a webcam. It offers YUYV, MJPEG, BGGR (`BA81`) and
GBRG (`GBRG`) at the video's native size (fitted to 1920×1080) and at 320×240,
640×360, 640×480, 1280×720 and 1920×1080, at 60, 30, 25, 24 and 15 fps plus
the video's own rate. It starts at the native size and rate. The application
chooses; the module reports the choice to the player (`hgvcam.h`), which
then publishes frames of exactly that format, size and rate. Frames whose
format does not match the camera's are dropped by the module, never
delivered.

Each camera serves a single application at a time. While a player is
publishing, other programs that open its producer node may only query it:
`VIDIOC_S_FMT` and `VIDIOC_S_PARM` fail with `EBUSY`.

## Build

```bash
make module
make player
```

`hgvcam.h` is shared by the module and the player; rebuild both after
changing it.

## Run

```bash
./player --file /path/to/movie.mp4
```

The player:

1. loads `hgvcam.ko` (located beside `player`; override with
   `--module FILE`) when it is not loaded, and installs the udev rule
   `70-hgvcam.rules` (also beside `player`) in `/etc/udev/rules.d`. The
   administrator password is asked through `sudo` (or `pkexec` when there is
   no terminal). Only that step runs as root; decoding runs as the user;
2. reloads the module when the loaded build differs from that file, unless
   something is using it (`/sys/module/hgvcam/refcnt` > 0), in which case
   the loaded module is kept;
3. creates its camera and prints its nodes, for example
   `virtual camera 1: /dev/video2 (producer /dev/video3)`;
4. does nothing while no application is capturing: the video waits where it
   stopped and continues from there when a capture starts;
5. loops the file forever, showing at each camera frame the video frame whose
   timestamp is due, so the video neither speeds up nor slows down whatever
   frame rate the camera uses.

Options: `--no-load` never loads, reloads or configures the module;
`--load-module` is accepted for compatibility and does nothing.

The udev rule gives the logged-in user (and the `video` group) access to
`/dev/hgvcam`, so after the first run no password is needed while the
module stays loaded. To install it by hand:

```bash
sudo install -m 0644 70-hgvcam.rules /etc/udev/rules.d/
```

## Controls

The camera node exposes standard webcam controls. The module only stores
them; the player applies them to the picture, whatever the format:

| Control | Range | Effect |
|---|---|---|
| Brightness | −100 … 100 (0) | luma offset |
| Contrast | 0 … 200 (100) | percent |
| Saturation | 0 … 200 (100) | percent |
| Horizontal Flip / Vertical Flip | 0/1 | mirror |
| Zoom, Absolute | 100 … 1000 (100) | digital zoom, 1× … 10× |
| Pan, Absolute / Tilt, Absolute | −100 … 100 (0) | moves the zoom window right / up; no effect at 1× |
| Pause virtual camera | 0/1 | freezes the picture (last frame repeated); resumes where it stopped |
| Black virtual camera | 0/1 | black picture |

When the camera's aspect ratio differs from the video's, the picture keeps the
video's proportions and black bars fill the rest.

## Authors

- Josemar Simao <josemarsimao@gmail.com>
- Co-authored with Claude Opus 5.5 (Anthropic) <noreply@anthropic.com>

The kernel module is licensed under GPL-2.0-only and the player under
GPL-2.0-or-later (see the SPDX line at the top of each source file).
