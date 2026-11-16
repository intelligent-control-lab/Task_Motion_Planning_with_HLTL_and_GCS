# Task video gallery

Open `index.html` in a browser. This static page uses local files and needs no
build, account, or hosting service. Keep `video-gallery/` and `media/task_videos/`
in their current relative locations. A GitHub source view will not execute the
page; download the checkout and open it locally.

To serve it locally instead, run this from the repository root:

```bash
python3 -m http.server 8766 --bind 127.0.0.1
```

Then visit <http://127.0.0.1:8766/video-gallery/>. Stop the server with Ctrl-C.
Nothing is published by these steps.

## Contents and provenance

- Ten clips from the authors’ [paper video](https://www.youtube.com/watch?v=FPTLGm5iigc):
  nine simulations and the WX200 hardware experiment. Source captions, audio,
  framing, and 1× / 3× labels are preserved. Clips are encoded at 30 fps and
  1080 pixels high, with H.264/AAC and MP4 fast-start metadata.
- Animated GIF previews provide playback in README viewers that do not support
  embedded video. The full MP4s play inline in this gallery.

The [manifest](../media/task_videos/manifest.json) records exclusive-end frame
intervals at the source's 60 fps, clip hashes, and durations.
The source video has 5,948 video frames; its audio/container runs
slightly longer. The opening two-second title is omitted. Related code links
identify task families; filmed target numbers can differ from current code.
The source file remains in Downloads and is not included in the checkout.

## Regeneration

Using Python 3.11 or newer, create an environment and install the encoder:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-video.txt
.venv/bin/python scripts/split_paper_video.py --source /path/to/paper-video.mp4
```

The final command extracts the ten paper clips and regenerates their posters,
GIF previews, and metadata. Use `--only two_handover` to regenerate one clip.
`clips.js` is generated from the manifest so the gallery
also works via `file://` without fetching JSON.
