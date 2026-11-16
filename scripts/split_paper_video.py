#!/usr/bin/env python3
"""Extract task clips, posters and animated README previews from the paper video.

Install the optional encoder with: uv pip install --python .venv/bin/python imageio-ffmpeg
Cut intervals use source frame indices; no downloaded source media is modified.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import subprocess
import imageio_ffmpeg

ROOT = Path(__file__).resolve().parents[1]
MEDIA = ROOT / "media" / "task_videos"


def sha256(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path)
    parser.add_argument("--only", nargs="*")
    parser.add_argument("--jobs", type=int, default=2)
    args = parser.parse_args()
    manifest_path = MEDIA / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    source = args.source or Path.home() / "Downloads" / manifest["source"]["filename"]
    if not source.is_file():
        parser.error("Source video is missing; pass --source /absolute/path/to/video.mp4")
    if args.jobs < 1:
        parser.error("jobs must be positive")
    known = {clip["id"] for clip in manifest["clips"]}
    if args.only and not set(args.only) <= known:
        parser.error("Unknown clip id")
    fps = manifest["source"]["fps"]
    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    manifest["source"]["sha256"] = sha256(source)

    def run(command):
        result = subprocess.run([ffmpeg, "-hide_banner", "-loglevel", "error", "-y", *command],
                                capture_output=True, text=True)
        if result.returncode:
            raise RuntimeError(result.stderr)

    def extract(clip):
        if clip.get("origin") == "local":
            return clip
        if args.only and clip["id"] not in args.only:
            return clip
        start = clip["start_frame"] / fps
        duration = (clip["end_frame"] - clip["start_frame"]) / fps
        video = MEDIA / f"{clip['id']}.mp4"
        poster = MEDIA / f"{clip['id']}.jpg"
        preview = MEDIA / f"{clip['id']}.gif"
        run(["-ss", f"{start:.9f}", "-i", str(source), "-t", f"{duration:.9f}",
             "-map", "0:v:0", "-map", "0:a:0?", "-vf", "scale=-2:1080,fps=30",
             "-c:v", "libx264", "-preset", "fast", "-crf", "20", "-pix_fmt", "yuv420p",
             "-threads", "2", "-c:a", "aac", "-b:a", "128k", "-movflags", "+faststart", str(video)])
        run(["-ss", f"{duration*.35:.6f}", "-i", str(video), "-frames:v", "1",
             "-vf", "scale=720:-2", "-q:v", "3", str(poster)])
        run(["-i", str(video), "-filter_complex",
             "fps=6,scale=480:-2:flags=lanczos,split[a][b];[a]palettegen=max_colors=128[p];[b][p]paletteuse=dither=bayer:bayer_scale=3",
             "-an", "-loop", "0", str(preview)])
        count, seconds = imageio_ffmpeg.count_frames_and_secs(str(video))
        if abs(seconds-duration) > 1/30 + 0.001 or count < 1:
            raise RuntimeError(f"Clip validation failed for {clip['id']}: {count} frames, {seconds}s")
        clip.update({"start_seconds": start, "end_seconds": start+duration,
                     "duration_seconds": duration, "output_frames": count,
                     "video": video.name, "poster": poster.name, "preview": preview.name,
                     "video_bytes": video.stat().st_size, "video_sha256": sha256(video)})
        print(f"{clip['id']}: {duration:.3f}s, {count} frames, {video.stat().st_size/1e6:.2f} MB", flush=True)
        return clip

    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        manifest["clips"] = list(pool.map(extract, manifest["clips"]))
    manifest["encoding"] = {"video": "H.264", "audio": "AAC", "fps": 30,
                            "height": 1080, "pixel_format": "yuv420p", "faststart": True,
                            "preview_width": 480, "preview_fps": 6}
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False)+"\n")
    gallery = ROOT / "video-gallery"
    gallery.mkdir(exist_ok=True)
    (gallery / "clips.js").write_text("window.TASK_VIDEO_DATA = " + json.dumps(manifest, indent=2, ensure_ascii=False) + ";\n")
    print("Updated frame-accurate manifest and gallery data.")


if __name__ == "__main__":
    main()
