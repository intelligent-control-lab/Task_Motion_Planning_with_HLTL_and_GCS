"use strict";
(() => {
  const data = window.TASK_VIDEO_DATA;
  const player = document.querySelector("#player");
  const playlist = document.querySelector("#playlist");
  const message = document.querySelector("#player-message");
  const base = "../media/task_videos/";
  if (!data?.clips?.length) {
    message.textContent = "The clip manifest is missing. Regenerate the clips using scripts/split_paper_video.py.";
    return;
  }
  const clips = data.clips.filter((clip) => clip.origin !== "local");
  let filter = "all";
  let selected = 0;
  const $ = (selector) => document.querySelector(selector);
  const visibleClips = () => clips.filter((clip) => filter === "all" || clip.kind === filter);
  player.muted = true;

  function renderPlaylist() {
    playlist.replaceChildren();
    const visible = visibleClips();
    $("#filtered-count").textContent = `${visible.length} ${visible.length === 1 ? "clip" : "clips"}`;
    for (const clip of visible) {
      const index = clips.indexOf(clip);
      const li = document.createElement("li");
      const button = document.createElement("button");
      button.type = "button";
      button.className = "clip-button";
      button.setAttribute("aria-current", String(index === selected));
      button.setAttribute("aria-label", `Play ${clip.title}`);
      const thumb = document.createElement("span");
      thumb.className = "thumb";
      const image = document.createElement("img");
      image.src = base + clip.poster;
      image.alt = "";
      image.loading = "lazy";
      const duration = document.createElement("span");
      duration.textContent = `${clip.duration_seconds.toFixed(1)}s`;
      thumb.append(image, duration);
      const copy = document.createElement("span");
      copy.className = "clip-copy";
      const order = document.createElement("span");
      order.className = "clip-order";
      order.textContent = `${String(index + 1).padStart(2, "0")} / ${clip.kind === "local" ? "Local Drake" : clip.kind === "hardware" ? "Hardware experiment" : "Paper simulation"}`;
      const title = document.createElement("strong");
      title.textContent = clip.title;
      const caption = document.createElement("small");
      caption.textContent = clip.robots;
      copy.append(order, title, caption);
      button.append(thumb, copy);
      button.addEventListener("click", () => select(index, true));
      li.append(button);
      playlist.append(li);
    }
  }

  function select(index, play = false, updateHash = true) {
    selected = (index + clips.length) % clips.length;
    const clip = clips[selected];
    player.pause();
    player.poster = base + clip.poster;
    player.src = base + clip.video;
    player.setAttribute("aria-label", clip.title);
    player.load();
    $("#clip-title").textContent = clip.title;
    $("#clip-description").textContent = clip.description;
    $("#clip-kind").textContent = clip.kind === "local" ? "Local Drake / MOSEK trajectory" : clip.kind === "hardware" ? "Paper footage / Hardware experiment" : "Paper footage / Simulation";
    $("#clip-count").textContent = `${String(selected + 1).padStart(2, "0")} / ${clips.length}`;
    $("#download").href = base + clip.video;
    $("#code-link").href = encodeURI("../" + clip.example);
    $("#code-name").textContent = clip.example.replace("examples/", "");
    message.textContent = "";
    if (updateHash) history.replaceState(null, "", `#${clip.id}`);
    renderPlaylist();
    if (play) player.play().catch(() => { message.textContent = "Press the video’s play button to start playback."; });
  }

  function step(direction) {
    const visible = visibleClips();
    const current = visible.indexOf(clips[selected]);
    const next = current < 0 ? 0 : (current + direction + visible.length) % visible.length;
    select(clips.indexOf(visible[next]), true);
  }
  $("#previous").addEventListener("click", () => step(-1));
  $("#next").addEventListener("click", () => step(1));
  $("#speed").addEventListener("change", (event) => { player.playbackRate = Number(event.target.value); });
  player.addEventListener("loadedmetadata", () => { player.playbackRate = Number($("#speed").value); });
  player.addEventListener("ended", () => { if ($("#autonext").checked) step(1); });
  player.addEventListener("error", () => { message.textContent = "This clip could not be loaded. Try the MP4 link or check that media/task_videos is present."; });
  for (const button of document.querySelectorAll("[data-filter]")) {
    button.addEventListener("click", () => {
      filter = button.dataset.filter;
      for (const other of document.querySelectorAll("[data-filter]")) other.setAttribute("aria-pressed", String(other === button));
      renderPlaylist();
    });
  }
  function fromHash() {
    const index = clips.findIndex((clip) => clip.id === location.hash.slice(1));
    select(index < 0 ? 0 : index, false, false);
  }
  window.addEventListener("hashchange", fromHash);
  fromHash();
})();
