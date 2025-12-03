# Point Cloud Essentials

Point Cloud Essentials is a Blender add‑on for fast, intuitive work with point clouds directly in the 3D Viewport.

![Point Cloud Essentials workflow preview](media/pce_workflow_preview.gif)
*(Example: import cloud → enable Colored → crop → split)*

Download (release builds): see the **Releases** page where you found this repository or release announcement.

This free edition focuses on a simple but powerful workflow:
- load a mesh that represents a point cloud,
- enable **Colored** to see millions of points in real time,
- use **Cropbox** and **Screen Selection** tools to split, keep or delete parts of the cloud.

It is designed to make everyday work with scans and dense point data feel natural inside Blender.

---

## What it does

Point Cloud Essentials provides:

- **Custom GPU point rendering** – displays large point sets smoothly, without heavy Geometry Nodes setups.
- **Colored mode** – shows your cloud as dense colored points driven by vertex colors.
- **Source mode** – toggles original meshes on/off so you can still work with geometry directly when needed.
- **Cropbox** – interactive 3D container (box / cylinder) that lets you:
  - fit to selected meshes or empties,
  - split the cloud into logical chunks,
  - cut away everything outside / inside the volume.
- **Screen selection tools** – Box, Lasso and Circle selection in screen space for working on the GLSL points:
  - select parts of the cloud directly in the viewport,
  - then **Split** or **Cut** them into separate meshes.
- **Multi‑cloud workflow** – you can load several clouds, split, duplicate and join pieces, and still keep a consistent Colored/Source view.

The add‑on works with **mesh objects that behave like point clouds** – i.e. meshes whose vertices carry color data and where faces are not important (they are simply ignored for the GLSL rendering).  
If your importer brings scans as meshes with colored vertices (PLY, FBX, GLB, etc.), Point Cloud Essentials will treat them uniformly as point clouds.

---

## Who it is for

This free edition is aimed at people who need to inspect and roughly organize point clouds inside Blender:

- architects and visualization artists,
- surveyors and scan specialists,
- 3D generalists who work with photogrammetry or LiDAR data,
- researchers and students dealing with dense spatial datasets.

The goal is to provide a **comfortable “first layer” workflow**: open, inspect, orbit, isolate parts, split into pieces – all without fighting the tools.

---

## Mission

In many industries, working with point clouds is already standard. Blender, as an open tool, historically has had no native, convenient way to handle large colored clouds in the viewport.

Point Cloud Essentials exists to close this gap and to make high‑quality point‑cloud work possible in an open ecosystem.  

The free version is available so that:
- enthusiasts,
- students,
- research groups,
- small teams

can use it without an upfront budget barrier.

If the add‑on saves you time or becomes part of your workflow, you can support development here (always optional, but very helpful):

> https://www.patreon.com/OlstFlow

---

## Important / Status

- This is an early **beta** of the free edition. Bugs and edge cases are still possible.
- Do not rely on it for one‑shot, mission‑critical projects without backups.
- Always keep a copy of your original scan data and `.blend` files.
- Developed and tested primarily with **Blender 5.0** (Windows, portable build). Other versions may work, but are not officially supported yet.

The add‑on is provided “as is” – you use it at your own risk.

---

## Pro version

For advanced production use there is a separate project in development – **Point Cloud Essentials Pro**.  
It is focused on:

- deeper control over multiple clouds and collections,
- smarter selection, grouping and tagging,
- more powerful alignment and snapping tools,
- extended workflows for architecture, VFX and scan‑heavy pipelines.

The free edition is intentionally kept focused and simple; Pro will build on top of it with more specialized tools.

---

## Learning

A short video walkthrough of the Lite workflow will be available on my YouTube channel (search for **“OlstFlow Point Cloud Essentials”** or follow the link from the add‑on release page).

The basic idea is:

1. Drag‑and‑drop a point‑cloud mesh into the 3D Viewport.
2. Press **Colored** – GLSL points appear for all supported clouds in the scene.
3. Optionally enable **Source** to see the original meshes alongside the GLSL clouds.
4. Use **Cropbox** or **Screen Selection** (Box/Lasso/Circle) to isolate the area you care about.
5. Run **Split** or **Cut** to create new meshes for the selected parts.

Once this basic cycle feels familiar, the rest of the workflow becomes quite intuitive.

---

## Known issues / Limitations

- **Performance of Circle / Lasso:** on very heavy clouds (millions of points) screen‑space Circle and Lasso selection can be slower than simple Box selection, especially on older GPUs.
- **Undo inside Screen Selection:** at the moment, `Ctrl+Z` behaves as Blender’s global Undo for the operator, not as a fine‑grained “step back one selection stroke” system.  
  - To quickly clear a selection you can drag Box/Lasso/Circle over empty space (outside all points).  
  - A more precise selection‑undo workflow is planned for future versions.
- **Import is external:** the add‑on does not include its own importers. It expects that your point cloud is already in the scene as mesh objects with vertices (and, ideally, vertex colors). Supported formats therefore depend on which import add‑ons you use in Blender (PLY, FBX, GLB, etc.).
- **Version support:** developed and tested with **Blender 5.0**. Older versions of Blender might work but are not officially supported.

If you run into a reproducible problem, please describe the steps and share a simplified test file if possible – this helps a lot.

---

## Disclaimer

This add‑on is provided **“as is”**, without warranty of any kind.  
The author and contributors are not responsible for any data loss, crashes or other issues that may arise from its use.

Always keep backup copies of important `.blend` files and original point‑cloud data. Use Point Cloud Essentials at your own risk.
