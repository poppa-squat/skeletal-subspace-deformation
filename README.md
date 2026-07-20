# Skeletal Subspace Deformation

An interactive skeletal animation viewer in C++ / OpenGL. It loads a hierarchical
joint skeleton and a triangle mesh, binds them together with per-vertex attachment
weights, and deforms the mesh in real time as the skeleton is posed — the technique
variously called *skeletal subspace deformation*, *linear blend skinning*, or
*enveloping*.

Eighteen joints are exposed as Euler-angle sliders. Dragging one re-poses the
skeleton and re-skins the mesh every frame.

---

## Attribution

**This project began as coursework.** The application framework, build system,
and model assets were provided to me; I did not write them. Being precise about
the split, since it matters:

| | |
|---|---|
| **Written by me** | `assignment_code/assignment2/SkeletonNode.cpp` · `SkeletonNode.hpp`<br>`gloo/Transform.cpp` — the `GetLocalToAncestorMatrix` and `GetLocalToWorldMatrix` methods |
| **Provided to me** | `gloo/` — the "GLOO" rendering framework (scene graph, renderer, shaders, camera, mesh/OBJ loading, ImGui integration)<br>`assignment_code/assignment2/main.cpp` and `SkeletonViewerApp.cpp` — application scaffolding and slider UI<br>`assets/` — the `.obj` / `.skel` / `.attach` model data<br>`CMakeLists.txt` — the build system |

So: the skeleton parsing, the skinning math, the normal recomputation, and the
scene-graph transform composition are mine. The window, the renderer, the shaders,
and the models are not.

The provided framework carries no license or copyright header, and is reproduced
here only as the context needed to make my own code readable and runnable.

---

## Implementation notes

Each vertex is transformed by a weighted blend of per-joint matrices, where joint
`j` contributes `T_j · B_j⁻¹` — current world transform composed with the inverse
of its bind-pose world transform:

```
p' = Σ_j  w_j · (T_j · B_j⁻¹) · p
```

- **Vertex normals** are recomputed after every deformation, area-weighted by
  accumulating each triangle's un-normalised cross product into its three vertices
  before a final normalise. Skipping this leaves the mesh lit as though it were
  still in the bind pose.
- **Bind-pose inverses are precomputed.** Inverting `B_j` inside the per-vertex,
  per-joint inner loop costs ~227,000 matrix inversions per slider drag on a
  13k-vertex mesh. They're pose-invariant, so they're computed once at load and
  the 18 skinning matrices once per pose.

---

## Repository layout

```
assignment_code/assignment2/   Skeleton loading, skinning, normals  [mine]
gloo/                          Rendering framework                  [provided]
assets/assignment2/            Model data: .obj / .skel / .attach   [provided]
CMakeLists.txt                 Build system                         [provided]
```

### File formats

`.skel` — one joint per line, `x y z parent_index`. Position is relative to the
parent; `parent_index` refers to an earlier line, and `-1` marks the root.

`.attach` — one line per mesh vertex, holding that vertex's attachment weights to
each non-root joint. For an 18-joint skeleton each row carries 17 weights.

---

## Building

**This repository contains source only — it will not build as-is.** The
dependencies and the reference binaries are deliberately not redistributed here.
To build it you need `external/src/` populated with:

| Dependency | Version |
|---|---|
| GLFW | 3.3.2 |
| GLM | 0.9.9.8 |
| Dear ImGui | with `examples/` backends for GLFW + OpenGL3 |
| glad | OpenGL loader |
| stb | image loading |

CMake resolves GLM via `find_package(... REQUIRED)` and adds GLFW with
`add_subdirectory`, both pointed at `external/src/`, so configuration fails fast
if they're absent. Requires a C++11 compiler and OpenGL 3.

```bash
mkdir build && cd build
cmake ..
cmake --build . -j
```

## Running

Takes a model prefix, resolved relative to `assets/assignment2/`:

```bash
./build/assignment2 Model1
```

Press **`S`** to toggle between skeleton view and the skinned mesh — the mesh
starts hidden. Drag the sliders in the control panel to pose the joints.
