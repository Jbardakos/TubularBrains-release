# TubularBrains — by tautologos
                                                            
                                                            
▄▄▄▄▄▄ ▄▄▄  ▄▄ ▄▄ ▄▄▄▄▄▄ ▄▄▄  ▄▄     ▄▄▄   ▄▄▄▄  ▄▄▄   ▄▄▄▄ 
  ██  ██▀██ ▀███▀   ██  ██▀██ ██    ██▀██ ██ ▄▄ ██▀██ ███▄▄ 
  ██  ██▀██   █     ██  ▀███▀ ██▄▄▄ ▀███▀ ▀███▀ ▀███▀ ▄▄██▀ 
                                                            

**Demo / early build. Provided as-is, work in progress.**

An interactive **LightWave Modeler** tool (compiled `MeshEditTool` `.p` plugin)
for drawing a tube along a multi-node chain with live, draggable viewport
handles. Interactive handles aren't possible from LScript — this is native C
built into a `.p`. In Modeler it registers as **TubularBrains**.

## Features

- Multi-node chain swept into one continuous, already-merged tube (miter-
  compensated for uniform wall thickness through bends).
- Handles editable from **every viewport** (Modeler-native picking).
- Per-node **scale** and **twist** handles (twist = per-node orientation, to
  fix cross-section "rubber ducky" warp on bends).
- **Caps**: None / Flat / Rounded.
- **Symmetry (X)**.
- Optional **Tube Weight Map** (0→1 along the chain).
- **TAB** subdivides the chain (inserts midpoint nodes) and keeps handles live.
- **Make Curve / Make 2pt Poly** toggles: also emit the node centreline on the
  next layer as a curve and/or a 2-point-poly chain, baked together with the tube.

The tube is a live preview; it bakes into the mesh when you **drop the tool**.
**Clear** discards.

## Build (macOS, Apple Silicon / Intel)

Requires the **LightWave 2024 SDK** — *not included here* (it is proprietary).
Point the build at your own SDK `include` directory.

```sh
clang -dynamiclib -D_MACOS \
      -I"/Applications/LightWaveDigital/LightWave3D_2024.2.0/sdk/lwsdk2024.2/include" \
      -o TubularBrains.p TubularBrains.c \
      -lm -framework CoreFoundation -arch arm64 -arch x86_64
xattr -d com.apple.quarantine TubularBrains.p
```

`install.sh` automates this (edit its `SDK=` line). `-D_MACOS` is required; the
result must be a universal binary (`lipo -archs` → `x86_64 arm64`).

## Build (Windows)

The C source is identical and portable — only the build differs. Build with
**Visual Studio (MSVC)** against the Windows SDK (define **`_MSWIN`**, export
`_mod_descrip` via the SDK's `serv.def`). `build_win.bat` automates it. *Untested
on a real Windows + LightWave setup — provided as-is.*

## Install

In Modeler: `Utilities → Plugins → Add Plugins`, select `TubularBrains.p`, and it
appears as **TubularBrains** (`Edit Plugins` to bind a menu/hotkey).

## Status

A demo / early build shared as-is — expect rough edges.
