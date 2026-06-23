#!/bin/bash
# =====================================================================
# install.sh — build & install CurveTool.p for LightWave Modeler (macOS)
#
# Run from inside the unzipped folder:
#     chmod +x install.sh        (first time only)
#     ./install.sh
#
# Produces a universal (arm64 + x86_64) CurveTool.p next to this script,
# and copies it into your LightWave Plugins folder if one is found.
# Final step is a one-time "Add Plugins" inside Modeler (see README).
# =====================================================================

set -u
cd "$(dirname "$0")"

PLUGIN_NAME="TubularBrains"
SOURCE="TubularBrains.c"
OUTPUT="${PLUGIN_NAME}.p"

# --- EDIT THIS if your SDK lives elsewhere ---------------------------
# LightWave 2024 SDK include dir. The SDK is proprietary and NOT shipped in this
# repo — point this at your own LightWave install (verify: ls "$SDK"/lwserver.h).
SDK="/Applications/LightWaveDigital/LightWave3D_2024.2.0/sdk/lwsdk2024.2/include"
# Directory holding the SDK server boilerplate (servmain.c etc.).
SDK_SRC="$(dirname "$0")/lwsdk2024.2/source"
# ---------------------------------------------------------------------

echo "==> CurveTool build (macOS, Apple Silicon + Intel universal)"

# 1. compiler present?
if ! command -v clang >/dev/null 2>&1; then
    echo "✗ clang not found. Install Xcode Command Line Tools:"
    echo "    xcode-select --install"
    exit 1
fi

# 2. SDK headers present?
if [ ! -d "$SDK" ]; then
    echo "✗ SDK include folder not found:"
    echo "    $SDK"
    echo "  Edit the SDK=... line near the top of this script to point at"
    echo "  your LightWave SDK 'include' directory, then re-run."
    exit 1
fi
if [ ! -f "$SDK/lwserver.h" ]; then
    echo "✗ '$SDK' exists but doesn't look like an SDK include dir"
    echo "  (lwserver.h not found). Check the path and re-run."
    exit 1
fi

# 3. compile
echo "==> Compiling $SOURCE ..."
# The plugin must export the SDK module descriptor (_mod_descrip) and its
# Startup/Shutdown entry points, which live in the SDK 'source' boilerplate.
# We link servmain.c/startup.c/shutdown.c with our own ServerDesc (in
# CurveTool.c), so we deliberately omit servdesc.c and username.c.
clang -dynamiclib \
      -D_MACOS \
      -I"$SDK" \
      -o "$OUTPUT" \
      "$SOURCE" \
      "$SDK_SRC/servmain.c" \
      "$SDK_SRC/startup.c" \
      "$SDK_SRC/shutdown.c" \
      -lm \
      -framework CoreFoundation \
      -arch arm64 -arch x86_64

if [ $? -ne 0 ]; then
    echo ""
    echo "✗ Build failed."
    echo "  If the error is a missing header (e.g. lwmodtool.h), check the"
    echo "  exact name in your SDK and fix the #include in $SOURCE:"
    echo "      ls \"$SDK\" | grep -i tool"
    exit 1
fi

# 4. clear Gatekeeper quarantine so LightWave can load it
xattr -d com.apple.quarantine "$OUTPUT" 2>/dev/null || true

BUILT_PATH="$(pwd)/$OUTPUT"
echo "✓ Built: $BUILT_PATH"

# 5. try to copy into a LightWave Plugins folder, if one exists
INSTALLED=""
for base in \
    "$HOME/Library/Application Support/LightWave"* \
    "$HOME/Library/Application Support/NewTek/LightWave"* \
    "/Applications/LightWave"*; do
    [ -d "$base" ] || continue
    # look for a Plugins directory a couple of levels down
    dest="$(find "$base" -maxdepth 3 -type d -iname "Plugins" 2>/dev/null | head -n 1)"
    if [ -n "$dest" ]; then
        cp -f "$OUTPUT" "$dest/" && INSTALLED="$dest/$OUTPUT"
        break
    fi
done

echo ""
if [ -n "$INSTALLED" ]; then
    echo "✓ Copied to: $INSTALLED"
    echo "  (A copy also remains next to this script.)"
else
    echo "ℹ No LightWave Plugins folder was auto-detected."
    echo "  That's fine — just point Add Plugins at the file above."
fi

echo ""
echo "==> FINAL STEP (one time, inside Modeler):"
echo "    Utilities tab  ->  Plugins  ->  Add Plugins"
echo "    Select:  $BUILT_PATH"
echo "    The tool then appears as 'CurveTool' (Edit Plugins lets you"
echo "    assign it to a menu button or hotkey)."
echo ""
echo "Done."
