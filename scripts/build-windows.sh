#!/usr/bin/env bash
set -euo pipefail

# Build Windows binary
rustup target add x86_64-pc-windows-gnu >/dev/null 2>&1 || true
cargo build --release --target x86_64-pc-windows-gnu --manifest-path stewart_sim/Cargo.toml

# Bundle assets
DIST_DIR="dist/windows"
rm -rf "$DIST_DIR"
mkdir -p "$DIST_DIR"
cp stewart_sim/target/x86_64-pc-windows-gnu/release/stewart_sim.exe "$DIST_DIR/"
cp index.html workspace.js wasm_optimizer.js "$DIST_DIR/"
cp -r stewart_sim/pkg "$DIST_DIR/"
cp -r "Raw Information" "$DIST_DIR/" 2>/dev/null || true
