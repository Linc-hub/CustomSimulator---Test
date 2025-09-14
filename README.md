# Custom Simulator

This project contains a Rust application along with web assets.

## Prerequisites
- [Rust](https://www.rust-lang.org/tools/install) with `cargo`
- Windows target: `rustup target add x86_64-pc-windows-gnu`
- MinGW toolchain (`mingw-w64`) for linking

## Building
To cross-compile a Windows `.exe` and bundle assets, run:

```bash
./scripts/build-windows.sh
```

The bundled binary and resources will be in `dist/windows/`.

To run the build manually without the script:

```bash
cargo build --release --target x86_64-pc-windows-gnu --manifest-path stewart_sim/Cargo.toml
```

## Continuous Integration
GitHub Actions configuration at `.github/workflows/build.yml` automatically builds the Windows executable and uploads it as a workflow artifact for each push or pull request.
