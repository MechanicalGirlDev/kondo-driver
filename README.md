# Kondo ICS Driver

Independent Rust crates for the Kondo ICS 3.5/3.6 servo protocol and RS-485
transport and command-line tool.

## Crates

- `kondo-ics`: `no_std` protocol types, encoding, and decoding.
- `kondo-ics-rs485`: serial transport, reusable transaction API, and `kondo-ics-cli`.

The RS-485 crate's `testing` feature exposes its in-memory fake port for
downstream integration tests.

## Build and test

Install Rust 1.97 and the platform serial-port build requirements. On Debian or
Ubuntu, install `pkg-config` and `libudev-dev`. Then run:

```sh
cargo fmt --all --check
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo test --workspace --all-features
cargo check -p kondo-ics --no-default-features
```

## License and attribution

Licensed under Apache-2.0. Copyright 2026 nop, MechanicalGirl LLC. See
[LICENSE](LICENSE) and [NOTICE](NOTICE).
