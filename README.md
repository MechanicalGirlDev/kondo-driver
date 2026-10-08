# Kondo ICS Driver

Independent Rust crates for the Kondo ICS 3.5/3.6 servo protocol and RS-485
transport and command-line tool.

## Crates

- `kondo-ics`: `no_std` protocol types, encoding, and decoding.
- `kondo-ics-rs485`: serial transport, reusable transaction API, and `kondo-ics-cli`.

The RS-485 crate's `testing` feature exposes its in-memory fake port for
downstream integration tests.

## Reiny 0.8 integration

Keep `kondo-ics` dependency-free and usable with `no_std`; keep
`kondo-ics-rs485` responsible for transport and transactions. Neither crate
declares Reiny ports, and `kondo-ics-cli` is a standalone bring-up tool.

A consuming adapter owns its `main.yaml`, compiled command/feedback types and
unit conversion. Open its named input/output ports and initialize the RS-485
bus before `Cloudy::ready()`. On `Cloudy::shutdown()`, complete the application's
safe-output policy before dropping the transport. Namespace identity belongs
to the deployment/module path, not the servo address. Use the transport's
`testing` feature for adapter integration tests without a physical bus.

Use published `reiny = "0.8.0"` and `reiny-build = "0.8.0"` in the adapter,
not these transport/protocol crates. Its runtime definition and schema
catalog use `version: 2`. Declare the executable, build and endpoint policies
in the adapter's own `main.yaml`: input `replay`/`buffer` and output
`qos`/`retention`. Callers reuse it through `source` and wire inputs with
`from`, without repeating child outputs.

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
