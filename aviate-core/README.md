# aviate-core

> ⚠️ **Work in progress — experimental. No guarantees of any kind.**
>
> Provided **as-is, with no warranty** — express or implied — including correctness, safety, reliability, fitness for a particular purpose, or airworthiness. Not qualified or certified for any use. Do not deploy on real hardware or rely on it in any safety-critical context. APIs may change without notice.

Minimal, deterministic, hard-real-time inner-loop flight control kernel. `aviate-core` does exactly three things:

1. **State estimation** — attitude, position, velocity, angular-rate estimation.
2. **Stabilization & control** — rate, attitude, and velocity/altitude/position loops.
3. **Actuation output** — force/torque commands mapped to actuator outputs via a mixer.

It deliberately does **not** do navigation, mission management, networking, logging, or UI — those live outside this crate.

`#![no_std]`, `#![forbid(unsafe_code)]`.

Most users should depend on the [`aviate`](https://crates.io/crates/aviate) facade crate rather than `aviate-core` directly.

## License

Licensed under either of MIT or Apache-2.0 at your option.
