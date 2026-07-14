# aviate

> ⚠️ **Work in progress — experimental. No guarantees of any kind.**
>
> Provided **as-is, with no warranty** — express or implied — including correctness, safety, reliability, fitness for a particular purpose, or airworthiness. Not qualified or certified for any use. Do not deploy on real hardware or rely on it in any safety-critical context. APIs may change without notice.

Public entry point for **Aviate**, a minimal, deterministic, hard-real-time inner-loop flight control kernel (state estimation, stabilization control, actuator mixing).

`aviate` is a thin facade that re-exports [`aviate-core`](https://crates.io/crates/aviate-core). Depend on this crate for the stable public surface; the layered implementation crates evolve behind it.

```toml
[dependencies]
aviate = "0.1.0"
```

`#![no_std]`, `#![forbid(unsafe_code)]`.

## License

Licensed under either of MIT or Apache-2.0 at your option.
