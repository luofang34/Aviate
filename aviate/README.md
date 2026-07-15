# aviate

> ⚠️ **Work in progress — experimental. No guarantees of any kind.**
>
> Provided **as-is, with no warranty** — express or implied — including correctness, safety, reliability, fitness for a particular purpose, or airworthiness. Not qualified or certified for any use. **Do not use it to fly or otherwise control a real vehicle, and do not rely on it in any safety-critical context.** Controlled bench bring-up and HITL testing are development-only; no hardware or flight path is qualified for operational use. APIs may change without notice.

Public entry point for **Aviate**, a minimal, deterministic, hard-real-time inner-loop flight control kernel (state estimation, stabilization control, actuator mixing).

`aviate` is a thin facade that re-exports [`aviate-core`](https://crates.io/crates/aviate-core); the layered implementation crates evolve behind it. The public API is **not stable** — it re-exports `aviate-core` wholesale and will change.

```toml
[dependencies]
aviate = "0.1"
```

`#![no_std]`, `#![forbid(unsafe_code)]`.

## License

Licensed under either of MIT or Apache-2.0 at your option.
