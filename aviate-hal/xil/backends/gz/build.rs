fn main() {
    // The FC-side shared-memory client is pure Rust (aviate-xil-shm,
    // #262): nothing links against C here anymore. The gz-sim system
    // plugin (the .so Gazebo dlopens at runtime) is a runtime
    // artifact built separately via CMake in plugin/, consuming the
    // cbindgen-generated contract header.
    //
    // plugin/aviate_gz_bridge.{cc,h} and plugin/shared_state.h are
    // the retired v1 FFI shim, kept only until the legacy-cleanup
    // issue removes them; they are deliberately not compiled.
}
