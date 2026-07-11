// emscriptencompat.js — browser-compatibility shims for the emsdk 6 runtime.
//
// Loaded via --pre-js, so this runs synchronously at mame.js parse time,
// BEFORE the emscripten runtime calls getMemoryBuffer() → toResizableBuffer()
// (which happens later, asynchronously in receiveInstance() after wasm
// compilation).
//
// emsdk 6 opts into resizable ArrayBuffer heaps when the browser offers
// WebAssembly.Memory.prototype.toResizableBuffer(), but TextDecoder.decode()
// and WebGL upload entry points (texImage2D, etc.) reject views backed by
// resizable buffers. Deleting the method forces emscripten's
// getMemoryBuffer() catch{} fallback to the classic fixed-length-buffer path
// (replace-on-grow, views rebuilt) — the semantics every emsdk before 6 used.
// Cost: one heap copy per memory-growth event (a few times during boot).
if (typeof WebAssembly !== "undefined" && WebAssembly.Memory &&
    WebAssembly.Memory.prototype.toResizableBuffer) {
  delete WebAssembly.Memory.prototype.toResizableBuffer;
}
