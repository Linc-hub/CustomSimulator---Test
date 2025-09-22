let acceleratorPromise = null;

async function initAccelerator() {
  if (typeof WebAssembly === 'undefined') {
    return null;
  }

  try {
    const url = new URL('./wasm/jacobian.wasm', import.meta.url);
    const response = await fetch(url);
    if (!response.ok) {
      console.warn('Workspace accelerator wasm fetch failed', response.status, response.statusText);
      return null;
    }
    const bytes = await response.arrayBuffer();
    const { instance } = await WebAssembly.instantiate(bytes, {});
    const exports = instance.exports;
    if (!exports || typeof exports.evaluate !== 'function') {
      return null;
    }
    const memory = exports.memory;
    if (!(memory instanceof WebAssembly.Memory)) {
      return null;
    }
    const jacobianPtr = exports.jacobianPtr >>> 0;
    const resultPtr = exports.resultPtr >>> 0;
    const inputLength = 36;
    const stride = Float64Array.BYTES_PER_ELEMENT;
    const inputOffset = jacobianPtr / stride;
    const outputOffset = resultPtr / stride;
    let view = new Float64Array(memory.buffer);

    const ensureView = () => {
      if (view.buffer !== memory.buffer) {
        view = new Float64Array(memory.buffer);
      }
      return view;
    };

    return {
      estimate(matrix) {
        if (!matrix || matrix.length !== inputLength) {
          return null;
        }
        const current = ensureView();
        current.set(matrix, inputOffset);
        exports.evaluate();
        const refreshed = ensureView();
        const sigmaMin = refreshed[outputOffset];
        const sigmaMax = refreshed[outputOffset + 1];
        if (!Number.isFinite(sigmaMin) || !Number.isFinite(sigmaMax)) {
          return null;
        }
        return { sigmaMin, sigmaMax };
      },
    };
  } catch (error) {
    console.warn('Workspace accelerator initialization failed', error);
    return null;
  }
}

export function getWorkspaceAccelerator() {
  if (!acceleratorPromise) {
    acceleratorPromise = initAccelerator();
  }
  return acceleratorPromise;
}
