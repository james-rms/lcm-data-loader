import { Experimental } from "@foxglove/extension";

import wasmUrl from "../data-loader.wasm";

export function activate(extensionContext: Experimental.ExtensionContext): void {
  extensionContext.registerDataLoader({
    type: "file",
    wasmUrl,
    supportedFileType: ".lcm",
  });
}
