## Example LCM Data loader

First, download and extract the WASI SDK from [the releases page](https://github.com/WebAssembly/wasi-sdk/releases).

Then run:

```
export WASI_SDK_PATH=...
make
cd lcm-loader
npm install && npm run build && npm run package
```

Then install your extension.
