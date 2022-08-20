import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import { comlink } from "vite-plugin-comlink";
import svgr from "vite-plugin-svgr";
import wasmPack from 'vite-plugin-wasm-pack';
// import macrosPlugin from "vite-plugin-babel-macros"

// https://vitejs.dev/config/
export default defineConfig({
  plugins: [
    // macrosPlugin(),
    react(),
    svgr(),
    comlink(),
    wasmPack('./coframe-rust')
  ],
  assetsInclude: ["**/*.gltf", "**/*.glb"],
  worker: {
    plugins: [comlink()],
  },
  optimizeDeps: {disabled:false}
});
