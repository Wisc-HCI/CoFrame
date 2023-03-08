import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import { comlink } from "vite-plugin-comlink";
import svgr from "vite-plugin-svgr";
import wasmPack from 'vite-plugin-wasm-pack';
import wasm from "vite-plugin-wasm";
import topLevelAwait from "vite-plugin-top-level-await";
import path from 'path'

// https://vitejs.dev/config/
export default defineConfig({
  base: '/CoFrame/', 
  plugins: [
    react(),
    svgr(),
    comlink(),
    wasmPack('./coframe-rust'),
    wasm(),
    topLevelAwait(),
  ],
  assetsInclude: ["**/*.gltf", "**/*.glb"],
  worker: {
    format: "es",
    plugins: [
      wasmPack('./coframe-rust'),
      comlink(),
      wasm(),
      topLevelAwait(),
    ],
  },
  resolve: {
    // mainFields:['module'],
    alias: {
      '$simple-vp': path.resolve('.', '$simple-vp'),
    },
  },
  server: {
    fs: {
      // Allow serving files from one level up to the project root
      allow: ['..'],
    },
  },
  test: {
    deps: { inline: ["@people_and_robots/lively", /\.wasm$/] },
  }
});
