import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import { comlink } from "vite-plugin-comlink";
import svgr from "vite-plugin-svgr";
import wasmPack from 'vite-plugin-wasm-pack';
import path from 'path'

// https://vitejs.dev/config/
export default defineConfig({
  base: '/CoFrame/',
  plugins: [
    react(),
    svgr(),
    comlink(),
    wasmPack('./coframe-rust')
  ],
  assetsInclude: ["**/*.gltf", "**/*.glb"],
  worker: {
    plugins: [comlink()],
  },
  resolve: {
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
});
