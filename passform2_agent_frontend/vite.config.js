import { defineConfig } from 'vite';
import elmPlugin from 'vite-plugin-elm';
import { viteSingleFile } from "vite-plugin-singlefile";
import autoprefixer from 'autoprefixer';
import cssnano from 'cssnano';

export default defineConfig({
  plugins: [
    elmPlugin(),
    viteSingleFile()
  ],

  css: {
    postcss: {
      plugins: [
        autoprefixer(),
        cssnano({
          preset: 'default',
        }),
      ],
    },
  },

  server: {
    port: 3000,
    strictPort: true, 
    open: false,
  },

  envPrefix: ['VITE_', 'TAURI_'],

  build: {
    target: process.env.TAURI_PLATFORM === 'windows' ? 'chrome105' : 'esnext',
    assetsInlineLimit: 100000000, 
    cssCodeSplit: false,
    minify: !process.env.TAURI_DEBUG ? 'terser' : false,
    terserOptions: {
      compress: {
        drop_console: !process.env.TAURI_DEBUG,
      },
    },
    sourcemap: !!process.env.TAURI_DEBUG
  }
});