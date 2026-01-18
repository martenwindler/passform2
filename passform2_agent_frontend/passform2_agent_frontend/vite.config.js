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

  build: {
    target: "esnext",
    assetsInlineLimit: 100000000,
    cssCodeSplit: false,
    minify: "terser",
    terserOptions: {
      compress: {
        drop_console: true,
      },
    },
  },

  server: {
    port: 3000,
    open: true,
  }
});