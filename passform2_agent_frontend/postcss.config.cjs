// postcss.config.cjs
module.exports = {
  plugins: {
    autoprefixer: {}, // Fügt automatisch -webkit-, -moz- etc. hinzu
    cssnano: { preset: 'default' }, // Das sorgt für die radikale Minifizierung
    tailwindcss
  }
}