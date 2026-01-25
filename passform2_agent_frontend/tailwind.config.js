/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    "./index.html",
    "./src/**/*.elm", // Scannt deinen Elm-Code nach Klassen
    "./src/**/*.ts",  // Falls du Klassen im TypeScript (Three.js) nutzt
  ],
  theme: {
    extend: {
      // --- 0. TYPOGRAFIE ---
      fontFamily: {
        // Wir setzen 'Open Sans' als primäre Sans-Schrift. 
        // Die Fallbacks sorgen für Stabilität, falls mal was nicht lädt.
        sans: ['"Open Sans"', 'ui-sans-serif', 'system-ui', 'sans-serif'],
      },

      // --- 1. FARBPALETTE ---
      colors: {
        'bg-dark': '#1a202c',
        'bg-main': '#1a202c',
        'nav-bg': '#1a365d',
        'rail-bg': '#171923',
        'drawer-bg': '#2d3748',
        'surface-white': '#ffffff',
        'brand-blue': '#3182ce',
        'blue-active': '#3182ce',
        'success': '#48bb78',
        'danger': '#f56565',
        'warning': '#ecc94b',
        'info': '#63b3ed',
        'text-main': '#e2e8f0',
        'text-muted': '#a0aec0',
        'text-dark': '#1a202c',
        'text-gray': '#a0aec0',
      },

      // --- 2. ABSTÄNDE & 3. LAYOUT ---
      spacing: {
        'xs': '4px',
        'sm': '8px',
        'md': '16px',
        'lg': '24px',
        'xl': '32px',
        'nav': '60px',  // Perfekt auf die BIBA-Seite abgestimmt
        'rail': '60px', 
        'drawer': '320px', 
      },

      // --- BORDER RADIUS ---
      borderRadius: {
        'industrial': '8px',
        'industrial-sm': '4px',
      },

      // --- 4. SCHATTEN ---
      boxShadow: {
        'sm': '0 1px 3px rgba(0,0,0,0.12)',
        'md': '0 4px 6px rgba(0,0,0,0.3)',
        'lg': '0 10px 15px rgba(0,0,0,0.4)',
        'glow-blue': '0 0 10px rgba(49, 130, 206, 0.4)',
      },

      // --- 5. ANIMATIONEN & TRANSITIONS ---
      transitionTimingFunction: {
        'industrial': 'cubic-bezier(0.4, 0, 0.2, 1)',
      },
      transitionDuration: {
        '200': '200ms',
      },
      keyframes: {
        'pulse-red': {
          '0%': { boxShadow: '0 0 0 0 rgba(245, 101, 101, 0.7)' },
          '70%': { boxShadow: '0 0 0 10px rgba(245, 101, 101, 0)' },
          '100%': { boxShadow: '0 0 0 0 rgba(245, 101, 101, 0)' },
        }
      },
      animation: {
        'pulse-error': 'pulse-red 2s infinite',
      }
    },
  },
  plugins: [],
}