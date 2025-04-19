/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    // Ensure these paths cover all your components, pages, and styles
    './app/**/*.{js,ts,jsx,tsx,mdx}',
    './pages/**/*.{js,ts,jsx,tsx,mdx}', // Keep if using pages dir too
    './components/**/*.{js,ts,jsx,tsx,mdx}',
    './styles/**/*.{css}', 
  ],
  darkMode: 'class', // or 'media'
  theme: {
    extend: {
      colors: {
        primary: {
          50: '#e6f1ff',
          100: '#cce3ff',
          200: '#99c7ff',
          300: '#66abff',
          400: '#338fff',
          500: '#0073ff', // Primary blue
          600: '#005cbf',
          700: '#004499',
          800: '#002e66',
          900: '#001733',
        }
      },
      fontFamily: {
        sans: ['Inter', 'system-ui', 'sans-serif'],
      },
    },
  },
  plugins: [
    require('@tailwindcss/forms'),
  ],
}
