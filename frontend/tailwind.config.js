/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    "./src/**/*.{js,jsx,ts,tsx}",
    "./docs/**/*.mdx",
    "./src/pages/**/*.{js,jsx,ts,tsx}",
    "./src/components/**/*.{js,jsx,ts,tsx}",
  ],
  theme: {
    extend: {
      // Color palette tokens (FR-002, FR-010)
      colors: {
        // Neon/cyber purple theme
        'neon-purple': {
          50: '#f0f0ff',
          100: '#e0e0ff',
          200: '#c1c1ff',
          300: '#a3a3ff',
          400: '#8484ff',
          500: '#6565ff',  // Primary color
          600: '#5252cc',
          700: '#3e3e99',
          800: '#2b2b66',
          900: '#181833',
        },
        'neon-pink': {
          50: '#fff0f5',
          100: '#ffe0eb',
          200: '#ffc1d7',
          300: '#ff93b9',
          400: '#ff659b',
          500: '#ff377d',  // Accent color
          600: '#cc2c64',
          700: '#99214b',
          800: '#661632',
          900: '#330b19',
        },
        'dark-bg': '#0a0a1a',
        'dark-card': '#1a1a2a',
        'dark-text': '#e6e6ff',
        'light-bg': '#f8f8ff',
        'light-card': '#ffffff',
        'light-text': '#0a0a1a',

        // Semantic color tokens (FR-017)
        primary: {
          DEFAULT: '#6565ff',
          light: '#8484ff',
          dark: '#3e3e99',
        },
        secondary: {
          DEFAULT: '#ff377d',
          light: '#ff659b',
          dark: '#99214b',
        },
        background: {
          DEFAULT: '#0a0a1a',
          light: '#f8f8ff',
        },
        surface: {
          DEFAULT: '#1a1a2a',
          light: '#ffffff',
        },
        card: {
          DEFAULT: '#1a1a2a',
          light: '#ffffff',
        }
      },

      // Typography scale tokens (FR-003)
      fontSize: {
        'xs': ['0.75rem', { lineHeight: '1rem' }],     // 12px
        'sm': ['0.875rem', { lineHeight: '1.25rem' }], // 14px
        'base': ['1rem', { lineHeight: '1.5rem' }],    // 16px
        'lg': ['1.125rem', { lineHeight: '1.75rem' }], // 18px
        'xl': ['1.25rem', { lineHeight: '1.75rem' }],  // 20px
        '2xl': ['1.5rem', { lineHeight: '2rem' }],     // 24px
        '3xl': ['1.875rem', { lineHeight: '2.25rem' }],// 30px
        '4xl': ['2.25rem', { lineHeight: '2.5rem' }],  // 36px
        '5xl': ['3rem', { lineHeight: '1' }],          // 48px
        '6xl': ['3.75rem', { lineHeight: '1' }],       // 60px
        '7xl': ['4.5rem', { lineHeight: '1' }],        // 72px
        '8xl': ['6rem', { lineHeight: '1' }],          // 96px
        '9xl': ['8rem', { lineHeight: '1' }],          // 128px
      },

      // Border radius system (FR-004, FR-016)
      borderRadius: {
        none: '0px',
        sm: '0.125rem',  // 2px
        base: '0.25rem', // 4px
        md: '0.375rem',  // 6px
        lg: '0.5rem',    // 8px
        xl: '0.75rem',   // 12px
        '2xl': '1rem',   // 16px
        '3xl': '1.5rem', // 24px
        'full': '9999px',
      },

      // Spacing scale (FR-005)
      spacing: {
        0: '0px',
        0.5: '0.125rem', // 2px
        1: '0.25rem',    // 4px
        1.5: '0.375rem', // 6px
        2: '0.5rem',     // 8px
        2.5: '0.625rem', // 10px
        3: '0.75rem',    // 12px
        3.5: '0.875rem', // 14px
        4: '1rem',       // 16px
        5: '1.25rem',    // 20px
        6: '1.5rem',     // 24px
        7: '1.75rem',    // 28px
        8: '2rem',       // 32px
        9: '2.25rem',    // 36px
        10: '2.5rem',    // 40px
        11: '2.75rem',   // 44px
        12: '3rem',      // 48px
        14: '3.5rem',    // 56px
        16: '4rem',      // 64px
        20: '5rem',      // 80px
        24: '6rem',      // 96px
        28: '7rem',      // 112px
        32: '8rem',      // 128px
        36: '9rem',      // 144px
        40: '10rem',     // 160px
        44: '11rem',     // 176px
        48: '12rem',     // 192px
        52: '13rem',     // 208px
        56: '14rem',     // 224px
        60: '15rem',     // 240px
        64: '16rem',     // 256px
        72: '18rem',     // 288px
        80: '20rem',     // 320px
        96: '24rem',     // 384px
      },

      // Shadow system (FR-006)
      boxShadow: {
        sm: '0 1px 2px 0 rgb(0 0 0 / 0.05)',
        DEFAULT: '0 1px 3px 0 rgb(0 0 0 / 0.1), 0 1px 2px -1px rgb(0 0 0 / 0.1)',
        md: '0 4px 6px -1px rgb(0 0 0 / 0.1), 0 2px 4px -2px rgb(0 0 0 / 0.1)',
        lg: '0 10px 15px -3px rgb(0 0 0 / 0.1), 0 4px 6px -4px rgb(0 0 0 / 0.1)',
        xl: '0 20px 25px -5px rgb(0 0 0 / 0.1), 0 8px 10px -6px rgb(0 0 0 / 0.1)',
        '2xl': '0 25px 50px -12px rgb(0 0 0 / 0.25)',
        'neon': '0 0 15px rgba(101, 101, 255, 0.5)',
        'neon-pink': '0 0 15px rgba(255, 55, 125, 0.5)',
        'neon-lg': '0 0 25px rgba(101, 101, 255, 0.7)',
        'neon-inset': 'inset 0 0 15px rgba(101, 101, 255, 0.3)',
        'inner': 'inset 0 2px 4px 0 rgb(0 0 0 / 0.05)',
      },

      // Z-index scale (FR-015)
      zIndex: {
        auto: 'auto',
        0: '0',
        10: '10',
        20: '20',
        30: '30',
        40: '40',
        50: '50',
      },

      // Custom gradients (FR-011)
      backgroundImage: {
        'gradient-neon-purple': 'linear-gradient(135deg, #6565ff 0%, #c1c1ff 100%)',
        'gradient-neon-pink': 'linear-gradient(135deg, #ff377d 0%, #ff93b9 100%)',
        'gradient-neon-combo': 'linear-gradient(135deg, #6565ff 0%, #ff377d 100%)',
        'gradient-radial': 'radial-gradient(var(--tw-gradient-stops))',
        'gradient-conic': 'conic-gradient(from 180deg at 50% 50%, var(--tw-gradient-stops))',
      },

      // Animation definitions (FR-014)
      animation: {
        'pulse-neon': 'pulse-neon 2s cubic-bezier(0.4, 0, 0.6, 1) infinite',
        'bounce-neon': 'bounce 3s infinite',
        'float': 'float 4s ease-in-out infinite',
        'spin-slow': 'spin 3s linear infinite',
        'ping-slow': 'ping 3s cubic-bezier(0, 0, 0.2, 1) infinite',
        'pulse-slow': 'pulse 3s cubic-bezier(0.4, 0, 0.6, 1) infinite',
      },

      // Container widths and breakpoints (FR-007)
      maxWidth: {
        'xs': '20rem',      // 320px
        'sm': '24rem',      // 384px
        'md': '28rem',      // 448px
        'lg': '32rem',      // 512px
        'xl': '36rem',      // 576px
        '2xl': '42rem',     // 672px
        '3xl': '48rem',     // 768px
        '4xl': '56rem',     // 896px
        '5xl': '64rem',     // 1024px
        '6xl': '72rem',     // 1152px
        '7xl': '80rem',     // 1280px
      },
    },
  },
  plugins: [
    require('@headlessui/tailwindcss')({ prefix: 'ui' }),
    // Add a plugin to safely reference all utilities
    function ({ addBase }) {
      addBase({
        // Add safe references for common utilities that might be used
        ':root': {
          // Define font-family to avoid font-sans error
          '--font-sans': 'ui-sans-serif, system-ui, sans-serif, "Apple Color Emoji", "Segoe UI Emoji", "Segoe UI Symbol", "Noto Color Emoji"',
        }
      });
    }
  ],
}