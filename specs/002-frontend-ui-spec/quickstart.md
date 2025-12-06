# Quickstart Guide: Frontend UI for Physical AI & Humanoid Robotics Platform

## Prerequisites

- Node.js 18+ installed
- npm or yarn package manager
- Git for version control

## Setup Instructions

### 1. Clone and Initialize the Project

```bash
# Clone the repository
git clone <repository-url>
cd <project-directory>

# Install dependencies
npm install
# OR
yarn install
```

### 2. Set up Docusaurus

```bash
# Install Docusaurus globally (if not already installed)
npm install -g @docusaurus/core@latest

# Create a new Docusaurus project (or integrate with existing)
npm init docusaurus@latest <project-name> classic
```

### 3. Configure Tailwind CSS

First, install the required dependencies:

```bash
npm install -D tailwindcss postcss autoprefixer
npx tailwindcss init -p
```

Then, update `tailwind.config.js` to include Docusaurus paths:

```js
/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    "./src/**/*.{js,jsx,ts,tsx}",
    "./static/**/*.{js,jsx,ts,tsx}",
    ...(require('@docusaurus/core/lib/babel/preset').getPreset(null, {
      // All Docusaurus-related paths are included automatically
    }).plugins[0][1]?.runtimeOpts?.content ?? []),
  ],
  theme: {
    extend: {
      // Extended theme will be here
      colors: {
        'neon-purple': '#8A2BE2',
        'cyber-purple': '#9370DB',
        'neon-cyan': '#00FFFF',
        'dark-bg': '#0F0F14',
        'dark-surface': '#1A1A20',
        'text-primary': '#FFFFFF',
        'text-secondary': '#B0B0B0',
      },
      spacing: {
        '18': '4.5rem',
        '88': '22rem',
        '100': '25rem',
      },
      fontFamily: {
        sans: ['Inter', 'system-ui', 'sans-serif'],
        'serif-display': ['Playfair Display', 'serif'],
        'arabic-urdu': ['Noto Sans Arabic', 'Noto Sans Urdu', 'sans-serif']
      },
      boxShadow: {
        'neon-sm': '0 0 8px rgba(138, 43, 226, 0.5)',
        'neon-md': '0 0 12px rgba(138, 43, 226, 0.5), 0 0 20px rgba(0, 255, 255, 0.3)',
        'neon-lg': '0 0 15px rgba(138, 43, 226, 0.6), 0 0 25px rgba(0, 255, 255, 0.4)'
      },
      animation: {
        'neon-pulse': 'neon-pulse 2s cubic-bezier(0.4, 0, 0.6, 1) infinite',
        'float': 'float 6s ease-in-out infinite',
        'glow': 'glow 2s ease-in-out infinite alternate'
      },
      keyframes: {
        'neon-pulse': {
          '0%, 100%': { opacity: 1 },
          '50%': { opacity: 0.5 }
        },
        'float': {
          '0%, 100%': { transform: 'translateY(0)' },
          '50%': { transform: 'translateY(-10px)' }
        },
        'glow': {
          'from': { textShadow: '0 0 5px #fff, 0 0 10px #8A2BE2, 0 0 15px #00FFFF' },
          'to': { textShadow: '0 0 10px #fff, 0 0 20px #8A2BE2, 0 0 30px #00FFFF' }
        }
      }
    },
  },
  plugins: [],
}
```

### 4. Update postcss.config.js

```js
module.exports = {
  plugins: {
    tailwindcss: {},
    autoprefixer: {},
  },
}
```

### 5. Create a global CSS file

Create `src/css/tailwind.css` and add:

```css
@tailwind base;
@tailwind components;
@tailwind utilities;

/* Custom styles */
:root {
  /* Define CSS variables that match Tailwind config */
}

/* Additional custom styles */
.docusaurus-highlight-code-line {
  background-color: rgba(0, 0, 0, 0.1);
  display: block;
  margin: 0 calc(-1 * var(--ifm-pre-padding));
  padding: 0 var(--ifm-pre-padding);
}

html[data-theme='dark'] .docusaurus-highlight-code-line {
  background-color: rgba(0, 0, 0, 0.3);
}

/* Custom animations */
@layer utilities {
  .animate-neon-pulse {
    animation: neon-pulse 2s cubic-bezier(0.4, 0, 0.6, 1) infinite;
  }
  
  .animate-float {
    animation: float 6s ease-in-out infinite;
  }
  
  .animate-glow {
    animation: glow 2s ease-in-out infinite alternate;
  }
}
```

### 6. Create Base Components

Create the base components as specified in your data model in `src/components/core/`:

- Button
- MetricBadge
- IconBubble
- etc.

### 7. Run the Development Server

```bash
npm run start
# OR
yarn start
```

The site will be available at http://localhost:3000.

## Development Workflow

1. Create new components in `src/components/`
2. Add pages in `src/pages/` or use Docusaurus docs in `docs/`
3. Use Tailwind classes for styling following the design system
4. Use Framer Motion for animations

## Adding New Pages

To add a new page, create a React component in `src/pages/`:

```jsx
// src/pages/custom-page.js
import React from 'react';
import Layout from '@theme/Layout';

export default function CustomPage() {
  return (
    <Layout title="Custom Page" description="Custom page example">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <h1>My Custom Page</h1>
            <p>This is a custom page example.</p>
          </div>
        </div>
      </div>
    </Layout>
  );
}
```

## Multi-language Support

Docusaurus supports multi-language sites out of the box. To add Urdu language support:

1. Create content in the `i18n/ur/` directory
2. Follow the same structure as the default language
3. Update `docusaurus.config.js` to include Urdu as a supported language

```js
module.exports = {
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
  },
  // ... rest of config
};
```

## Building for Production

```bash
npm run build
# OR
yarn build
```

This will generate a `build/` directory with the production-ready site.

## Deployment

The site can be deployed to various platforms like Vercel, Netlify, GitHub Pages, etc. Refer to Docusaurus deployment documentation for specific instructions.