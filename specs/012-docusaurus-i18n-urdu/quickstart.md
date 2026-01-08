# Quickstart Guide: Docusaurus Urdu Translation

## Overview
This guide provides a quick setup for implementing Urdu translation in your Docusaurus project using AI-powered translation with Claude via SpecKitPlus.

## Prerequisites
- Node.js 16+ installed
- Docusaurus project already set up
- Claude API access through SpecKitPlus
- Git repository initialized

## Installation Steps

### 1. Clone the Repository
```bash
git clone <your-repo-url>
cd <your-repo-directory>
```

### 2. Install Dependencies
```bash
npm install
```

### 3. Set Up Environment Variables
Create a `.env` file in the root directory with your Claude API credentials:
```env
CLAUDE_API_KEY=your_claude_api_key
SPEC_KIT_PLUS_CONFIG_PATH=path_to_spec_kit_plus_config
```

### 4. Configure Docusaurus for i18n
Update your `docusaurus.config.ts` to include Urdu as a supported locale:

```typescript
module.exports = {
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',
      },
    },
  },
  // ... other config
};
```

### 5. Run Initial Translation
To translate all existing English content to Urdu:

```bash
npm run translate -- --target=ur --source=en
```

This command will:
- Parse all English MDX files in the `docs/` directory
- Send content to Claude for translation
- Generate corresponding Urdu MDX files in `docs/ur/`
- Create necessary directory structure

## Running the Development Server
To run the site with both English and Urdu languages:

```bash
npm run start
```

To run specifically in Urdu locale:
```bash
npm run start -- --locale ur
```

## Adding New Content
When adding new English documentation:

1. Create your MDX file in the appropriate `docs/en/` subdirectory
2. Run the translation command to generate Urdu version:
   ```bash
   npm run translate -- --target=ur --source=en --file=path/to/your/file.mdx
   ```
3. Or translate all content after adding new files:
   ```bash
   npm run translate -- --target=ur --source=en
   ```

## Customizing RTL Styles
To customize Right-to-Left styles for Urdu:

1. Modify `src/css/rtl.css` with your RTL-specific CSS rules
2. Use Tailwind's RTL variant prefixes where needed:
   ```css
   .rtl\:custom-class {
     /* RTL-specific styles */
   }
   ```

## Language Switcher Component
The language switcher is automatically available in the navbar. To customize it:

1. Modify the component in `src/components/LanguageSwitcher`
2. The component uses Docusaurus' `useDocusaurusContext` to manage locale switching

## API Endpoints
The translation functionality exposes the following API endpoints:

- `POST /api/translate` - Trigger new translation job
- `GET /api/translate/:jobId` - Get status of a translation job
- `GET /api/locales` - Get available locales

## Troubleshooting
- If translations are not appearing, ensure the `docs/ur/` directory exists and has proper file permissions
- If RTL styles aren't applying, check that `dir="rtl"` is set on the HTML element for Urdu locale
- For API errors, verify your Claude API credentials in the `.env` file