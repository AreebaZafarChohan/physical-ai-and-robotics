# Research Summary: Docusaurus Urdu Translation Implementation

## 1. Docusaurus i18n Architecture

### Decision: Use Docusaurus Built-in i18n
Docusaurus has built-in internationalization support that works well with our requirements. The architecture involves:

- **Locale Configuration**: Defined in `docusaurus.config.ts` with supported locales
- **Content Organization**: Separate directories for each language under `docs/`
- **Translation Files**: JSON files for UI translations in `i18n/[locale]/`
- **Routing**: Automatic routing based on locale with fallback mechanisms

### Rationale
- Officially supported by Docusaurus
- Maintains English as source of truth
- Handles both content (MDX) and UI translations
- Supports versioning of translations

### Alternatives Considered
- Third-party i18n libraries (e.g., react-i18next)
- Custom solution with dynamic imports

## 2. MDX and JSON Translation Handling

### Decision: Separate MDX Files with Automated Generation
- **Content Translation**: Generate separate MDX files for Urdu in `docs/ur/` directory
- **UI Translation**: Use JSON files in `i18n/ur/` for UI elements
- **Automated Process**: Script to parse English MDX and generate Urdu MDX via Claude API

### Rationale
- Maintains Docusaurus conventions
- Allows for proper RTL layout handling
- Enables version-specific translations
- Supports both content and UI translation

### Alternatives Considered
- Inline translation within same MDX files
- Database-driven content management

## 3. RTL (Right-to-Left) Layout Considerations for Urdu

### Decision: CSS-based RTL with Direction Attribute
- **HTML Direction**: Set `dir="rtl"` attribute on HTML element for Urdu locale
- **CSS Adjustments**: Use logical CSS properties and RTL-specific styles
- **Component Adaptation**: Ensure all custom components support RTL

### Rationale
- Standard approach for RTL languages
- Works well with Tailwind CSS
- Properly handles text alignment and layout flow
- Compatible with Docusaurus theme components

### Alternatives Considered
- JavaScript-based RTL detection and transformation
- Separate RTL stylesheets

## 4. Tailwind CSS Adjustments for RTL

### Decision: Use Tailwind's Built-in RTL Support
- **Logical Properties**: Use `start`/`end` instead of `left`/`right` where applicable
- **RTL Prefix**: Use `rtl:` prefix for specific RTL overrides
- **Configuration**: Enable RTL mode in `tailwind.config.js`

### Rationale
- Native support in Tailwind CSS
- Minimal configuration required
- Maintains consistency with existing codebase
- Automatically handles many layout adjustments

### Alternatives Considered
- Custom RTL utility classes
- PostCSS plugins for RTL transformation

## 5. Language Switcher UI Integration

### Decision: Custom React Component with Docusaurus Context
- **Location**: Integrate into navigation header using Docusaurus theme customization
- **Functionality**: Use Docusaurus' `useLocation` and `navigate` for locale switching
- **Persistence**: Store user preference in localStorage with URL fallback

### Rationale
- Seamless integration with Docusaurus navigation
- Maintains routing consistency
- Provides good UX for language switching
- Follows Docusaurus theming patterns

### Alternatives Considered
- Third-party language switcher components
- Server-side language detection

## 6. Translation Workflow with Claude

### Decision: CLI Tool for Automated Translation
- **Implementation**: Create a Node.js CLI tool using Claude API via SpecKitPlus
- **Process**: Parse English MDX files, send to Claude, receive and format Urdu translation
- **Quality Control**: Implement review mechanism for AI-generated translations
- **Incremental Updates**: Track changes and only translate updated/new content

### Rationale
- Leverages existing SpecKitPlus infrastructure
- Provides consistent translation quality
- Automates repetitive translation tasks
- Maintains version control for translations

### Alternatives Considered
- Manual translation process
- Multiple AI service comparison