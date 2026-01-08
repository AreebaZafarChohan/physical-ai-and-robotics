# Data Model: Docusaurus Urdu Translation Implementation

## 1. Documentation Content Model

### Entity: DocumentationContent
- **Attributes**:
  - `id` (string): Unique identifier for the document, derived from file path
  - `language` (string): Language code ('en', 'ur')
  - `filePath` (string): Relative path within docs directory
  - `title` (string): Document title
  - `content` (string): Raw MDX content
  - `metadata` (object): Frontmatter data (description, keywords, etc.)
  - `lastModified` (timestamp): Last modification time
  - `version` (string): Documentation version (for versioned docs)

### Relationships:
- Each English document (`language: 'en'`) can have one corresponding Urdu translation (`language: 'ur'`)
- Identified by matching `filePath` across languages

## 2. Translation Job Model

### Entity: TranslationJob
- **Attributes**:
  - `id` (string): Unique identifier for the translation job
  - `sourceFilePath` (string): Path to source English document
  - `targetFilePath` (string): Path to target Urdu document
  - `status` (enum): 'pending', 'in_progress', 'completed', 'failed', 'review_required'
  - `sourceLanguage` (string): Source language code ('en')
  - `targetLanguage` (string): Target language code ('ur')
  - `aiModelUsed` (string): Name of AI model used (e.g., 'Claude-3')
  - `timestamp` (timestamp): When the job was created
  - `completionTime` (timestamp): When the job was completed
  - `notes` (string): Any special notes or errors during translation

### State Transitions:
- `pending` → `in_progress` (when translation starts)
- `in_progress` → `completed` (when translation succeeds)
- `in_progress` → `failed` (when translation fails)
- `in_progress` → `review_required` (when quality check fails)
- `failed` → `pending` (when retry is initiated)

## 3. Locale Configuration Model

### Entity: LocaleConfig
- **Attributes**:
  - `localeCode` (string): Locale identifier ('en', 'ur')
  - `direction` (string): Text direction ('ltr', 'rtl')
  - `label` (string): Display name for language switcher ('English', 'اردو')
  - `path` (string): URL path prefix ('/en/', '/ur/')
  - `htmlLang` (string): HTML lang attribute ('en', 'ur')

## 4. UI Translation Model

### Entity: UITranslation
- **Attributes**:
  - `localeCode` (string): Locale identifier ('ur')
  - `translationKey` (string): Unique key for UI element
  - `originalText` (string): English text
  - `translatedText` (string): Urdu translation
  - `context` (string): Context for translation (e.g., 'header', 'footer', 'sidebar')
  - `lastUpdated` (timestamp): When translation was last updated

## 5. Language Switcher Model

### Entity: LanguageSwitcher
- **Attributes**:
  - `currentLocale` (string): Currently selected locale
  - `availableLocales` (array): List of available locales
  - `userPreference` (string): User's preferred locale (from localStorage)
  - `urlLocale` (string): Locale derived from URL
  - `fallbackLocale` (string): Default locale if none specified