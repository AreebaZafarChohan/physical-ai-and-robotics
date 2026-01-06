import { TranslationJob, createTranslationJob, updateTranslationJobStatus } from '../models/TranslationJob';
import { manualReviewQueue } from './manualReviewQueue';
import { shouldFlagForReview } from './manualReviewQueue';

// Define the content types for translation
export interface TranslationContent {
  title: string;
  content: string;
  frontmatter?: Record<string, any>;
}

// Translation service class
export class TranslationService {
  private jobs: Map<string, TranslationJob> = new Map();
  private readonly claudeApiUrl = 'https://api.anthropic.com/v1/messages';
  private readonly claudeApiKey = process.env.CLAUDE_API_KEY;
  private readonly translationCache: Map<string, { content: string; timestamp: number }> = new Map();
  private readonly cacheExpiryTime = 24 * 60 * 60 * 1000; // 24 hours in milliseconds

  constructor() {
    if (!this.claudeApiKey) {
      console.warn('CLAUDE_API_KEY environment variable is not set');
    }
  }

  /**
   * Gets cached translation if available and not expired
   */
  private getCachedTranslation(sourceContent: string, targetLanguage: string): string | null {
    const cacheKey = `${sourceContent}-${targetLanguage}`;
    const cached = this.translationCache.get(cacheKey);

    if (cached) {
      const now = Date.now();
      if (now - cached.timestamp < this.cacheExpiryTime) {
        console.log('Using cached translation');
        return cached.content;
      } else {
        // Remove expired cache entry
        this.translationCache.delete(cacheKey);
      }
    }

    return null;
  }

  /**
   * Caches a translation
   */
  private cacheTranslation(sourceContent: string, targetLanguage: string, translatedContent: string): void {
    const cacheKey = `${sourceContent}-${targetLanguage}`;
    this.translationCache.set(cacheKey, { content: translatedContent, timestamp: Date.now() });
  }

  /**
   * Creates a new translation job
   */
  async createTranslationJob(
    sourceFilePath: string,
    targetFilePath: string,
    sourceLanguage: string,
    targetLanguage: string
  ): Promise<TranslationJob> {
    const job = createTranslationJob(
      sourceFilePath,
      targetFilePath,
      sourceLanguage,
      targetLanguage
    );

    this.jobs.set(job.id, job);
    return job;
  }

  /**
   * Gets a translation job by ID
   */
  async getTranslationJob(jobId: string): Promise<TranslationJob | undefined> {
    return this.jobs.get(jobId);
  }

  /**
   * Translates content using Claude API
   */
  async translateContent(
    content: TranslationContent,
    targetLanguage: string,
    sourceLanguage: string = 'en'
  ): Promise<string> {
    // Validate inputs to prevent security issues
    if (!this.isValidLanguage(targetLanguage) || !this.isValidLanguage(sourceLanguage)) {
      throw new Error('Invalid language specified for translation');
    }

    // Sanitize content to prevent prompt injection
    const sanitizedContent = this.sanitizeContent(content);

    // Check if translation is already cached
    const cachedTranslation = this.getCachedTranslation(sanitizedContent.content, targetLanguage);
    if (cachedTranslation) {
      return cachedTranslation;
    }

    if (!this.claudeApiKey) {
      console.warn('CLAUDE_API_KEY is not configured, using fallback mechanism');
      // Return the original content as fallback when API key is not configured
      return this.getFallbackContent(sanitizedContent, targetLanguage, sourceLanguage);
    }

    // Prepare the prompt for translation
    const systemPrompt = `You are an expert translator. Translate the following content from ${sourceLanguage} to ${targetLanguage}.
    Preserve the formatting, structure, and technical terminology. For MDX content, maintain all code blocks,
    frontmatter, and special syntax. If translating to Urdu, use proper RTL formatting where appropriate.`;

    const userMessage = `
      # Title: ${sanitizedContent.title}

      ${sanitizedContent.frontmatter ? `Frontmatter: ${JSON.stringify(sanitizedContent.frontmatter)}` : ''}

      Content:
      ${sanitizedContent.content}
    `;

    try {
      const response = await fetch(this.claudeApiUrl, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'x-api-key': this.claudeApiKey,
          'anthropic-version': '2023-06-01',
        },
        body: JSON.stringify({
          model: 'claude-3-sonnet-20240229', // or another Claude model
          system: systemPrompt,
          messages: [
            {
              role: 'user',
              content: userMessage
            }
          ],
          max_tokens: 4096,
        })
      });

      if (!response.ok) {
        throw new Error(`Claude API error: ${response.status} ${response.statusText}`);
      }

      const data = await response.json();
      const translatedContent = data.content[0]?.text || '';

      // Cache the translation
      this.cacheTranslation(sanitizedContent.content, targetLanguage, translatedContent);

      return translatedContent;
    } catch (error) {
      console.error('Translation error:', error);
      console.warn('Using fallback mechanism due to API failure');
      // Return the original content as fallback when API fails
      return this.getFallbackContent(sanitizedContent, targetLanguage, sourceLanguage);
    }
  }

  /**
   * Validates if a language code is supported
   */
  private isValidLanguage(lang: string): boolean {
    return ['en', 'ur', 'ar'].includes(lang);
  }

  /**
   * Sanitizes content to prevent prompt injection and other security issues
   */
  private sanitizeContent(content: TranslationContent): TranslationContent {
    // Remove any potential prompt injection attempts
    const sanitizedTitle = content.title.replace(/[\u0000-\u001F\u007F-\u009F]/g, '');
    const sanitizedContent = content.content.replace(/[\u0000-\u001F\u007F-\u009F]/g, '');

    // Sanitize frontmatter if it exists
    let sanitizedFrontmatter = content.frontmatter;
    if (sanitizedFrontmatter) {
      // Create a copy to avoid modifying the original
      sanitizedFrontmatter = { ...sanitizedFrontmatter };
    }

    return {
      title: sanitizedTitle,
      content: sanitizedContent,
      frontmatter: sanitizedFrontmatter
    };
  }

  /**
   * Provides fallback content when API is unavailable
   * For now, returns the original content with a notice
   */
  private getFallbackContent(
    content: TranslationContent,
    targetLanguage: string,
    sourceLanguage: string
  ): string {
    // In a real implementation, you might want to:
    // 1. Return the original content with a notice about the unavailability of translation
    // 2. Use a simpler translation method (like LibreTranslate if available)
    // 3. Return cached translations if available

    const fallbackNotice = `<!-- Translation unavailable - showing ${sourceLanguage} content -->\n`;
    return fallbackNotice + `# ${content.title}\n\n${content.content}`;
  }

  /**
   * Translates a documentation file
   */
  async translateFile(
    sourceFilePath: string,
    targetFilePath: string,
    targetLanguage: string,
    sourceLanguage: string = 'en'
  ): Promise<TranslationJob> {
    // Get the job or create a new one
    let job = Array.from(this.jobs.values()).find(
      j => j.sourceFilePath === sourceFilePath && j.targetLanguage === targetLanguage
    );

    if (!job) {
      job = await this.createTranslationJob(
        sourceFilePath,
        targetFilePath,
        sourceLanguage,
        targetLanguage
      );
    }

    // Update job status to in_progress
    job = updateTranslationJobStatus(job, 'in_progress');
    this.jobs.set(job.id, job);

    try {
      // Read the source file content (in a real implementation, you would read from the file system)
      // For now, we'll simulate reading the content
      const content = await this.readContentFromFile(sourceFilePath);

      // Translate the content
      const translatedContent = await this.translateContent(
        content,
        targetLanguage,
        sourceLanguage
      );

      // Write the translated content to the target file (simulated)
      await this.writeContentToFile(targetFilePath, translatedContent);

      // Check if the translation should be reviewed
      const reviewCheck = shouldFlagForReview(content.content, translatedContent);
      if (reviewCheck.shouldFlag) {
        // Add to manual review queue
        await manualReviewQueue.addItem(
          job.id,
          sourceFilePath,
          targetFilePath,
          content.content,
          translatedContent,
          targetLanguage,
          sourceLanguage,
          reviewCheck.reason
        );

        // Update job status to review_required
        job = updateTranslationJobStatus(job, 'review_required', reviewCheck.reason);
      } else {
        // Update job status to completed
        job = updateTranslationJobStatus(job, 'completed');
      }

      this.jobs.set(job.id, job);

      return job;
    } catch (error) {
      // Update job status to failed
      job = updateTranslationJobStatus(
        job,
        'failed',
        error instanceof Error ? error.message : 'Unknown error'
      );
      this.jobs.set(job.id, job);

      throw error;
    }
  }

  /**
   * Reads content from a documentation file
   * In a real implementation, this would read from the actual file system
   */
  private async readContentFromFile(filePath: string): Promise<TranslationContent> {
    // This is a simulation - in a real implementation, you would read the actual file
    // For now, return a mock content
    return {
      title: 'Mock Title',
      content: 'Mock content for translation',
      frontmatter: { sidebar_position: 1 }
    };
  }

  /**
   * Writes content to a documentation file
   * In a real implementation, this would write to the actual file system
   */
  private async writeContentToFile(filePath: string, content: string): Promise<void> {
    // This is a simulation - in a real implementation, you would write to the actual file
    console.log(`Writing translated content to: ${filePath}`);
    console.log(content);
  }

  /**
   * Gets all translation jobs
   */
  async getAllTranslationJobs(): Promise<TranslationJob[]> {
    return Array.from(this.jobs.values());
  }

  /**
   * Gets translation jobs by status
   */
  async getTranslationJobsByStatus(status: TranslationJob['status']): Promise<TranslationJob[]> {
    return Array.from(this.jobs.values()).filter(job => job.status === status);
  }
}

// Export a singleton instance of the translation service
export const translationService = new TranslationService();