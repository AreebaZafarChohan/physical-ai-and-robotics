// Claude API integration service
export interface ClaudeMessageRequest {
  model: string;
  system?: string;
  messages: ClaudeMessage[];
  max_tokens: number;
  temperature?: number;
  top_p?: number;
  top_k?: number;
  stop_sequences?: string[];
  stream?: boolean;
}

export interface ClaudeMessage {
  role: 'user' | 'assistant';
  content: string | Array<{
    type: 'text' | 'image';
    text?: string;
    source?: {
      type: 'base64';
      media_type: string;
      data: string;
    };
  }>;
}

export interface ClaudeMessageResponse {
  id: string;
  type: 'message';
  role: 'assistant';
  content: Array<{
    type: 'text';
    text: string;
  }>;
  model: string;
  stop_reason: 'end_turn' | 'max_tokens' | 'stop_sequence';
  stop_sequence: string | null;
  usage: {
    input_tokens: number;
    output_tokens: number;
  };
}

export interface ClaudeErrorResponse {
  type: 'error';
  error: {
    type: string;
    message: string;
  };
}

export class ClaudeService {
  private readonly apiUrl = 'https://api.anthropic.com/v1/messages';
  private readonly apiKey: string;
  private readonly defaultModel = 'claude-3-sonnet-20240229';
  private readonly defaultMaxTokens = 4096;

  constructor(apiKey?: string) {
    this.apiKey = apiKey || process.env.CLAUDE_API_KEY || '';
    if (!this.apiKey) {
      console.warn('CLAUDE_API_KEY is not set. Claude API calls will fail.');
    }
  }

  /**
   * Sends a message to Claude API and returns the response
   */
  async sendMessage(
    userMessage: string,
    systemPrompt?: string,
    model: string = this.defaultModel,
    maxTokens: number = this.defaultMaxTokens
  ): Promise<ClaudeMessageResponse> {
    if (!this.apiKey) {
      throw new Error('CLAUDE_API_KEY is not configured');
    }

    const request: ClaudeMessageRequest = {
      model,
      messages: [
        {
          role: 'user',
          content: userMessage
        }
      ],
      max_tokens: maxTokens,
    };

    if (systemPrompt) {
      request.system = systemPrompt;
    }

    try {
      const response = await fetch(this.apiUrl, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'x-api-key': this.apiKey,
          'anthropic-version': '2023-06-01',
        },
        body: JSON.stringify(request)
      });

      if (!response.ok) {
        const errorData: ClaudeErrorResponse = await response.json();
        throw new Error(`Claude API error: ${response.status} - ${errorData.error.message}`);
      }

      const data: ClaudeMessageResponse = await response.json();
      return data;
    } catch (error) {
      console.error('Error calling Claude API:', error);
      throw error;
    }
  }

  /**
   * Translates content using Claude API
   */
  async translateContent(
    content: string,
    targetLanguage: string,
    sourceLanguage: string = 'en',
    customSystemPrompt?: string
  ): Promise<string> {
    const systemPrompt = customSystemPrompt || 
      `You are an expert translator. Translate the following content from ${sourceLanguage} to ${targetLanguage}. 
      Preserve the formatting, structure, and technical terminology. For MDX content, maintain all code blocks, 
      frontmatter, and special syntax. If translating to Urdu, use proper RTL formatting where appropriate.`;

    const response = await this.sendMessage(content, systemPrompt);
    
    // Extract the translated text from the response
    const translatedContent = response.content
      .filter(item => item.type === 'text')
      .map(item => item.text)
      .join('\n\n');
    
    return translatedContent;
  }

  /**
   * Translates a document with specific formatting requirements
   */
  async translateDocument(
    title: string,
    content: string,
    frontmatter: Record<string, any> = {},
    targetLanguage: string,
    sourceLanguage: string = 'en'
  ): Promise<string> {
    // Construct the document with frontmatter and content
    const documentParts = [];
    
    if (Object.keys(frontmatter).length > 0) {
      documentParts.push(`Frontmatter: ${JSON.stringify(frontmatter)}`);
    }
    
    documentParts.push(`# ${title}`);
    documentParts.push(content);
    
    const fullDocument = documentParts.join('\n\n');
    
    const systemPrompt = 
      `You are an expert translator for documentation. Translate the following document from ${sourceLanguage} to ${targetLanguage}. 
      Preserve the formatting, structure, and technical terminology. Maintain all Markdown/MDX syntax including:
      - Headers (#, ##, ###)
      - Lists (-, *, 1., 2.)
      - Code blocks (\`\`\`)
      - Inline code (\`)
      - Bold (**text**) and italic (*text*)
      - Links ([text](url)) and images (![alt](url))
      - Blockquotes (>)
      - Tables
      If translating to Urdu, ensure proper RTL formatting where appropriate while keeping code blocks LTR.`;

    const response = await this.sendMessage(fullDocument, systemPrompt);
    
    // Extract the translated text from the response
    const translatedContent = response.content
      .filter(item => item.type === 'text')
      .map(item => item.text)
      .join('\n\n');
    
    return translatedContent;
  }

  /**
   * Checks if the service is properly configured
   */
  isConfigured(): boolean {
    return !!this.apiKey;
  }
}

// Export a singleton instance of the Claude service
export const claudeService = new ClaudeService();