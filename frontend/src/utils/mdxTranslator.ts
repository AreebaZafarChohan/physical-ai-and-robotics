import { translate } from '@docusaurus/core/lib/client/exports/translate';
import { claudeService } from '../services/claudeService';

// Interface for parsed MDX content
export interface ParsedMdxContent {
  frontmatter: Record<string, any>;
  title: string;
  content: string;
  headings: string[];
  codeBlocks: string[];
}

/**
 * Parses MDX content to extract frontmatter, title, content, and other elements
 */
export const parseMdxContent = (mdxContent: string): ParsedMdxContent => {
  // Extract frontmatter (content between --- delimiters)
  let frontmatter: Record<string, any> = {};
  let content = mdxContent;
  
  const frontmatterRegex = /^---\n([\s\S]*?)\n---\n/;
  const frontmatterMatch = mdxContent.match(frontmatterRegex);
  
  if (frontmatterMatch) {
    const frontmatterStr = frontmatterMatch[1];
    content = mdxContent.replace(frontmatterMatch[0], '');
    
    // Simple YAML-like parsing (in a real implementation, you'd use a proper YAML parser)
    const lines = frontmatterStr.split('\n');
    for (const line of lines) {
      const colonIndex = line.indexOf(':');
      if (colonIndex > 0) {
        const key = line.substring(0, colonIndex).trim();
        let value = line.substring(colonIndex + 1).trim();
        
        // Try to parse as number or boolean
        if (value === 'true') value = true;
        else if (value === 'false') value = false;
        else if (!isNaN(Number(value))) value = Number(value);
        
        frontmatter[key] = value;
      }
    }
  }
  
  // Extract title (first H1)
  const titleMatch = content.match(/^#\s+(.+)$/m);
  const title = titleMatch ? titleMatch[1] : '';
  
  // Extract headings
  const headingRegex = /^(#{2,6})\s+(.+)$/gm;
  const headings: string[] = [];
  let headingMatch;
  while ((headingMatch = headingRegex.exec(content)) !== null) {
    headings.push(headingMatch[2]);
  }
  
  // Extract code blocks
  const codeBlockRegex = /```[\s\S]*?```/g;
  const codeBlocks = content.match(codeBlockRegex) || [];
  
  // Remove code blocks from content for translation (they should remain unchanged)
  let contentWithoutCode = content;
  for (const codeBlock of codeBlocks) {
    contentWithoutCode = contentWithoutCode.replace(codeBlock, `{{CODE_BLOCK_${codeBlocks.indexOf(codeBlock)}}}`);
  }
  
  return {
    frontmatter,
    title,
    content: contentWithoutCode,
    headings,
    codeBlocks
  };
};

/**
 * Reconstructs MDX content after translation, putting code blocks back in place
 */
export const reconstructMdxContent = (
  parsedContent: ParsedMdxContent,
  translatedContent: string
): string => {
  let result = translatedContent;
  
  // Put code blocks back in their original positions
  for (let i = 0; i < parsedContent.codeBlocks.length; i++) {
    result = result.replace(`{{CODE_BLOCK_${i}}}`, parsedContent.codeBlocks[i]);
  }
  
  // Reconstruct with frontmatter
  let finalContent = '';
  
  if (Object.keys(parsedContent.frontmatter).length > 0) {
    finalContent += '---\n';
    for (const [key, value] of Object.entries(parsedContent.frontmatter)) {
      finalContent += `${key}: ${typeof value === 'string' ? `"${value}"` : value}\n`;
    }
    finalContent += '---\n\n';
  }
  
  finalContent += result;
  
  return finalContent;
};

/**
 * Translates MDX content using Claude API while preserving structure
 */
export const translateMdxContent = async (
  mdxContent: string,
  targetLanguage: string,
  sourceLanguage: string = 'en'
): Promise<string> => {
  // Parse the MDX content
  const parsedContent = parseMdxContent(mdxContent);
  
  // Create a translation prompt that preserves structure
  const systemPrompt = 
    `You are an expert translator for technical documentation. Translate the following content from ${sourceLanguage} to ${targetLanguage}. 
    Preserve the formatting, structure, and technical terminology. Maintain all Markdown/MDX syntax including:
    - Headers (#, ##, ###)
    - Lists (-, *, 1., 2.)
    - Bold (**text**) and italic (*text*)
    - Links ([text](url)) and images (![alt](url))
    - Blockquotes (>)
    - Tables
    - Special Docusaurus syntax
    Do NOT translate code blocks or technical terms that should remain in English. If translating to Urdu, ensure proper RTL formatting where appropriate.`;

  // Translate the content part
  const translatedContent = await claudeService.sendMessage(
    parsedContent.content,
    systemPrompt
  );

  const translatedText = translatedContent.content
    .filter(item => item.type === 'text')
    .map(item => item.text)
    .join('\n\n');

  // Reconstruct the MDX content with translated parts
  return reconstructMdxContent(parsedContent, translatedText);
};

/**
 * Translates an entire MDX file while preserving frontmatter and code blocks
 */
export const translateMdxFile = async (
  filePath: string,
  targetLanguage: string,
  sourceLanguage: string = 'en'
): Promise<string> => {
  // In a real implementation, you would read the file content from the file system
  // For now, we'll simulate reading the content
  const fileContent = `---
sidebar_position: 1
---

# Sample Document

This is a sample document to demonstrate MDX translation.

## First Section

Here is some content in the first section.

\`\`\`javascript
// This is a code block that should not be translated
console.log("Hello, world!");
\`\`\`

## Second Section

More content in the second section.`;

  return await translateMdxContent(fileContent, targetLanguage, sourceLanguage);
};

/**
 * Extracts and translates only the text content from MDX, leaving structure intact
 */
export const translateMdxTextContent = async (
  mdxContent: string,
  targetLanguage: string,
  sourceLanguage: string = 'en'
): Promise<string> => {
  // Parse the content to separate translatable text from structure
  const parsed = parseMdxContent(mdxContent);
  
  // Just translate the main content part
  const translatedContent = await claudeService.translateContent(
    parsed.content,
    targetLanguage,
    sourceLanguage
  );
  
  // Reconstruct with the translated content
  return reconstructMdxContent(parsed, translatedContent);
};