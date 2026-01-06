#!/usr/bin/env node

import fs from 'fs/promises';
import path from 'path';
import { fileURLToPath } from 'url';

// Get command line arguments
const args = process.argv.slice(2);
const sourceLang = args.find(arg => arg.startsWith('--source='))?.split('=')[1] || 'en';
const targetLang = args.find(arg => arg.startsWith('--target='))?.split('=')[1] || 'ur';
const specificFile = args.find(arg => arg.startsWith('--file='))?.split('=')[1];

// Get the project root directory
const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const projectRoot = path.resolve(__dirname, '..');

// Claude API configuration
const CLAUDE_API_KEY = process.env.CLAUDE_API_KEY;
const CLAUDE_API_URL = 'https://api.anthropic.com/v1/messages';

if (!CLAUDE_API_KEY) {
  console.error('Error: CLAUDE_API_KEY environment variable is required');
  process.exit(1);
}

/**
 * Translates content using Claude API
 */
async function translateWithClaude(text, targetLanguage = 'ur', sourceLanguage = 'en') {
  try {
    // Prepare the translation prompt
    const systemPrompt = `You are an expert translator. Translate the following content from ${sourceLanguage} to ${targetLanguage}. 
    Preserve the formatting, structure, and technical terminology. For MDX content, maintain all code blocks, 
    frontmatter, and special syntax. If translating to Urdu, use proper RTL formatting where appropriate.`;
    
    const response = await fetch(CLAUDE_API_URL, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'x-api-key': CLAUDE_API_KEY,
        'anthropic-version': '2023-06-01',
      },
      body: JSON.stringify({
        model: 'claude-3-sonnet-20240229', // or another Claude model
        system: systemPrompt,
        messages: [
          {
            role: 'user',
            content: text
          }
        ],
        max_tokens: 4096,
      })
    });

    if (!response.ok) {
      throw new Error(`Claude API error: ${response.status} ${response.statusText}`);
    }

    const data = await response.json();
    return data.content[0]?.text || '';
  } catch (error) {
    console.error('Translation error:', error);
    throw error;
  }
}

/**
 * Process a single MDX file
 */
async function processFile(filePath) {
  try {
    console.log(`Processing file: ${filePath}`);
    
    // Read the source file
    const content = await fs.readFile(filePath, 'utf8');
    
    // Translate the content
    const translatedContent = await translateWithClaude(content, targetLang, sourceLang);
    
    // Determine the target file path
    const relativePath = path.relative(path.join(projectRoot, 'docs'), filePath);
    const targetFilePath = path.join(projectRoot, 'docs', targetLang, relativePath);
    
    // Ensure target directory exists
    const targetDir = path.dirname(targetFilePath);
    await fs.mkdir(targetDir, { recursive: true });
    
    // Write the translated content
    await fs.writeFile(targetFilePath, translatedContent);
    
    console.log(`Translated: ${filePath} -> ${targetFilePath}`);
    return { source: filePath, target: targetFilePath, status: 'success' };
  } catch (error) {
    console.error(`Error processing file ${filePath}:`, error);
    return { source: filePath, target: null, status: 'error', error: error.message };
  }
}

/**
 * Process all MDX files in the source directory
 */
async function processAllFiles() {
  const sourceDir = path.join(projectRoot, 'docs', sourceLang);
  
  try {
    const files = await getAllMdxFiles(sourceDir);
    const results = [];
    
    for (const file of files) {
      const result = await processFile(file);
      results.push(result);
      
      // Add a small delay to avoid rate limiting
      await new Promise(resolve => setTimeout(resolve, 1000));
    }
    
    return results;
  } catch (error) {
    console.error('Error processing all files:', error);
    throw error;
  }
}

/**
 * Get all MDX files recursively from a directory
 */
async function getAllMdxFiles(dir) {
  const entries = await fs.readdir(dir, { withFileTypes: true });
  let files = [];
  
  for (const entry of entries) {
    const fullPath = path.join(dir, entry.name);
    
    if (entry.isDirectory()) {
      const subFiles = await getAllMdxFiles(fullPath);
      files = files.concat(subFiles);
    } else if (entry.isFile() && (entry.name.endsWith('.mdx') || entry.name.endsWith('.md'))) {
      files.push(fullPath);
    }
  }
  
  return files;
}

/**
 * Main function
 */
async function main() {
  console.log(`Starting translation from ${sourceLang} to ${targetLang}`);
  
  try {
    let results;
    
    if (specificFile) {
      // Process specific file
      const sourceFilePath = path.join(projectRoot, 'docs', sourceLang, specificFile);
      results = [await processFile(sourceFilePath)];
    } else {
      // Process all files
      results = await processAllFiles();
    }
    
    // Report results
    const successful = results.filter(r => r.status === 'success').length;
    const failed = results.filter(r => r.status === 'error').length;
    
    console.log(`\nTranslation completed: ${successful} successful, ${failed} failed`);
    
    if (failed > 0) {
      console.log('Failed files:');
      results.filter(r => r.status === 'error').forEach(r => {
        console.log(`  - ${r.source}: ${r.error}`);
      });
    }
  } catch (error) {
    console.error('Translation process failed:', error);
    process.exit(1);
  }
}

// Run the main function
if (process.argv[1] === __filename) {
  main();
}