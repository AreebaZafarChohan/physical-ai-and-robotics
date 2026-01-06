#!/usr/bin/env node

// Compatibility validation script to verify no breaking changes to English docs
// This script checks for common issues that might break the Docusaurus site

const fs = require('fs');
const path = require('path');
const { execSync } = require('child_process');

// Define the paths to check
const frontendPath = path.join(__dirname, '..', '..', 'frontend');
const englishDocsPath = path.join(frontendPath, 'docs');
const docusaurusConfigPath = path.join(frontendPath, 'docusaurus.config.ts');

// Validation functions
function validateDocusaurusConfig() {
  console.log('🔍 Validating Docusaurus configuration...');
  
  if (!fs.existsSync(docusaurusConfigPath)) {
    console.error('❌ FAIL: Docusaurus config file does not exist');
    return false;
  }
  
  const configContent = fs.readFileSync(docusaurusConfigPath, 'utf8');
  
  // Check for basic syntax issues by attempting to parse
  try {
    // Try to check if the TypeScript compiles properly
    const result = execSync(`cd ${frontendPath} && npx tsc --noEmit --skipLibCheck docusaurus.config.ts`, { 
      encoding: 'utf8',
      stdio: ['pipe', 'pipe', 'pipe'] 
    });
    console.log('✅ PASS: Docusaurus config syntax is valid');
  } catch (error) {
    console.error('❌ FAIL: Docusaurus config has syntax errors:');
    console.error(error.stdout || error.stderr);
    return false;
  }
  
  // Check if required fields exist
  if (!configContent.includes('title:') && !configContent.includes('tagline:')) {
    console.error('❌ FAIL: Docusaurus config missing required fields (title, tagline)');
    return false;
  }
  
  console.log('✅ PASS: Docusaurus config contains required fields');
  return true;
}

function validateEnglishDocsStructure() {
  console.log('\n🔍 Validating English documentation structure...');
  
  if (!fs.existsSync(englishDocsPath)) {
    console.error('❌ FAIL: English docs directory does not exist');
    return false;
  }
  
  // Check for common MD/MDX file issues
  const filesToCheck = [];
  walkDir(englishDocsPath, filesToCheck);
  
  const mdxFiles = filesToCheck.filter(file => 
    file.endsWith('.mdx') || file.endsWith('.md')
  );
  
  console.log(`Found ${mdxFiles.length} documentation files to validate`);
  
  let allValid = true;
  
  for (const file of mdxFiles) {
    try {
      const content = fs.readFileSync(file, 'utf8');
      
      // Check for basic MDX/MD syntax issues
      if (content.includes('```') && (content.match(/```/g) || []).length % 2 !== 0) {
        console.error(`❌ FAIL: Unmatched code block delimiters in ${file}`);
        allValid = false;
        continue;
      }
      
      // Check for proper frontmatter (if it exists)
      if (content.startsWith('---')) {
        const endIndex = content.indexOf('---', 3);
        if (endIndex === -1) {
          console.error(`❌ FAIL: Unmatched frontmatter delimiters in ${file}`);
          allValid = false;
          continue;
        }
      }
      
      // Check for common Docusaurus-specific syntax
      if (content.includes('<') && content.includes('{') && content.includes('}')) {
        // This might be JSX/MDX syntax, do more specific checks
        if (content.includes('</') && !content.includes('/>')) {
          // Check if HTML-like tags are properly closed
          const openTags = (content.match(/<[A-Z][a-zA-Z0-9]*/g) || []).length;
          const closeTags = (content.match(/<\/[A-Z][a-zA-Z0-9]*>/g) || []).length;
          
          if (openTags !== closeTags) {
            console.warn(`⚠️  WARN: Potentially unmatched JSX tags in ${file} (this may be OK for MDX)`);
          }
        }
      }
      
    } catch (error) {
      console.error(`❌ FAIL: Error reading file ${file}: ${error.message}`);
      allValid = false;
    }
  }
  
  if (allValid) {
    console.log('✅ PASS: English documentation files have valid syntax');
  }
  
  return allValid;
}

function validateSidebarConfig() {
  console.log('\n🔍 Validating sidebar configuration...');
  
  const sidebarPath = path.join(frontendPath, 'sidebars.ts');
  
  if (!fs.existsSync(sidebarPath)) {
    console.error('❌ FAIL: Sidebar configuration file does not exist');
    return false;
  }
  
  try {
    // Try to check if the TypeScript compiles properly
    const result = execSync(`cd ${frontendPath} && npx tsc --noEmit --skipLibCheck sidebars.ts`, { 
      encoding: 'utf8',
      stdio: ['pipe', 'pipe', 'pipe'] 
    });
    console.log('✅ PASS: Sidebar config syntax is valid');
  } catch (error) {
    console.error('❌ FAIL: Sidebar config has syntax errors:');
    console.error(error.stdout || error.stderr);
    return false;
  }
  
  // Check if sidebar references existing files
  const sidebarContent = fs.readFileSync(sidebarPath, 'utf8');
  const englishDocsFiles = [];
  walkDir(englishDocsPath, englishDocsFiles);
  
  // Extract file paths referenced in sidebar
  const sidebarReferences = [...sidebarContent.matchAll(/['"]([^'"]*\.mdx?)['"]/g)]
    .map(match => match[1])
    .filter(ref => !ref.startsWith('http')); // Exclude external links
  
  let allReferencesValid = true;
  for (const ref of sidebarReferences) {
    // Convert sidebar reference to actual file path
    const filePath = path.join(englishDocsPath, ref);
    if (!fs.existsSync(filePath)) {
      // Also try with .md or .mdx extensions if not specified
      const filePathWithMd = path.join(englishDocsPath, ref + '.md');
      const filePathWithMdx = path.join(englishDocsPath, ref + '.mdx');
      
      if (!fs.existsSync(filePathWithMd) && !fs.existsSync(filePathWithMdx)) {
        console.error(`❌ FAIL: Sidebar references non-existent file: ${ref}`);
        allReferencesValid = false;
      }
    }
  }
  
  if (allReferencesValid) {
    console.log('✅ PASS: All sidebar references point to existing files');
  }
  
  return allReferencesValid;
}

function walkDir(dir, fileList) {
  const files = fs.readdirSync(dir);
  
  files.forEach(file => {
    const filePath = path.join(dir, file);
    const stat = fs.statSync(filePath);
    
    if (stat.isDirectory()) {
      walkDir(filePath, fileList);
    } else {
      fileList.push(filePath);
    }
  });
}

function runCompatibilityValidation() {
  console.log('🚀 Starting compatibility validation for English documentation...\n');
  
  const results = [];
  
  results.push(validateDocusaurusConfig());
  results.push(validateEnglishDocsStructure());
  results.push(validateSidebarConfig());
  
  const passedTests = results.filter(r => r).length;
  const totalTests = results.length;
  
  console.log(`\n📊 Validation Results: ${passedTests}/${totalTests} validation checks passed`);
  
  if (passedTests === totalTests) {
    console.log('🎉 All compatibility validations passed! English documentation remains compatible.');
    console.log('✅ No breaking changes detected in English documentation.');
    process.exit(0);
  } else {
    console.error('💥 Some compatibility validations failed! Please fix the issues above.');
    process.exit(1);
  }
}

// Run the validation
if (require.main === module) {
  runCompatibilityValidation();
}

module.exports = {
  validateDocusaurusConfig,
  validateEnglishDocsStructure,
  validateSidebarConfig,
  runCompatibilityValidation
};