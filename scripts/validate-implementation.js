#!/usr/bin/env node

/**
 * Validation script to verify the Urdu translation implementation
 * Follows the steps outlined in the quickstart guide
 */

import fs from 'fs';
import path from 'path';

// Define paths
const projectRoot = path.join(process.cwd(), 'frontend');
const docsPath = path.join(projectRoot, 'docs');
const urduDocsPath = path.join(docsPath, 'ur');
const configPath = path.join(projectRoot, 'docusaurus.config.ts');
const envPath = path.join(process.cwd(), '.env');

console.log('🚀 Starting validation of Urdu translation implementation...\n');

// 1. Check if environment variables are properly set
function validateEnvironment() {
  console.log('🔍 Checking environment variables...');
  
  if (!fs.existsSync(envPath)) {
    console.error('❌ FAIL: .env file does not exist');
    return false;
  }
  
  const envContent = fs.readFileSync(envPath, 'utf8');
  if (!envContent.includes('CLAUDE_API_KEY')) {
    console.error('❌ FAIL: CLAUDE_API_KEY not found in .env file');
    return false;
  }
  
  console.log('✅ PASS: Environment variables properly configured');
  return true;
}

// 2. Check Docusaurus i18n configuration
function validateDocusaurusConfig() {
  console.log('\n🔍 Checking Docusaurus i18n configuration...');
  
  if (!fs.existsSync(configPath)) {
    console.error('❌ FAIL: docusaurus.config.ts does not exist');
    return false;
  }
  
  const configContent = fs.readFileSync(configPath, 'utf8');
  
  if (!configContent.includes('ur') && !configContent.includes('urdu')) {
    console.error('❌ FAIL: Urdu locale not configured in docusaurus.config.ts');
    return false;
  }
  
  if (!configContent.includes('direction') || !configContent.includes('rtl')) {
    console.error('❌ FAIL: RTL direction not configured for Urdu locale');
    return false;
  }
  
  console.log('✅ PASS: Docusaurus i18n configuration is correct');
  return true;
}

// 3. Check Urdu documentation directory structure
function validateUrduDocsStructure() {
  console.log('\n🔍 Checking Urdu documentation directory structure...');
  
  if (!fs.existsSync(urduDocsPath)) {
    console.error('❌ FAIL: docs/ur/ directory does not exist');
    return false;
  }
  
  // Check if there are some documentation files
  const urduFiles = fs.readdirSync(urduDocsPath, { withFileTypes: true });
  const mdxFiles = urduFiles.filter(file => 
    file.isFile() && (file.name.endsWith('.md') || file.name.endsWith('.mdx'))
  );
  
  if (mdxFiles.length === 0) {
    console.warn('⚠️  WARN: No Urdu documentation files found (this may be expected if translations haven\'t been generated yet)');
  } else {
    console.log(`✅ PASS: Found ${mdxFiles.length} Urdu documentation files`);
  }
  
  // Check if subdirectories exist
  const urduSubdirs = urduFiles.filter(dir => dir.isDirectory());
  if (urduSubdirs.length > 0) {
    console.log(`✅ PASS: Found ${urduSubdirs.length} Urdu documentation subdirectories`);
  }
  
  return true;
}

// 4. Check for the existence of key components
function validateKeyComponents() {
  console.log('\n🔍 Checking key components...');
  
  const componentsToCheck = [
    'src/components/LanguageSwitcher/index.tsx',
    'src/theme/UrduLayout/index.tsx',
    'src/css/rtl.css',
    'src/utils/locale.ts'
  ];
  
  let allComponentsExist = true;
  
  for (const componentPath of componentsToCheck) {
    const fullPath = path.join(projectRoot, componentPath);
    if (!fs.existsSync(fullPath)) {
      console.error(`❌ FAIL: Component does not exist: ${componentPath}`);
      allComponentsExist = false;
    } else {
      console.log(`✅ PASS: Component exists: ${componentPath}`);
    }
  }
  
  return allComponentsExist;
}

// 5. Check for translation service and API endpoints
function validateTranslationService() {
  console.log('\n🔍 Checking translation service...');
  
  const servicePaths = [
    'src/services/translationService.ts',
    'src/services/claudeService.ts',
    'src/models/TranslationJob.ts',
    'scripts/translate.mjs'
  ];
  
  let allServicesExist = true;
  
  for (const servicePath of servicePaths) {
    const fullPath = path.join(projectRoot, servicePath);
    if (!fs.existsSync(fullPath)) {
      console.error(`❌ FAIL: Translation service does not exist: ${servicePath}`);
      allServicesExist = false;
    } else {
      console.log(`✅ PASS: Translation service exists: ${servicePath}`);
    }
  }
  
  return allServicesExist;
}

// 6. Check for RTL support
function validateRtlSupport() {
  console.log('\n🔍 Checking RTL support...');
  
  const rtlPath = path.join(projectRoot, 'src/css/rtl.css');
  if (!fs.existsSync(rtlPath)) {
    console.error('❌ FAIL: RTL CSS file does not exist');
    return false;
  }
  
  const rtlContent = fs.readFileSync(rtlPath, 'utf8');
  if (!rtlContent.includes('rtl') && !rtlContent.includes('direction')) {
    console.error('❌ FAIL: RTL CSS does not contain RTL-specific styles');
    return false;
  }
  
  console.log('✅ PASS: RTL support is properly implemented');
  return true;
}

// Main validation function
function runValidation() {
  const results = [];
  
  results.push(validateEnvironment());
  results.push(validateDocusaurusConfig());
  results.push(validateUrduDocsStructure());
  results.push(validateKeyComponents());
  results.push(validateTranslationService());
  results.push(validateRtlSupport());
  
  const passedTests = results.filter(r => r).length;
  const totalTests = results.length;
  
  console.log(`\n📊 Validation Results: ${passedTests}/${totalTests} checks passed`);
  
  if (passedTests === totalTests) {
    console.log('\n🎉 All validations passed! Urdu translation implementation is correctly set up.');
    console.log('\n✅ You can now:');
    console.log('   - Switch to Urdu language using the language switcher');
    console.log('   - Run translations using: npm run translate -- --target=ur --source=en');
    console.log('   - Start the development server: npm run start');
    process.exit(0);
  } else {
    console.error('\n💥 Some validations failed! Please fix the issues above.');
    process.exit(1);
  }
}

// Run the validation
runValidation();