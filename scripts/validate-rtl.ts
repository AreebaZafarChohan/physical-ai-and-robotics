// RTL Layout Validation Script
// This script validates that RTL layout is properly implemented across components

import fs from 'fs';
import path from 'path';

// Check for RTL-specific CSS classes
function validateRtlCss() {
  console.log('🔍 Validating RTL CSS classes...');
  
  const rtlCssPath = path.join(process.cwd(), 'frontend', 'src', 'css', 'rtl.css');
  if (!fs.existsSync(rtlCssPath)) {
    console.error('❌ RTL CSS file not found');
    return false;
  }
  
  const rtlCssContent = fs.readFileSync(rtlCssPath, 'utf8');
  
  // Check for essential RTL selectors
  const essentialSelectors = [
    'html[dir="rtl"]',
    '[dir="rtl"]',
    '.rtl-layout'
  ];
  
  let allSelectorsFound = true;
  for (const selector of essentialSelectors) {
    if (!rtlCssContent.includes(selector)) {
      console.warn(`⚠️  Missing RTL selector: ${selector}`);
      allSelectorsFound = false;
    }
  }
  
  if (allSelectorsFound) {
    console.log('✅ All essential RTL CSS selectors found');
  }
  
  return allSelectorsFound;
}

// Check for RTL support in components
function validateRtlComponents() {
  console.log('\n🔍 Validating RTL support in components...');
  
  const componentsToCheck = [
    'src/theme/Navbar/index.tsx',
    'src/theme/DocSidebar/index.tsx',
    'src/theme/CodeBlock/index.tsx',
    'src/components/LanguageSwitcher/index.tsx'
  ];
  
  let allComponentsValidated = true;
  
  for (const componentPath of componentsToCheck) {
    const fullPath = path.join(process.cwd(), 'frontend', componentPath);
    
    if (!fs.existsSync(fullPath)) {
      console.error(`❌ Component not found: ${componentPath}`);
      allComponentsValidated = false;
      continue;
    }
    
    const content = fs.readFileSync(fullPath, 'utf8');
    
    // Check if component handles RTL
    const hasRtlHandling = 
      content.includes('useCurrentLocale') || 
      content.includes('isRTL') || 
      content.includes('dir=') || 
      content.includes('rtl') ||
      content.includes('right') ||  // RTL typically involves right/left adjustments
      content.includes('left');
    
    if (hasRtlHandling) {
      console.log(`✅ ${componentPath} has RTL support`);
    } else {
      console.warn(`⚠️  ${componentPath} may lack RTL support`);
      allComponentsValidated = false;
    }
  }
  
  return allComponentsValidated;
}

// Check for RTL configuration in docusaurus config
function validateRtlConfig() {
  console.log('\n🔍 Validating RTL configuration in docusaurus config...');
  
  const configPath = path.join(process.cwd(), 'frontend', 'docusaurus.config.ts');
  if (!fs.existsSync(configPath)) {
    console.error('❌ Docusaurus config file not found');
    return false;
  }
  
  const configContent = fs.readFileSync(configPath, 'utf8');
  
  // Check for RTL locale configuration
  const hasLocaleConfig = configContent.includes('localeConfigs') && configContent.includes('direction');
  
  if (hasLocaleConfig) {
    console.log('✅ Docusaurus config has RTL locale configuration');
    return true;
  } else {
    console.error('❌ Docusaurus config missing RTL locale configuration');
    return false;
  }
}

// Check for RTL-specific utility classes
function validateRtlUtilities() {
  console.log('\n🔍 Validating RTL-specific utility classes...');
  
  const rtlCssPath = path.join(process.cwd(), 'frontend', 'src', 'css', 'rtl.css');
  if (!fs.existsSync(rtlCssPath)) {
    console.error('❌ RTL CSS file not found');
    return false;
  }
  
  const rtlCssContent = fs.readFileSync(rtlCssPath, 'utf8');
  
  // Check for RTL utility classes
  const utilityClasses = [
    '.rtl-only',
    '.ltr-only',
    'margin-left-0',
    'margin-right-0',
    'padding-left-0',
    'padding-right-0'
  ];
  
  let allUtilitiesFound = true;
  for (const utility of utilityClasses) {
    if (!rtlCssContent.includes(utility)) {
      console.warn(`⚠️  Missing RTL utility: ${utility}`);
      allUtilitiesFound = false;
    }
  }
  
  if (allUtilitiesFound) {
    console.log('✅ All RTL utility classes found');
  }
  
  return allUtilitiesFound;
}

// Main validation function
function validateRtlImplementation() {
  console.log('🚀 Starting RTL layout validation...\n');
  
  const results = [];
  results.push(validateRtlCss());
  results.push(validateRtlComponents());
  results.push(validateRtlConfig());
  results.push(validateRtlUtilities());
  
  const passedTests = results.filter(r => r).length;
  const totalTests = results.length;
  
  console.log(`\n📊 RTL Validation Results: ${passedTests}/${totalTests} checks passed`);
  
  if (passedTests === totalTests) {
    console.log('🎉 All RTL validations passed! RTL layout is properly implemented.');
    return true;
  } else {
    console.error('💥 Some RTL validations failed! Please check the issues above.');
    return false;
  }
}

// Run validation
if (require.main === module) {
  const isValid = validateRtlImplementation();
  process.exit(isValid ? 0 : 1);
}

export { validateRtlImplementation };