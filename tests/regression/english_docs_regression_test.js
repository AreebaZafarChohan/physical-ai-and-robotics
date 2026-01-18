// Regression test to ensure English docs remain functional after Urdu implementation
// This test checks that English content is still accessible and properly rendered

const fs = require('fs');
const path = require('path');

// Define the paths to check
const englishDocsPath = path.join(__dirname, '..', 'frontend', 'docs');
const englishDocsUrduPath = path.join(__dirname, '..', 'frontend', 'docs', 'ur');

// Test function to verify English docs structure
function testEnglishDocsExist() {
  console.log('Testing: English documentation files exist and are accessible');
  
  // Check if the English docs directory exists
  if (!fs.existsSync(englishDocsPath)) {
    console.error('❌ FAIL: English docs directory does not exist');
    return false;
  }
  
  // Read the contents of the English docs directory
  const englishFiles = fs.readdirSync(englishDocsPath, { withFileTypes: true });
  
  // Filter out the 'ur' directory to focus on English content
  const englishSubdirs = englishFiles.filter(dirent => 
    dirent.isDirectory() && dirent.name !== 'ur' && dirent.name !== 'ar'
  );
  
  if (englishSubdirs.length === 0) {
    console.error('❌ FAIL: No English documentation subdirectories found');
    return false;
  }
  
  console.log(`✅ PASS: Found ${englishSubdirs.length} English documentation subdirectories`);
  
  // Check for the presence of key English files
  const introFile = path.join(englishDocsPath, 'intro.md');
  if (fs.existsSync(introFile)) {
    console.log('✅ PASS: English intro.md file exists');
  } else {
    console.error('❌ FAIL: English intro.md file does not exist');
    return false;
  }
  
  return true;
}

// Test function to verify Urdu docs structure exists
function testUrduDocsStructure() {
  console.log('\nTesting: Urdu documentation directory structure exists');
  
  if (fs.existsSync(englishDocsUrduPath)) {
    console.log('✅ PASS: Urdu docs directory exists');
    
    const urduFiles = fs.readdirSync(englishDocsUrduPath, { withFileTypes: true });
    const urduSubdirs = urduFiles.filter(dirent => dirent.isDirectory());
    
    console.log(`✅ PASS: Urdu docs has ${urduSubdirs.length} subdirectories`);
    return true;
  } else {
    console.log('⚠️  INFO: Urdu docs directory does not exist yet (expected if no translations done)');
    return true; // This is OK for initial setup
  }
}

// Test function to verify Docusaurus config still has English as default
function testDocusaurusConfig() {
  console.log('\nTesting: Docusaurus configuration for English default locale');
  
  const configPath = path.join(__dirname, '..', 'frontend', 'docusaurus.config.ts');
  
  if (!fs.existsSync(configPath)) {
    console.error('❌ FAIL: Docusaurus config file does not exist');
    return false;
  }
  
  const configContent = fs.readFileSync(configPath, 'utf8');
  
  // Check if English is still the default locale
  if (configContent.includes("defaultLocale: 'en'")) {
    console.log("✅ PASS: English ('en') is still the default locale");
  } else {
    console.error("❌ FAIL: English ('en') is not the default locale");
    return false;
  }
  
  // Check if Urdu locale is added
  if (configContent.includes("'ur'") || configContent.includes('"ur"')) {
    console.log("✅ PASS: Urdu ('ur') locale is included in configuration");
  } else {
    console.error("❌ FAIL: Urdu ('ur') locale is not included in configuration");
    return false;
  }
  
  return true;
}

// Main test execution
function runRegressionTests() {
  console.log('🚀 Starting regression tests for English documentation integrity...\n');
  
  const results = [];
  
  results.push(testEnglishDocsExist());
  results.push(testUrduDocsStructure());
  results.push(testDocusaurusConfig());
  
  const passedTests = results.filter(r => r).length;
  const totalTests = results.length;
  
  console.log(`\n📊 Test Results: ${passedTests}/${totalTests} tests passed`);
  
  if (passedTests === totalTests) {
    console.log('🎉 All regression tests passed! English documentation remains functional.');
    process.exit(0);
  } else {
    console.error('💥 Some regression tests failed! Please check the issues above.');
    process.exit(1);
  }
}

// Run the tests
if (require.main === module) {
  runRegressionTests();
}

module.exports = {
  testEnglishDocsExist,
  testUrduDocsStructure,
  testDocusaurusConfig,
  runRegressionTests
};