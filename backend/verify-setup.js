/**
 * Verification Script for AI Assistant Setup
 * Run this after restarting the backend server
 */

const http = require('http');

const colors = {
  reset: '\x1b[0m',
  green: '\x1b[32m',
  red: '\x1b[31m',
  yellow: '\x1b[33m',
  blue: '\x1b[34m',
  cyan: '\x1b[36m'
};

function log(message, color = 'reset') {
  console.log(`${colors[color]}${message}${colors.reset}`);
}

function makeRequest(options, data = null) {
  return new Promise((resolve, reject) => {
    const req = http.request(options, (res) => {
      let body = '';
      res.on('data', (chunk) => body += chunk);
      res.on('end', () => {
        try {
          resolve({ status: res.statusCode, data: JSON.parse(body) });
        } catch (e) {
          resolve({ status: res.statusCode, data: body });
        }
      });
    });
    
    req.on('error', reject);
    if (data) req.write(JSON.stringify(data));
    req.end();
  });
}

async function runVerification() {
  log('\n🔍 AI Assistant Setup Verification\n', 'cyan');
  log('=' .repeat(60), 'blue');
  
  let allPassed = true;

  // Test 1: Health Check
  log('\n[Test 1] Health Check', 'yellow');
  try {
    const result = await makeRequest({
      hostname: 'localhost',
      port: 8080,
      path: '/health',
      method: 'GET'
    });
    
    if (result.status === 200) {
      log('✓ Server is running', 'green');
      
      const deps = result.data.dependencies;
      
      // Check vector store
      if (deps.vectorStore === 'connected') {
        log('✓ Vector store: connected', 'green');
      } else {
        log('✗ Vector store: disconnected', 'red');
        allPassed = false;
      }
      
      // Check LLM provider
      if (deps.llmProvider === 'connected') {
        log('✓ LLM provider: connected', 'green');
      } else {
        log('✗ LLM provider: disconnected (OpenAI API key issue)', 'red');
        log('  → Fix: Update OPENAI_API_KEY in .env file', 'yellow');
        allPassed = false;
      }
      
      // Check content index
      if (deps.contentIndex === 'ready') {
        log('✓ Content index: ready', 'green');
      } else {
        log('✗ Content index: not ready', 'red');
        log('  → This will be fixed after server restart', 'yellow');
        allPassed = false;
      }
    } else {
      log('✗ Server health check failed', 'red');
      allPassed = false;
    }
  } catch (error) {
    log('✗ Cannot connect to server', 'red');
    log(`  → Error: ${error.message}`, 'red');
    log('  → Make sure backend server is running on port 8080', 'yellow');
    allPassed = false;
  }

  // Test 2: Content Retrieval
  log('\n[Test 2] Content Retrieval', 'yellow');
  try {
    const result = await makeRequest({
      hostname: 'localhost',
      port: 8080,
      path: '/api/retrieve',
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': 'Bearer dummy-key'
      }
    }, {
      query: 'ROS 2 fundamentals',
      maxResults: 3
    });
    
    if (result.status === 200 && result.data.results && result.data.results.length > 0) {
      log(`✓ Content retrieval working (${result.data.results.length} results found)`, 'green');
      log(`  → Top result: ${result.data.results[0].chapterId}`, 'cyan');
    } else {
      log('✗ No content found (vector store is empty)', 'red');
      log('  → Server needs to be restarted to load 304 chunks', 'yellow');
      allPassed = false;
    }
  } catch (error) {
    log('✗ Content retrieval test failed', 'red');
    log(`  → Error: ${error.message}`, 'red');
    allPassed = false;
  }

  // Test 3: Chat Endpoint (only if OpenAI key is configured)
  log('\n[Test 3] Chat Endpoint', 'yellow');
  try {
    const result = await makeRequest({
      hostname: 'localhost',
      port: 8080,
      path: '/api/chat',
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': 'Bearer dummy-key'
      }
    }, {
      query: 'What is ROS 2?',
      sessionId: 'verify-test-' + Date.now()
    });
    
    if (result.status === 200 && result.data.response) {
      log('✓ Chat endpoint working', 'green');
      log(`  → Response length: ${result.data.response.length} chars`, 'cyan');
      log(`  → Grounded: ${result.data.isGrounded}`, 'cyan');
      log(`  → Confidence: ${result.data.confidenceScore}`, 'cyan');
      
      if (result.data.sources && result.data.sources.length > 0) {
        log(`  → Sources: ${result.data.sources.length} textbook sections`, 'cyan');
      }
    } else {
      log('✗ Chat endpoint failed', 'red');
      if (result.data.error) {
        log(`  → Error: ${result.data.error.message || result.data.error.code}`, 'red');
      }
      allPassed = false;
    }
  } catch (error) {
    log('✗ Chat endpoint test failed', 'red');
    log(`  → Error: ${error.message}`, 'red');
    allPassed = false;
  }

  // Final Summary
  log('\n' + '='.repeat(60), 'blue');
  if (allPassed) {
    log('\n✅ ALL TESTS PASSED! Setup is complete.\n', 'green');
    log('You can now use the chatbot on the frontend.', 'cyan');
  } else {
    log('\n⚠️  SOME TESTS FAILED. Please fix the issues above.\n', 'yellow');
    log('Common fixes:', 'cyan');
    log('1. Restart backend server: npm run dev', 'cyan');
    log('2. Fix OpenAI API key in .env file', 'cyan');
    log('3. Ensure textbook content is loaded (304 chunks)', 'cyan');
  }
  log('');
}

runVerification().catch(error => {
  log('\n✗ Verification script error:', 'red');
  console.error(error);
  process.exit(1);
});
