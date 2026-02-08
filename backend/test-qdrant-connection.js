/**
 * Quick test to verify Qdrant connection
 */

require('dotenv').config();
const { QdrantClient } = require('@qdrant/js-client-rest');

async function testQdrantConnection() {
  console.log('🔍 Testing Qdrant Cloud Connection...\n');

  const qdrantUrl = process.env.QDRANT_URL;
  const qdrantApiKey = process.env.QDRANT_API_KEY;

  console.log(`URL: ${qdrantUrl}`);
  console.log(`API Key: ${qdrantApiKey.substring(0, 20)}...`);
  console.log('');

  try {
    const client = new QdrantClient({
      url: qdrantUrl,
      apiKey: qdrantApiKey,
    });

    console.log('✓ Qdrant client created');

    // Test connection by getting collections
    const collections = await client.getCollections();
    console.log(`✓ Connected successfully!`);
    console.log(`✓ Found ${collections.collections.length} collection(s)`);

    // Check if our collection exists
    const hasTextbookCollection = collections.collections.some(
      col => col.name === 'textbook-content'
    );

    if (hasTextbookCollection) {
      console.log('✓ Collection "textbook-content" exists');
      
      // Get collection info
      const collectionInfo = await client.getCollection('textbook-content');
      console.log(`✓ Collection has ${collectionInfo.points_count} documents`);
    } else {
      console.log('⚠ Collection "textbook-content" does not exist yet');
      console.log('  → Will be created when server starts');
    }

    console.log('\n✅ Qdrant connection test PASSED!\n');
    return true;

  } catch (error) {
    console.error('\n❌ Qdrant connection test FAILED!');
    console.error(`Error: ${error.message}\n`);
    
    if (error.message.includes('401')) {
      console.error('→ API key is invalid or expired');
    } else if (error.message.includes('ENOTFOUND')) {
      console.error('→ URL is incorrect or unreachable');
    } else {
      console.error('→ Check your Qdrant credentials in .env file');
    }
    
    return false;
  }
}

testQdrantConnection()
  .then(success => process.exit(success ? 0 : 1))
  .catch(error => {
    console.error('Unexpected error:', error);
    process.exit(1);
  });
