/**
 * Load Textbook Content to Vector Store
 * This script loads all textbook content into the vector store at startup
 */

const path = require('path');
const fs = require('fs').promises;
const DocumentProcessor = require('./document-processor');
const vectorStoreService = require('./vector-store');

async function loadTextbookContent() {
  console.log('Loading textbook content to vector store...');

  try {
    // Initialize vector store
    await vectorStoreService.initialize();

    // Clear any existing content
    await vectorStoreService.clear();

    // Create document processor
    const processor = new DocumentProcessor();

    // Define the path to the textbook content
    // Go up from backend/src/rag/ to book-ai/, then into my-website/docs
    const textbookPath = path.join(__dirname, '..', '..', '..', 'my-website', 'docs');

    console.log(`Processing textbook content from: ${textbookPath}`);

    // Get all markdown files from the textbook directory
    const markdownFiles = await getAllMarkdownFiles(textbookPath);

    console.log(`Found ${markdownFiles.length} markdown files to process`);

    let totalChunks = 0;

    // Process each markdown file
    for (const filePath of markdownFiles) {
      console.log(`Processing file: ${filePath}`);

      try {
        // Read the file content
        const content = await fs.readFile(filePath, 'utf8');

        // Extract metadata from the file path
        const relativePath = path.relative(textbookPath, filePath);
        const pathParts = relativePath.split(path.sep);
        const fileName = path.basename(filePath, '.md');

        // Create metadata for the content
        const metadata = {
          id: filePath.replace(/[\/\\]/g, '_'), // Create a unique ID from the path
          source: relativePath,
          fileName: fileName,
          module: pathParts[0] || 'unknown',
          chapter: fileName,
          filePath: filePath,
          processedAt: new Date().toISOString()
        };

        // Process the markdown content into chunks
        const chunks = await processor.processMarkdownContent(content, metadata);

        console.log(`  - Created ${chunks.length} chunks`);

        // Add chunks to vector store
        if (vectorStoreService.isReady()) {
          const result = await vectorStoreService.addDocuments(chunks);
          console.log(`  - Added to vector store: ${result.count} chunks`);
          totalChunks += result.count;
        } else {
          console.log('  - Vector store not ready, skipping');
        }

      } catch (error) {
        console.error(`Error processing file ${filePath}:`, error.message);
      }
    }

    console.log(`Textbook content ingestion completed! Total chunks: ${totalChunks}`);

    // Show vector store stats
    const stats = await vectorStoreService.getStats();
    console.log('Vector store stats:', stats);

    return { success: true, totalChunks };

  } catch (error) {
    console.error('Textbook content loading failed:', error);
    throw error;
  }
}

async function getAllMarkdownFiles(dirPath) {
  const files = [];
  const items = await fs.readdir(dirPath);

  for (const item of items) {
    const fullPath = path.join(dirPath, item);
    const stat = await fs.stat(fullPath);

    if (stat.isDirectory()) {
      // Recursively get files from subdirectories
      const subDirFiles = await getAllMarkdownFiles(fullPath);
      files.push(...subDirFiles);
    } else if (stat.isFile() && item.endsWith('.md')) {
      files.push(fullPath);
    }
  }

  return files;
}

// Run the content loading if this script is executed directly
if (require.main === module) {
  loadTextbookContent()
    .then(() => {
      console.log('Content loading completed successfully');
      process.exit(0);
    })
    .catch(error => {
      console.error('Content loading failed:', error);
      process.exit(1);
    });
}

module.exports = { loadTextbookContent, getAllMarkdownFiles };