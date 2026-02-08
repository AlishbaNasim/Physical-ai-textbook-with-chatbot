/**
 * Document Processor for Physical AI & Humanoid Robotics RAG System
 * Handles content extraction and preparation for vector storage
 */

const fs = require('fs').promises;
const path = require('path');

class DocumentProcessor {
  constructor(options = {}) {
    this.chunkSize = options.chunkSize || parseInt(process.env.CONTENT_CHUNK_SIZE) || 1000;
    this.overlap = options.overlap || Math.floor(this.chunkSize * 0.1); // 10% overlap
    this.supportedFormats = ['.md', '.txt', '.html'];
  }

  /**
   * Process content from text
   */
  async processTextContent(content, metadata = {}) {
    if (!content || typeof content !== 'string') {
      throw new Error('Content must be a non-empty string');
    }

    const chunks = this._splitText(content);

    return chunks.map((chunk, index) => ({
      id: `${metadata.id || 'content'}-chunk-${index}`,
      content: chunk.trim(),
      metadata: {
        ...metadata,
        chunkIndex: index,
        totalChunks: chunks.length,
        wordCount: chunk.split(/\s+/).length,
        charCount: chunk.length
      }
    }));
  }

  /**
   * Process content from a file
   */
  async processFile(filePath) {
    if (!fs.existsSync(filePath)) {
      throw new Error(`File does not exist: ${filePath}`);
    }

    const ext = path.extname(filePath).toLowerCase();
    if (!this.supportedFormats.includes(ext)) {
      throw new Error(`Unsupported file format: ${ext}. Supported: ${this.supportedFormats.join(', ')}`);
    }

    const content = await fs.readFile(filePath, 'utf8');
    const metadata = {
      source: filePath,
      fileName: path.basename(filePath),
      fileType: ext,
      lastModified: new Date().toISOString()
    };

    return this.processTextContent(content, metadata);
  }

  /**
   * Process content from multiple files
   */
  async processFiles(filePaths) {
    const allChunks = [];

    for (const filePath of filePaths) {
      try {
        const chunks = await this.processFile(filePath);
        allChunks.push(...chunks);
      } catch (error) {
        console.error(`Error processing file ${filePath}:`, error.message);
        // Continue with other files
      }
    }

    return allChunks;
  }

  /**
   * Process content from a directory recursively
   */
  async processDirectory(dirPath, recursive = true) {
    const allChunks = [];
    const files = await this._getFilesInDirectory(dirPath, recursive);

    for (const file of files) {
      try {
        const chunks = await this.processFile(file);
        allChunks.push(...chunks);
      } catch (error) {
        console.error(`Error processing file ${file}:`, error.message);
        // Continue with other files
      }
    }

    return allChunks;
  }

  /**
   * Split text into chunks of appropriate size
   */
  _splitText(text) {
    if (!text || typeof text !== 'string') {
      return [];
    }

    const paragraphs = text.split(/\n\s*\n/).filter(p => p.trim() !== '');
    const chunks = [];
    let currentChunk = '';

    for (const paragraph of paragraphs) {
      // If adding this paragraph would exceed chunk size
      if ((currentChunk + paragraph).length > this.chunkSize && currentChunk !== '') {
        // Add the current chunk to results
        if (currentChunk.trim() !== '') {
          chunks.push(currentChunk.trim());
        }

        // Start a new chunk with this paragraph
        currentChunk = paragraph + '\n\n';
      } else {
        // Add paragraph to current chunk
        currentChunk += paragraph + '\n\n';

        // If current chunk is getting large, add it and start a new one
        if (currentChunk.length >= this.chunkSize) {
          chunks.push(currentChunk.trim());
          currentChunk = '';
        }
      }
    }

    // Add the last chunk if it has content
    if (currentChunk.trim() !== '') {
      chunks.push(currentChunk.trim());
    }

    return chunks;
  }

  /**
   * Get all supported files in a directory
   */
  async _getFilesInDirectory(dirPath, recursive = true) {
    const files = [];
    const items = await fs.readdir(dirPath);

    for (const item of items) {
      const fullPath = path.join(dirPath, item);
      const stat = await fs.stat(fullPath);

      if (stat.isDirectory() && recursive) {
        const subDirFiles = await this._getFilesInDirectory(fullPath, recursive);
        files.push(...subDirFiles);
      } else if (stat.isFile()) {
        const ext = path.extname(item).toLowerCase();
        if (this.supportedFormats.includes(ext)) {
          files.push(fullPath);
        }
      }
    }

    return files;
  }

  /**
   * Extract text content from MDX/Markdown files
   */
  async processMarkdownContent(content, metadata = {}) {
    // For now, treat markdown as plain text
    // In a real implementation, we might want to parse the markdown structure
    return this.processTextContent(content, {
      ...metadata,
      format: 'markdown'
    });
  }

  /**
   * Validate content for RAG suitability
   */
  validateContent(content) {
    if (!content) {
      return {
        isValid: false,
        errors: ['Content cannot be empty']
      };
    }

    const errors = [];

    if (typeof content !== 'string') {
      errors.push('Content must be a string');
    }

    if (content.length < 10) {
      errors.push('Content must be at least 10 characters long');
    }

    if (content.length > 100000) { // 100k characters max
      errors.push('Content exceeds maximum length of 100,000 characters');
    }

    return {
      isValid: errors.length === 0,
      errors
    };
  }

  /**
   * Process and validate content before storing
   */
  async processAndValidate(content, metadata = {}) {
    const validation = this.validateContent(content);

    if (!validation.isValid) {
      throw new Error(`Content validation failed: ${validation.errors.join(', ')}`);
    }

    return this.processTextContent(content, metadata);
  }
}

module.exports = DocumentProcessor;