/**
 * Content Chunker for Physical AI & Humanoid Robotics RAG System
 * Handles the processing and chunking of textbook content for vector storage
 */

const DocumentProcessor = require('./document-processor');

class ContentChunker {
  constructor(options = {}) {
    this.documentProcessor = new DocumentProcessor(options);
    this.chunkSize = options.chunkSize || parseInt(process.env.CONTENT_CHUNK_SIZE) || 1000;
    this.overlap = options.overlap || Math.floor(this.chunkSize * 0.1); // 10% overlap
    this.semanticBoundaries = options.semanticBoundaries || [
      '\n## ',  // Markdown headers
      '\n### ', // Markdown subheaders
      '\n\n',   // Paragraph breaks
      '. ',     // Sentence endings
      '; ',     // Clause separators
    ];
  }

  /**
   * Process and chunk a single document
   */
  async chunkDocument(documentPath, metadata = {}) {
    try {
      // Process the document into text chunks
      const chunks = await this.documentProcessor.processFile(documentPath);

      // Add additional metadata to each chunk
      const processedChunks = chunks.map(chunk => ({
        ...chunk,
        metadata: {
          ...chunk.metadata,
          ...metadata,
          processedAt: new Date().toISOString(),
          chunkType: this._determineChunkType(chunk.content)
        }
      }));

      return processedChunks;
    } catch (error) {
      console.error(`Error chunking document ${documentPath}:`, error.message);
      throw error;
    }
  }

  /**
   * Process and chunk multiple documents
   */
  async chunkDocuments(documentPaths, commonMetadata = {}) {
    const allChunks = [];

    for (const path of documentPaths) {
      try {
        const chunks = await this.chunkDocument(path, commonMetadata);
        allChunks.push(...chunks);
      } catch (error) {
        console.error(`Skipping document ${path} due to error:`, error.message);
        // Continue processing other documents
      }
    }

    return allChunks;
  }

  /**
   * Process and chunk content from a directory
   */
  async chunkDirectory(directoryPath, commonMetadata = {}, recursive = true) {
    try {
      // This would use the document processor's directory functionality
      const processor = new DocumentProcessor();
      const chunks = await processor.processDirectory(directoryPath, recursive);

      // Add common metadata to each chunk
      return chunks.map(chunk => ({
        ...chunk,
        metadata: {
          ...chunk.metadata,
          ...commonMetadata,
          processedAt: new Date().toISOString(),
          chunkType: this._determineChunkType(chunk.content)
        }
      }));
    } catch (error) {
      console.error(`Error chunking directory ${directoryPath}:`, error.message);
      throw error;
    }
  }

  /**
   * Process raw text content into chunks
   */
  async chunkTextContent(content, metadata = {}) {
    try {
      // Process the content using the document processor
      const chunks = await this.documentProcessor.processTextContent(content, metadata);

      // Add chunk-specific metadata
      return chunks.map(chunk => ({
        ...chunk,
        metadata: {
          ...chunk.metadata,
          processedAt: new Date().toISOString(),
          chunkType: this._determineChunkType(chunk.content)
        }
      }));
    } catch (error) {
      console.error('Error chunking text content:', error.message);
      throw error;
    }
  }

  /**
   * Determine the type of content in a chunk
   */
  _determineChunkType(content) {
    const lowerContent = content.toLowerCase();

    if (lowerContent.includes('#') && (lowerContent.match(/^#+\s/mg) || []).length > 0) {
      return 'header-content';
    } else if (lowerContent.includes('fig') || lowerContent.includes('diagram') || lowerContent.includes('image')) {
      return 'diagram-description';
    } else if (lowerContent.includes('code') || lowerContent.includes('```') || lowerContent.match(/`[^`]+`/g)) {
      return 'code-content';
    } else if (lowerContent.includes('algorithm') || lowerContent.includes('step') || lowerContent.includes('procedure')) {
      return 'procedural-content';
    } else if (lowerContent.includes('definit') || lowerContent.includes('meaning') || lowerContent.includes('is the')) {
      return 'definition-content';
    } else {
      return 'general-content';
    }
  }

  /**
   * Optimize chunks for RAG retrieval
   */
  optimizeChunks(chunks) {
    return chunks.map(chunk => {
      // Ensure chunk content is appropriate length
      const trimmedContent = chunk.content.substring(0, this.chunkSize * 1.5); // Allow slight overflow

      return {
        ...chunk,
        content: trimmedContent,
        metadata: {
          ...chunk.metadata,
          wordCount: trimmedContent.split(/\s+/).length,
          charCount: trimmedContent.length,
          isOptimized: true
        }
      };
    }).filter(chunk => chunk.content.trim().length > 20); // Remove very small chunks
  }

  /**
   * Add semantic boundaries to improve chunk quality
   */
  async enhanceChunksWithSemantics(chunks) {
    // This would add semantic enrichment to chunks
    // For now, we'll just add some basic semantic tags

    return chunks.map(chunk => {
      const content = chunk.content.toLowerCase();
      const semanticTags = [];

      // Add tags based on content
      if (content.includes('ros') || content.includes('robot operating system')) {
        semanticTags.push('ros', 'robotics', 'middleware');
      }
      if (content.includes('gazebo') || content.includes('simulation')) {
        semanticTags.push('simulation', 'gazebo', 'physics');
      }
      if (content.includes('unity') || content.includes('game engine')) {
        semanticTags.push('unity', 'simulation', 'rendering');
      }
      if (content.includes('isaac') || content.includes('nvidia')) {
        semanticTags.push('nvidia', 'isaac', 'gpu');
      }
      if (content.includes('ai') || content.includes('intelligence') || content.includes('learning')) {
        semanticTags.push('ai', 'machine-learning', 'artificial-intelligence');
      }
      if (content.includes('physical') || content.includes('embodied')) {
        semanticTags.push('physical-ai', 'embodied-ai', 'embodiment');
      }

      return {
        ...chunk,
        metadata: {
          ...chunk.metadata,
          semanticTags: [...new Set([...(chunk.metadata.semanticTags || []), ...semanticTags])],
          hasEnrichment: true
        }
      };
    });
  }

  /**
   * Validate chunks for RAG appropriateness
   */
  validateChunks(chunks) {
    const validationResult = {
      validChunks: [],
      invalidChunks: [],
      statistics: {
        total: chunks.length,
        valid: 0,
        invalid: 0,
        averageLength: 0,
        totalCharacters: 0
      }
    };

    let totalLength = 0;

    for (const chunk of chunks) {
      const isValid = this._validateSingleChunk(chunk);

      if (isValid) {
        validationResult.validChunks.push(chunk);
        validationResult.statistics.valid++;
        totalLength += chunk.content.length;
      } else {
        validationResult.invalidChunks.push({
          ...chunk,
          validationErrors: this._getValidationErrors(chunk)
        });
        validationResult.statistics.invalid++;
      }
    }

    validationResult.statistics.averageLength = totalLength / (validationResult.statistics.valid || 1);
    validationResult.statistics.totalCharacters = totalLength;

    return validationResult;
  }

  /**
   * Validate a single chunk
   */
  _validateSingleChunk(chunk) {
    // Check content length
    if (!chunk.content || chunk.content.length < 10) {
      return false;
    }

    // Check for meaningful content (not just whitespace or special characters)
    const meaningfulText = chunk.content.replace(/[^\w\s]/gi, '').trim();
    if (meaningfulText.length < 5) {
      return false;
    }

    return true;
  }

  /**
   * Get validation errors for a chunk
   */
  _getValidationErrors(chunk) {
    const errors = [];

    if (!chunk.content || chunk.content.length < 10) {
      errors.push('Content too short');
    }

    const meaningfulText = chunk.content.replace(/[^\w\s]/gi, '').trim();
    if (meaningfulText.length < 5) {
      errors.push('Content lacks meaningful text');
    }

    return errors;
  }

  /**
   * Process content through the complete pipeline
   */
  async processContent(content, metadata = {}) {
    try {
      // Step 1: Chunk the content
      const chunks = await this.chunkTextContent(content, metadata);

      // Step 2: Optimize chunks
      const optimizedChunks = this.optimizeChunks(chunks);

      // Step 3: Enhance with semantics
      const enhancedChunks = await this.enhanceChunksWithSemantics(optimizedChunks);

      // Step 4: Validate chunks
      const validation = this.validateChunks(enhancedChunks);

      return {
        chunks: validation.validChunks,
        validation,
        processedAt: new Date().toISOString()
      };
    } catch (error) {
      console.error('Error in complete content processing pipeline:', error.message);
      throw error;
    }
  }
}

module.exports = ContentChunker;