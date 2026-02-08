/**
 * Qdrant Vector Store Service for Physical AI & Humanoid Robotics RAG System
 * Handles vector database operations using Qdrant Cloud
 */

require('dotenv').config();
const { QdrantClient } = require('@qdrant/js-client-rest');

class QdrantVectorStoreService {
  constructor() {
    this.client = null;
    this.isInitialized = false;
    this.collectionName = 'textbook-content';
    this.vectorSize = 1536; // OpenAI text-embedding-ada-002 dimension
  }

  /**
   * Initialize Qdrant connection
   */
  async initialize() {
    try {
      const qdrantUrl = process.env.QDRANT_URL;
      const qdrantApiKey = process.env.QDRANT_API_KEY;

      if (!qdrantUrl || !qdrantApiKey) {
        throw new Error('QDRANT_URL and QDRANT_API_KEY must be set in .env');
      }

      // Initialize Qdrant client
      this.client = new QdrantClient({
        url: qdrantUrl,
        apiKey: qdrantApiKey,
      });

      // Check if collection exists, create if not
      await this.ensureCollection();

      this.isInitialized = true;
      console.log('Qdrant vector store initialized successfully');
    } catch (error) {
      console.error('Failed to initialize Qdrant:', error);
      throw error;
    }
  }

  /**
   * Ensure collection exists
   */
  async ensureCollection() {
    try {
      const collections = await this.client.getCollections();
      const exists = collections.collections.some(
        col => col.name === this.collectionName
      );

      if (!exists) {
        console.log(`Creating collection: ${this.collectionName}`);
        await this.client.createCollection(this.collectionName, {
          vectors: {
            size: this.vectorSize,
            distance: 'Cosine'
          }
        });
        console.log('Collection created successfully');
      } else {
        console.log(`Collection ${this.collectionName} already exists`);
      }
    } catch (error) {
      console.error('Error ensuring collection:', error);
      throw error;
    }
  }

  /**
   * Add documents to Qdrant (without embeddings for now - using keyword search)
   */
  async addDocuments(documents) {
    if (!this.isInitialized) {
      throw new Error('Vector store not initialized');
    }

    try {
      // For now, we'll store documents with dummy vectors since we don't have embeddings
      // In production, you'd generate embeddings using OpenAI's embedding API
      const points = documents.map((doc, index) => ({
        id: doc.id || `doc-${Date.now()}-${index}`,
        vector: Array(this.vectorSize).fill(0), // Dummy vector
        payload: {
          content: doc.content,
          metadata: doc.metadata || {},
          createdAt: new Date().toISOString()
        }
      }));

      await this.client.upsert(this.collectionName, {
        wait: true,
        points: points
      });

      console.log(`Added ${documents.length} documents to Qdrant`);

      return {
        success: true,
        documentIds: points.map(p => p.id),
        count: documents.length
      };
    } catch (error) {
      console.error('Error adding documents to Qdrant:', error);
      throw error;
    }
  }

  /**
   * Search documents using scroll (keyword-based for now)
   */
  async similaritySearch(query, k = 5) {
    if (!this.isInitialized) {
      throw new Error('Vector store not initialized');
    }

    try {
      // Since we don't have real embeddings, we'll use scroll to get all documents
      // and do keyword matching in memory
      const scrollResult = await this.client.scroll(this.collectionName, {
        limit: 1000, // Get up to 1000 documents
        with_payload: true,
        with_vector: false
      });

      const allDocs = scrollResult.points.map(point => ({
        id: point.id,
        content: point.payload.content,
        metadata: point.payload.metadata,
        createdAt: point.payload.createdAt
      }));

      // Apply keyword-based scoring
      return this._scoreDocuments(query, allDocs, k);
    } catch (error) {
      console.error('Error searching Qdrant:', error);
      throw error;
    }
  }

  /**
   * Score documents based on keyword matching
   */
  _scoreDocuments(query, documents, k) {
    const queryLower = query.toLowerCase();
    const queryWords = queryLower.split(/\s+/).filter(word => word.length > 3);

    const stopWords = ['what', 'how', 'why', 'when', 'where', 'which', 'does', 'the', 'and', 'for', 'with', 'this', 'that', 'from'];
    const meaningfulQueryWords = queryWords.filter(word => !stopWords.includes(word));

    const scoredDocs = documents.map(doc => {
      let score = 0;
      const contentLower = doc.content.toLowerCase();

      // Exact phrase match
      if (contentLower.includes(queryLower)) {
        score += 20;
      }

      // Meaningful word matches
      let wordMatchCount = 0;
      for (const word of meaningfulQueryWords) {
        if (contentLower.includes(word)) {
          wordMatchCount++;
          score += 2;
        }
      }

      // Match ratio bonus
      const matchRatio = meaningfulQueryWords.length > 0
        ? wordMatchCount / meaningfulQueryWords.length
        : 0;
      score += matchRatio * 10;

      // Metadata relevance
      if (doc.metadata) {
        const metadataText = `${doc.metadata.module || ''} ${doc.metadata.chapter || ''} ${doc.metadata.fileName || ''}`.toLowerCase();
        for (const word of meaningfulQueryWords) {
          if (metadataText.includes(word)) {
            score += 1.5;
          }
        }
      }

      // Document length optimization
      if (doc.content.length < 200) {
        score *= 0.7;
      }
      if (doc.content.length >= 500 && doc.content.length <= 2000) {
        score *= 1.2;
      }

      return { ...doc, score };
    });

    // Sort and return top k
    const results = scoredDocs
      .filter(doc => doc.score > 0)
      .sort((a, b) => b.score - a.score)
      .slice(0, k);

    // Normalize scores
    if (results.length > 0) {
      const maxScore = results[0].score;
      results.forEach(result => {
        result.score = maxScore > 0 ? result.score / maxScore : 0;
      });
    }

    return results;
  }

  /**
   * Retrieve relevant documents
   */
  async retrieveRelevantDocuments(query, options = {}) {
    const { maxResults = 5, filters = null } = options;

    let results = await this.similaritySearch(query, maxResults);

    // Apply filters if provided
    if (filters && Object.keys(filters).length > 0) {
      results = results.filter(doc => {
        for (const [key, value] of Object.entries(filters)) {
          if (doc.metadata && doc.metadata[key] !== value) {
            return false;
          }
        }
        return true;
      });
    }

    return {
      query,
      results: results.map(doc => ({
        id: doc.id,
        content: doc.content,
        metadata: doc.metadata,
        score: doc.score
      })),
      retrievedCount: results.length,
      totalAvailable: results.length
    };
  }

  /**
   * Check if ready
   */
  isReady() {
    return this.isInitialized && this.client !== null;
  }

  /**
   * Get stats
   */
  async getStats() {
    if (!this.isInitialized) {
      return null;
    }

    try {
      const collectionInfo = await this.client.getCollection(this.collectionName);
      
      return {
        type: 'qdrant',
        isConnected: true,
        initialized: this.isInitialized,
        config: {
          url: process.env.QDRANT_URL,
          collectionName: this.collectionName
        },
        documentCount: collectionInfo.points_count || 0
      };
    } catch (error) {
      console.error('Error getting stats:', error);
      return {
        type: 'qdrant
