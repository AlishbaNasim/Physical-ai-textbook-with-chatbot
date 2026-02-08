/**
 * Vector Store Service for Physical AI & Humanoid Robotics RAG System
 * Handles vector database operations for content retrieval
 */

require('dotenv').config();

class VectorStoreService {
  constructor() {
    this.vectorStore = null;
    this.isInitialized = false;
    this.storeType = process.env.VECTOR_STORE || 'chromadb';
    this.documents = []; // In-memory storage for fallback
    this.qdrantClient = null;
    this.collectionName = 'textbook-content';
  }

  /**
   * Initialize the vector store connection
   */
  async initialize() {
    try {
      switch (this.storeType.toLowerCase()) {
        case 'qdrant':
          await this._initializeQdrant();
          break;
        case 'pinecone':
          await this._initializePinecone();
          break;
        case 'chromadb':
        default:
          await this._initializeChromaDB();
          break;
      }

      this.isInitialized = true;
      console.log(`${this.storeType} vector store initialized successfully`);
    } catch (error) {
      console.error('Failed to initialize vector store:', error);
      throw error;
    }
  }

  /**
   * Initialize Qdrant vector store with real client
   */
  async _initializeQdrant() {
    console.log('Initializing Qdrant vector store...');

    try {
      const { QdrantClient } = require('@qdrant/js-client-rest');

      const qdrantUrl = process.env.QDRANT_URL;
      const qdrantApiKey = process.env.QDRANT_API_KEY;

      if (!qdrantUrl || !qdrantApiKey || qdrantUrl.includes('your_qdrant')) {
        console.warn('Qdrant credentials not configured, using in-memory fallback');
        this.vectorStore = {
          type: 'qdrant-fallback',
          isConnected: true,
          config: { collectionName: this.collectionName }
        };
        return;
      }

      // Initialize real Qdrant client
      this.qdrantClient = new QdrantClient({
        url: qdrantUrl,
        apiKey: qdrantApiKey,
      });

      // Test connection and ensure collection exists
      await this._ensureQdrantCollection();

      this.vectorStore = {
        type: 'qdrant',
        isConnected: true,
        config: {
          url: qdrantUrl,
          collectionName: this.collectionName
        }
      };

      console.log('✓ Connected to Qdrant Cloud successfully');
    } catch (error) {
      console.error('Qdrant initialization error:', error.message);
      console.warn('Falling back to in-memory storage');
      this.vectorStore = {
        type: 'qdrant-fallback',
        isConnected: true,
        config: { collectionName: this.collectionName }
      };
    }
  }

  /**
   * Ensure Qdrant collection exists
   */
  async _ensureQdrantCollection() {
    try {
      const collections = await this.qdrantClient.getCollections();
      const exists = collections.collections.some(
        col => col.name === this.collectionName
      );

      if (!exists) {
        console.log(`Creating Qdrant collection: ${this.collectionName}`);
        await this.qdrantClient.createCollection(this.collectionName, {
          vectors: {
            size: 1536, // OpenAI embedding dimension
            distance: 'Cosine'
          }
        });
        console.log('✓ Collection created');
      } else {
        console.log(`✓ Collection ${this.collectionName} exists`);
      }
    } catch (error) {
      console.error('Error ensuring collection:', error);
      throw error;
    }
  }

  /**
   * Initialize Pinecone vector store
   */
  async _initializePinecone() {
    // Placeholder for Pinecone initialization
    console.log('Initializing Pinecone vector store...');

    // For now, we'll just simulate initialization
    this.vectorStore = {
      type: 'pinecone',
      isConnected: true,
      config: {
        environment: process.env.PINECONE_ENVIRONMENT || 'gcp-starter',
        index: process.env.PINECONE_INDEX || 'textbook-content'
      }
    };
  }

  /**
   * Initialize ChromaDB vector store
   */
  async _initializeChromaDB() {
    // Placeholder for ChromaDB initialization
    console.log('Initializing ChromaDB vector store...');

    // For now, we'll just simulate initialization
    this.vectorStore = {
      type: 'chromadb',
      isConnected: true,
      config: {
        collectionName: 'textbook-content'
      }
    };
  }

  /**
   * Add document chunks to the vector store
   */
  async addDocuments(documents) {
    if (!this.isInitialized) {
      throw new Error('Vector store not initialized');
    }

    // If using real Qdrant, store in cloud
    if (this.qdrantClient && this.vectorStore.type === 'qdrant') {
      try {
        const points = documents.map((doc, index) => ({
          id: doc.id || `doc-${Date.now()}-${index}`,
          vector: Array(1536).fill(0), // Dummy vector until we have embeddings
          payload: {
            content: doc.content,
            metadata: doc.metadata || {},
            createdAt: new Date().toISOString()
          }
        }));

        await this.qdrantClient.upsert(this.collectionName, {
          wait: true,
          points: points
        });

        console.log(`Added ${documents.length} documents to Qdrant Cloud`);

        return {
          success: true,
          documentIds: points.map(p => p.id),
          count: documents.length
        };
      } catch (error) {
        console.error('Error adding to Qdrant, falling back to memory:', error.message);
        // Fall through to in-memory storage
      }
    }

    // Fallback: in-memory storage
    for (const doc of documents) {
      this.documents.push({
        id: doc.id,
        content: doc.content,
        metadata: doc.metadata,
        createdAt: new Date().toISOString()
      });
    }

    console.log(`Added ${documents.length} documents to in-memory store. Total: ${this.documents.length}`);

    return {
      success: true,
      documentIds: documents.map(doc => doc.id),
      count: documents.length
    };
  }

  /**
   * Enhanced text-based similarity search with better relevance scoring
   * Works with both Qdrant and in-memory storage
   */
  async similaritySearch(query, k = 5) {
    if (!this.isInitialized) {
      throw new Error('Vector store not initialized');
    }

    let allDocs = [];

    // Try to get documents from Qdrant first
    if (this.qdrantClient && this.vectorStore.type === 'qdrant') {
      try {
        const scrollResult = await this.qdrantClient.scroll(this.collectionName, {
          limit: 1000,
          with_payload: true,
          with_vector: false
        });

        allDocs = scrollResult.points.map(point => ({
          id: point.id,
          content: point.payload.content,
          metadata: point.payload.metadata,
          createdAt: point.payload.createdAt
        }));

        console.log(`Retrieved ${allDocs.length} documents from Qdrant for search`);
      } catch (error) {
        console.error('Error retrieving from Qdrant:', error.message);
        // Fall back to in-memory
        allDocs = this.documents;
      }
    } else {
      // Use in-memory documents
      allDocs = this.documents;
    }

    // Enhanced keyword-based search
    const queryLower = query.toLowerCase();
    const queryWords = queryLower.split(/\s+/).filter(word => word.length > 3);

    // Remove common stop words for better matching
    const stopWords = ['what', 'how', 'why', 'when', 'where', 'which', 'does', 'the', 'and', 'for', 'with', 'this', 'that', 'from'];
    const meaningfulQueryWords = queryWords.filter(word => !stopWords.includes(word));

    // Score documents based on keyword matches
    const scoredDocs = allDocs.map(doc => {
      let score = 0;
      const contentLower = doc.content.toLowerCase();

      // 1. Exact phrase match (highest priority)
      if (contentLower.includes(queryLower)) {
        score += 20;
      }

      // 2. Check for meaningful word matches
      let wordMatchCount = 0;
      for (const word of meaningfulQueryWords) {
        if (contentLower.includes(word)) {
          wordMatchCount++;
          score += 2; // Boost for each meaningful word match
        }
      }

      // 3. Bonus for high percentage of query words found
      const matchRatio = meaningfulQueryWords.length > 0
        ? wordMatchCount / meaningfulQueryWords.length
        : 0;
      score += matchRatio * 10;

      // 4. Check metadata relevance (module, chapter names)
      if (doc.metadata) {
        const metadataText = `${doc.metadata.module || ''} ${doc.metadata.chapter || ''} ${doc.metadata.fileName || ''}`.toLowerCase();
        for (const word of meaningfulQueryWords) {
          if (metadataText.includes(word)) {
            score += 1.5; // Metadata matches are valuable
          }
        }
      }

      // 5. Penalize very short documents (likely not comprehensive)
      if (doc.content.length < 200) {
        score *= 0.7;
      }

      // 6. Slight boost for medium-length documents (sweet spot)
      if (doc.content.length >= 500 && doc.content.length <= 2000) {
        score *= 1.2;
      }

      return {
        ...doc,
        score: score
      };
    });

    // Sort by score and return top k results
    const results = scoredDocs
      .filter(doc => doc.score > 0) // Only return documents with positive scores
      .sort((a, b) => b.score - a.score)
      .slice(0, k);

    // Normalize scores to 0-1 range
    if (results.length > 0) {
      const maxScore = results[0].score;
      results.forEach(result => {
        result.score = maxScore > 0 ? result.score / maxScore : 0;
      });
    }

    return results;
  }

  /**
   * Retrieve relevant documents for a query
   */
  async retrieveRelevantDocuments(query, options = {}) {
    const {
      maxResults = 5,
      filters = null
    } = options;

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

    // Format results to match expected structure
    const formattedResults = results.map(doc => ({
      id: doc.id,
      content: doc.content,
      metadata: doc.metadata,
      score: doc.score
    }));

    return {
      query,
      results: formattedResults,
      retrievedCount: formattedResults.length,
      totalAvailable: this.documents.length
    };
  }

  /**
   * Check if the vector store is connected and ready
   */
  isReady() {
    return this.isInitialized && this.vectorStore && this.vectorStore.isConnected;
  }

  /**
   * Get vector store statistics
   */
  async getStats() {
    if (!this.isInitialized) {
      return null;
    }

    // Try to get stats from Qdrant
    if (this.qdrantClient && this.vectorStore.type === 'qdrant') {
      try {
        const collectionInfo = await this.qdrantClient.getCollection(this.collectionName);

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
        console.error('Error getting Qdrant stats:', error.message);
        // Fall through to in-memory stats
      }
    }

    // Return in-memory stats
    return {
      type: this.vectorStore.type,
      isConnected: this.vectorStore.isConnected,
      initialized: this.isInitialized,
      config: this.vectorStore.config,
      documentCount: this.documents.length
    };
  }

  /**
   * Clear all documents from the vector store
   */
  async clear() {
    // Clear Qdrant if connected
    if (this.qdrantClient && this.vectorStore.type === 'qdrant') {
      try {
        await this.qdrantClient.deleteCollection(this.collectionName);
        await this._ensureQdrantCollection();
        console.log('Qdrant collection cleared and recreated');
      } catch (error) {
        console.error('Error clearing Qdrant:', error.message);
      }
    }

    // Also clear in-memory storage
    this.documents = [];
    console.log('In-memory vector store cleared');
  }

  /**
   * Get all documents (for debugging purposes)
   */
  async getAllDocuments() {
    // Try Qdrant first
    if (this.qdrantClient && this.vectorStore.type === 'qdrant') {
      try {
        const scrollResult = await this.qdrantClient.scroll(this.collectionName, {
          limit: 10000,
          with_payload: true,
          with_vector: false
        });

        return scrollResult.points.map(point => ({
          id: point.id,
          content: point.payload.content,
          metadata: point.payload.metadata
        }));
      } catch (error) {
        console.error('Error getting documents from Qdrant:', error.message);
      }
    }

    // Return in-memory documents
    return this.documents;
  }
}

// Export singleton instance
const vectorStoreService = new VectorStoreService();
module.exports = vectorStoreService;