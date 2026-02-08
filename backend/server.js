const express = require('express');
const cors = require('cors');
const helmet = require('helmet');
require('dotenv').config();

// Import routes
const chatRouter = require('./src/api/chat-router');

// Initialize services
let vectorStoreService;
try {
  vectorStoreService = require('./src/rag/vector-store');
} catch (error) {
  console.warn('Vector store service not available:', error.message);
  vectorStoreService = null;
}

const app = express();
const PORT = process.env.PORT || 8080;

// Middleware
app.use(helmet()); // Security headers
app.use(cors());
app.use(express.json({ limit: '10mb' }));
app.use(express.urlencoded({ extended: true }));

// Log requests
app.use((req, res, next) => {
  console.log(`${new Date().toISOString()} - ${req.method} ${req.path}`);
  next();
});

// Basic route
app.get('/', (req, res) => {
  res.json({
    message: 'Physical AI & Humanoid Robotics RAG Service',
    version: '1.0.0',
    status: 'operational'
  });
});

// Health check endpoint
app.get('/health', async (req, res) => {
  try {
    const dependencies = {
      vectorStore: 'disconnected',
      llmProvider: 'disconnected',
      contentIndex: 'not-ready'
    };

    // Check vector store status if available
    if (vectorStoreService && typeof vectorStoreService.isReady === 'function') {
      dependencies.vectorStore = vectorStoreService.isReady() ? 'connected' : 'disconnected';
    }

    // In a real implementation, we would also check LLM provider status

    res.json({
      status: 'healthy',
      timestamp: new Date().toISOString(),
      dependencies
    });
  } catch (error) {
    console.error('Health check error:', error);
    res.status(500).json({
      status: 'unhealthy',
      error: error.message,
      timestamp: new Date().toISOString()
    });
  }
});

// API routes
app.use('/api', chatRouter);

// 404 handler
app.use('*', (req, res) => {
  res.status(404).json({
    error: {
      code: 'NOT_FOUND',
      message: 'Endpoint not found'
    }
  });
});

// Error handling middleware
app.use((err, req, res, next) => {
  console.error(err.stack);
  res.status(500).json({
    error: {
      code: 'INTERNAL_ERROR',
      message: 'Something went wrong!'
    }
  });
});

// Graceful shutdown
process.on('SIGTERM', () => {
  console.log('SIGTERM received, shutting down gracefully');
  process.exit(0);
});

process.on('SIGINT', () => {
  console.log('SIGINT received, shutting down gracefully');
  process.exit(0);
});

// Import the content loading function
const { loadTextbookContent } = require('./src/rag/load-textbook-content');

// Start server
const server = app.listen(PORT, async () => {
  console.log(`Physical AI RAG Service server running on port ${PORT}`);
  console.log(`Health check available at http://localhost:${PORT}/health`);
  console.log(`API endpoints available at http://localhost:${PORT}/api`);

  // Initialize vector store and load textbook content
  if (vectorStoreService && typeof vectorStoreService.initialize === 'function') {
    try {
      await vectorStoreService.initialize();
      console.log('Vector store initialized successfully');

      // Load textbook content into the vector store
      await loadTextbookContent();
      console.log('Textbook content loaded successfully');

      // Check how many documents were loaded
      const stats = await vectorStoreService.getStats();
      console.log('Vector store stats after loading:', stats);
    } catch (error) {
      console.error('Failed to initialize vector store or load content:', error.message);
      console.error('Error stack:', error.stack);
    }
  } else {
    console.log('Vector store service not available');
  }
});

module.exports = server;