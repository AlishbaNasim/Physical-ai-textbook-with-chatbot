const express = require('express');
const rateLimit = require('express-rate-limit');
const router = express.Router();

// Import services
const vectorStoreService = require('../rag/vector-store');
const DocumentProcessor = require('../rag/document-processor');
const QueryHandler = require('../rag/query-handler');
const sessionManager = require('../utils/session-manager');

// Initialize query handler
const queryHandler = new QueryHandler();

// Rate limiting middleware - 100 requests per 15 minutes per IP
const limiter = rateLimit({
  windowMs: 15 * 60 * 1000, // 15 minutes
  max: 100, // Limit each IP to 100 requests per windowMs
  message: {
    error: {
      code: 'RATE_LIMIT_EXCEEDED',
      message: 'Too many requests from this IP, please try again later'
    }
  },
  standardHeaders: true, // Return rate limit info in the `RateLimit-*` headers
  legacyHeaders: false, // Disable the `X-RateLimit-*` headers
});

// Rate limiting specifically for chat endpoint - 20 queries per 15 minutes per IP
const chatLimiter = rateLimit({
  windowMs: 15 * 60 * 1000, // 15 minutes
  max: 20, // Limit each IP to 20 chat queries per windowMs
  message: {
    error: {
      code: 'CHAT_RATE_LIMIT_EXCEEDED',
      message: 'Too many chat requests from this IP, please try again later'
    }
  },
  standardHeaders: true,
  legacyHeaders: false,
});

// Placeholder middleware for authentication
const authenticateApiKey = (req, res, next) => {
  const apiKey = req.headers.authorization?.replace('Bearer ', '');

  if (!apiKey) {
    return res.status(401).json({
      error: {
        code: 'MISSING_API_KEY',
        message: 'API key is required in Authorization header'
      }
    });
  }

  // In a real implementation, you would validate the API key here
  // For now, we'll just pass through
  next();
};

// POST /chat - Submit a query to the RAG-powered chatbot
router.post('/chat', authenticateApiKey, chatLimiter, async (req, res) => {
  const { query, sessionId, context } = req.body;

  // Validate required fields
  if (!query) {
    return res.status(400).json({
      error: {
        code: 'MISSING_QUERY',
        message: 'Query is required in request body'
      }
    });
  }

  if (!sessionId) {
    return res.status(400).json({
      error: {
        code: 'MISSING_SESSION_ID',
        message: 'Session ID is required in request body'
      }
    });
  }

  try {
    // Create or retrieve session
    let session = sessionManager.getSession(sessionId);

    if (!session) {
      // Create new session if it doesn't exist
      session = sessionManager.createSession(sessionId, {
        userAgent: req.get('User-Agent'),
        ip: req.ip,
        createdAt: new Date().toISOString()
      });
    }

    // Add user query to session
    sessionManager.addMessageToSession(sessionId, {
      type: 'user_query',
      content: query,
      timestamp: new Date().toISOString()
    });

    // Process query using the query handler
    const result = await queryHandler.processQuery(query, context);

    // Add bot response to session
    sessionManager.addMessageToSession(sessionId, {
      type: 'bot_response',
      content: result.response,
      sources: result.sources,
      isGrounded: result.isGrounded,
      confidenceScore: result.confidenceScore,
      timestamp: new Date().toISOString()
    });

    res.json({
      id: `response-${Date.now()}`,
      query,
      response: result.response,
      sources: result.sources,
      isGrounded: result.isGrounded,
      confidenceScore: result.confidenceScore,
      groundingExplanation: result.groundingExplanation,
      sessionId: sessionId,
      timestamp: new Date().toISOString()
    });
  } catch (error) {
    console.error('Chat endpoint error:', error);
    res.status(500).json({
      error: {
        code: 'INTERNAL_ERROR',
        message: 'Error processing chat query'
      }
    });
  }
});

// POST /validate-query - Validate if a query is within the scope of the textbook content
router.post('/validate-query', authenticateApiKey, limiter, async (req, res) => {
  const { query, sessionId } = req.body;

  if (!query) {
    return res.status(400).json({
      error: {
        code: 'MISSING_QUERY',
        message: 'Query is required in request body'
      }
    });
  }

  try {
    // Validate query scope using the query handler
    const validation = queryHandler.validateQueryScope(query);

    // Create or update session if sessionId is provided
    if (sessionId) {
      let session = sessionManager.getSession(sessionId);

      if (!session) {
        session = sessionManager.createSession(sessionId, {
          userAgent: req.get('User-Agent'),
          ip: req.ip,
          createdAt: new Date().toISOString()
        });
      }

      // Add validation query to session
      sessionManager.addMessageToSession(sessionId, {
        type: 'validation_query',
        content: query,
        isValid: validation.isWithinScope,
        timestamp: new Date().toISOString()
      });
    }

    res.json({
      query,
      isWithinScope: validation.isWithinScope,
      explanation: validation.explanation,
      suggestions: validation.suggestions || [],
      timestamp: new Date().toISOString()
    });
  } catch (error) {
    console.error('Validate query endpoint error:', error);
    res.status(500).json({
      error: {
        code: 'INTERNAL_ERROR',
        message: 'Error validating query scope'
      }
    });
  }
});

// POST /retrieve - Retrieve relevant textbook content for a given query
router.post('/retrieve', authenticateApiKey, async (req, res) => {
  const { query, maxResults = 5, sessionId } = req.body;

  if (!query) {
    return res.status(400).json({
      error: {
        code: 'MISSING_QUERY',
        message: 'Query is required in request body'
      }
    });
  }

  try {
    // Validate that vector store is initialized
    if (!vectorStoreService.isReady()) {
      await vectorStoreService.initialize();
    }

    // Retrieve relevant documents from the vector store
    const retrievalResult = await vectorStoreService.retrieveRelevantDocuments(query, {
      maxResults: maxResults
    });

    res.json({
      query,
      results: retrievalResult.results.map(result => ({
        id: result.id,
        content: result.content,
        chapterId: result.metadata?.source || 'unknown',
        moduleId: result.metadata?.module || 'unknown',
        relevanceScore: result.score || 0.5,
        metadata: result.metadata || {}
      })),
      timestamp: new Date().toISOString()
    });
  } catch (error) {
    console.error('Retrieve endpoint error:', error);
    res.status(500).json({
      error: {
        code: 'INTERNAL_ERROR',
        message: 'Error retrieving content'
      }
    });
  }
});

// GET /session/:sessionId - Retrieve information about a specific chat session
router.get('/session/:sessionId', authenticateApiKey, (req, res) => {
  const { sessionId } = req.params;

  if (!sessionId) {
    return res.status(400).json({
      error: {
        code: 'MISSING_SESSION_ID',
        message: 'Session ID parameter is required'
      }
    });
  }

  const session = sessionManager.getSession(sessionId);

  if (!session) {
    return res.status(404).json({
      error: {
        code: 'SESSION_NOT_FOUND',
        message: 'Session not found or has expired'
      }
    });
  }

  res.json({
    sessionId: session.id,
    createdAt: session.createdAt,
    lastActivity: session.lastActivity,
    queryCount: session.queryCount,
    active: session.active,
    messageCount: session.messages.length,
    userData: session.userData
  });
});

// DELETE /session/:sessionId - End and clear a chat session
router.delete('/session/:sessionId', authenticateApiKey, (req, res) => {
  const { sessionId } = req.params;

  if (!sessionId) {
    return res.status(400).json({
      error: {
        code: 'MISSING_SESSION_ID',
        message: 'Session ID parameter is required'
      }
    });
  }

  const endedSession = sessionManager.endSession(sessionId);

  if (!endedSession) {
    return res.status(404).json({
      error: {
        code: 'SESSION_NOT_FOUND',
        message: 'Session not found or has already been ended'
      }
    });
  }

  res.json({
    sessionId: endedSession.id,
    status: 'ended',
    message: 'Session successfully ended and cleared',
    endDate: endedSession.endDate
  });
});


module.exports = router;