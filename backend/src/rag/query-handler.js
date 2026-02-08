/**
 * Query Handler for Physical AI & Humanoid Robotics RAG System
 * Processes user queries and generates responses based on textbook content
 */

const vectorStoreService = require('./vector-store');
const DocumentProcessor = require('./document-processor');
const OpenAI = require('openai');

class QueryHandler {
  constructor(options = {}) {
    this.vectorStore = vectorStoreService;
    this.documentProcessor = new DocumentProcessor();
    this.maxResults = options.maxResults || 5;
    this.responseTokenLimit = options.responseTokenLimit || 1000;

    // Initialize OpenAI client
    this.openai = new OpenAI({
      apiKey: process.env.OPENAI_API_KEY
    });

    // System prompt for the textbook assistant
    this.systemPrompt = `You are a strict AI teaching assistant for the "Physical AI & Humanoid Robotics" textbook. Your ONLY role is to answer questions using EXCLUSIVELY the textbook content provided to you.

CRITICAL RULES - YOU MUST FOLLOW THESE STRICTLY:
1. NEVER use information from outside the provided textbook content
2. NEVER add your own knowledge, examples, or explanations beyond what's in the textbook
3. ONLY answer based on the exact textbook passages provided in the context
4. If the provided textbook content doesn't contain the answer, you MUST say: "I cannot find information about this in the provided textbook content."
5. Quote or paraphrase directly from the textbook content when answering
6. If the question is partially covered, explain ONLY what the textbook covers and acknowledge what it doesn't
7. Do NOT make assumptions or inferences beyond what's explicitly stated in the textbook
8. Do NOT provide general knowledge about robotics, AI, or related topics - ONLY textbook content

RESPONSE FORMAT:
- Start by directly addressing the question using textbook content
- Use clear, educational language
- Reference which section/chapter the information comes from when possible
- If insufficient information: clearly state what IS covered in the textbook

Remember: You are NOT a general AI assistant. You are a textbook-only assistant. Accuracy to the source material is more important than being helpful with external knowledge.`;
  }

  /**
   * Process a user query and return a response based on textbook content
   */
  async processQuery(query, context = {}) {
    try {
      // Validate inputs
      if (!query || typeof query !== 'string' || query.trim().length === 0) {
        throw new Error('Query is required and must be a non-empty string');
      }

      // Ensure vector store is initialized
      if (!this.vectorStore.isReady()) {
        await this.vectorStore.initialize();
      }

      // Retrieve relevant documents from the vector store
      const retrievalResult = await this.retrieveContent(query, context);

      // Generate response based on retrieved content
      const response = await this.generateResponse(query, retrievalResult.results, context);

      // Validate that the response is grounded in textbook content
      const groundingValidation = this.validateResponseGrounding(
        response.text,
        query,
        retrievalResult.results
      );

      return {
        query,
        response: response.text,
        sources: response.sources,
        isGrounded: groundingValidation.isGrounded,
        confidenceScore: groundingValidation.confidenceScore,
        groundingExplanation: groundingValidation.explanation,
        timestamp: new Date().toISOString()
      };
    } catch (error) {
      console.error('Query processing error:', error);
      throw error;
    }
  }

  /**
   * Retrieve relevant content for a query
   */
  async retrieveContent(query, context = {}) {
    const retrievalOptions = {
      maxResults: this.maxResults,
      filters: context.filters || null
    };

    // If context specifies a particular module, filter to that module
    if (context.currentModule) {
      retrievalOptions.filters = {
        ...retrievalOptions.filters,
        module: context.currentModule
      };
    }

    return await this.vectorStore.retrieveRelevantDocuments(query, retrievalOptions);
  }

  /**
   * Generate a response based on query and retrieved content
   */
  async generateResponse(query, results, context) {
    // Check if we have any results
    if (!results || results.length === 0) {
      return {
        text: `I cannot find information about "${query}" in the textbook content. Please ensure your question relates to topics covered in the Physical AI & Humanoid Robotics textbook, such as embodied AI, ROS 2, simulation systems, or Vision-Language-Action systems.`,
        sources: [],
        isGrounded: false,
        confidenceScore: 0.0
      };
    }

    // Filter out low-relevance results (score < 0.3)
    const relevantResults = results.filter(result => (result.score || 0) >= 0.3);

    if (relevantResults.length === 0) {
      return {
        text: `I cannot find sufficiently relevant information about "${query}" in the textbook content. The retrieved content does not closely match your question. Please try rephrasing or asking about a different topic covered in the textbook.`,
        sources: [],
        isGrounded: false,
        confidenceScore: 0.2
      };
    }

    // Create source information from relevant results only
    const sources = relevantResults.map(result => ({
      chapterId: result.metadata?.source || 'unknown',
      chapterTitle: result.metadata?.fileName || 'Unknown Chapter',
      module: result.metadata?.module || 'unknown',
      section: result.metadata?.section || 'General',
      relevanceScore: result.score || 0.5
    }));

    // Build context from retrieved documents
    const textbookContext = relevantResults.map((result, index) => {
      return `[Source ${index + 1}: ${result.metadata?.fileName || 'Unknown'} - ${result.metadata?.module || 'Unknown Module'}]\n${result.content}`;
    }).join('\n\n---\n\n');

    try {
      // Generate response using OpenAI with strict parameters
      const completion = await this.openai.chat.completions.create({
        model: 'gpt-4o-mini',
        messages: [
          {
            role: 'system',
            content: this.systemPrompt
          },
          {
            role: 'user',
            content: `Here is the ONLY content you may use to answer the question:\n\n${textbookContext}\n\n---\n\nStudent Question: ${query}\n\nIMPORTANT: Answer ONLY using the textbook content provided above. Do NOT add any information from your general knowledge. If the textbook content above doesn't contain the answer, you MUST say so clearly.`
          }
        ],
        temperature: 0.2,  // Low temperature for more deterministic, factual responses
        max_tokens: 800,
        top_p: 0.9,  // Slightly reduced for more focused responses
        frequency_penalty: 0,
        presence_penalty: 0
      });

      const responseText = completion.choices[0]?.message?.content || 'I apologize, but I was unable to generate a response. Please try again.';

      // Additional validation: check if response indicates lack of information
      const lowerResponse = responseText.toLowerCase();
      const lackOfInfoPhrases = [
        'cannot find', 'not in the textbook', 'not covered',
        'does not contain', 'no information', 'not mentioned',
        'textbook does not', 'not available in the textbook'
      ];

      const indicatesLackOfInfo = lackOfInfoPhrases.some(phrase => lowerResponse.includes(phrase));

      return {
        text: responseText,
        sources,
        isGrounded: !indicatesLackOfInfo,
        confidenceScore: indicatesLackOfInfo ? 0.3 : (relevantResults[0]?.score || 0.7)
      };
    } catch (error) {
      console.error('OpenAI API error:', error);

      // Fallback to basic response if OpenAI fails
      const fallbackText = `I encountered an error generating a response. Here is the relevant textbook content:\n\n${relevantResults[0].content.substring(0, 400)}...\n\nSource: ${sources[0].chapterTitle} (${sources[0].module})`;

      return {
        text: fallbackText,
        sources,
        isGrounded: true,
        confidenceScore: 0.5
      };
    }
  }

  /**
   * Process a validation query
   */
  async processValidationQuery(query) {
    return this.validateQueryScope(query);
  }

  /**
   * Validate that a response is grounded in textbook content
   */
  validateResponseGrounding(response, query, retrievedContent) {
    // Check if the response references content that exists in retrieved sources
    const responseText = response.toLowerCase();

    // First check: if response admits lack of information, it's not grounded
    const lackOfInfoPhrases = [
      'cannot find', 'not in the textbook', 'couldn\'t find', 'not covered',
      'not mentioned', 'no information', 'not specified', 'unclear',
      'does not contain', 'textbook does not', 'not available in the textbook',
      'i don\'t know', 'insufficient information'
    ];

    for (const phrase of lackOfInfoPhrases) {
      if (responseText.includes(phrase)) {
        return {
          isGrounded: false,
          confidenceScore: 0.2,
          explanation: `Response indicates lack of textbook information: "${phrase}"`
        };
      }
    }

    // Look for evidence that the response is based on retrieved content
    let hasSupportingEvidence = false;
    let groundingScore = 0.0;

    if (retrievedContent && retrievedContent.length > 0) {
      // Filter to only relevant content (score >= 0.3)
      const relevantContent = retrievedContent.filter(c => (c.score || 0) >= 0.3);

      if (relevantContent.length === 0) {
        return {
          isGrounded: false,
          confidenceScore: 0.2,
          explanation: 'No sufficiently relevant textbook content found'
        };
      }

      // Check if response contains fragments or concepts from retrieved content
      for (const content of relevantContent) {
        const contentText = content.content.toLowerCase();

        // Count overlapping meaningful terms between response and content
        const responseWords = responseText.split(/\W+/).filter(w => w.length > 4);
        const contentWords = contentText.split(/\W+/).filter(w => w.length > 4);

        let overlapCount = 0;
        for (const word of responseWords) {
          if (contentWords.includes(word)) {
            overlapCount++;
          }
        }

        // Calculate overlap ratio - need at least 15% overlap for grounding
        const overlapRatio = overlapCount / Math.max(responseWords.length, 1);
        if (overlapRatio > 0.15) {
          hasSupportingEvidence = true;
          groundingScore = Math.max(groundingScore, overlapRatio);
        }
      }
    }

    return {
      isGrounded: hasSupportingEvidence,
      confidenceScore: hasSupportingEvidence ? Math.min(groundingScore * 2 + 0.3, 0.95) : 0.2,
      explanation: hasSupportingEvidence
        ? `Response grounded in textbook content (${Math.round(groundingScore * 100)}% term overlap)`
        : 'Insufficient overlap with retrieved textbook content'
    };
  }

  /**
   * Validate if a query is within the scope of the textbook
   */
  validateQueryScope(query) {
    if (!query || typeof query !== 'string') {
      return {
        isWithinScope: false,
        explanation: 'Query must be a non-empty string'
      };
    }

    const textbookTopics = [
      'physical ai', 'robotics', 'robot', 'ros', 'ros 2', 'humanoid', 'embodied ai',
      'simulation', 'gazebo', 'unity', 'isaac', 'vision language action', 'vla',
      'conversational robotics', 'large language models', 'ai', 'machine learning',
      'computer vision', 'robot control', 'navigation', 'manipulation', 'perception',
      'control systems', 'motion planning', 'sensor fusion', 'robot hardware',
      'kinematics', 'dynamics', 'path planning', 'behavior trees', 'state machines'
    ];

    const lowerQuery = query.toLowerCase();
    const isRelevant = textbookTopics.some(topic => lowerQuery.includes(topic));

    if (isRelevant) {
      return {
        isWithinScope: true,
        explanation: 'Query is within the scope of Physical AI & Humanoid Robotics textbook content',
        suggestions: []
      };
    } else {
      const suggestions = this.generateSuggestions(query);
      return {
        isWithinScope: false,
        explanation: 'This query is outside the scope of the Physical AI & Humanoid Robotics textbook content.',
        suggestions
      };
    }
  }

  /**
   * Generate suggestions for out-of-scope queries
   */
  generateSuggestions(query) {
    return [
      {
        suggestion: "Explain the basics of ROS 2 in robotics",
        relevance: "high"
      },
      {
        suggestion: "How does simulation-first development work?",
        relevance: "high"
      },
      {
        suggestion: "What are Vision-Language-Action systems?",
        relevance: "medium"
      }
    ];
  }

  /**
   * Process a validation query
   */
  async processValidationQuery(query) {
    return this.validateQueryScope(query);
  }
}

module.exports = QueryHandler;