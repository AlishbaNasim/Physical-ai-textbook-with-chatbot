# Data Model: Physical AI & Humanoid Robotics Textbook

## Overview
This document defines the data models and structures for the Physical AI & Humanoid Robotics textbook project, including content organization, RAG system entities, and user interaction data.

## Core Entities

### 1. Textbook Content

**Module**
- `id`: Unique identifier for the module
- `title`: Module title (e.g., "Physical AI Foundations", "ROS 2 Nervous System")
- `description`: Brief overview of the module content
- `learningObjectives`: Array of learning objectives for the module
- `order`: Numeric ordering for navigation
- `chapters`: Array of chapter references
- `createdAt`: Creation timestamp
- `updatedAt`: Last update timestamp

**Chapter**
- `id`: Unique identifier for the chapter
- `moduleId`: Reference to parent module
- `title`: Chapter title
- `subtitle`: Optional subtitle for detailed identification
- `learningObjectives`: Array of specific learning objectives for this chapter
- `content`: Markdown/MDX content of the chapter
- `prerequisites`: Array of prerequisite concepts or chapters
- `relatedTopics`: Array of related topics or chapters
- `diagrams`: Array of diagram references in the chapter
- `order`: Chapter order within the module
- `wordCount`: Approximate word count
- `estimatedReadingTime`: In minutes
- `createdAt`: Creation timestamp
- `updatedAt`: Last update timestamp

**Diagram**
- `id`: Unique identifier for the diagram
- `chapterId`: Reference to the chapter containing this diagram
- `title`: Diagram title
- `description`: Textual description of the diagram content
- `type`: Type of diagram (e.g., "architecture", "workflow", "sequence")
- `sourcePath`: Path to the diagram source file
- `altText`: Accessibility text description
- `keyElements`: Array of important elements in the diagram with descriptions
- `relatedConcepts`: Array of concepts explained by this diagram
- `createdAt`: Creation timestamp
- `updatedAt`: Last update timestamp

### 2. RAG System Entities

**DocumentChunk**
- `id`: Unique identifier for the chunk
- `chapterId`: Reference to the chapter this chunk originates from
- `content`: The actual text content of the chunk
- `chunkOrder`: Order of this chunk within the chapter
- `chunkType`: Type of content (e.g., "text", "code", "diagram-description")
- `semanticTags`: Array of semantic tags for retrieval
- `embeddingVector`: Vector representation for similarity search
- `wordCount`: Number of words in the chunk
- `createdAt`: Creation timestamp
- `updatedAt`: Last update timestamp

**Query**
- `id`: Unique identifier for the query
- `sessionId`: Reference to the chat session
- `userQuery`: Original user query text
- `processedQuery`: Query after preprocessing and cleaning
- `queryIntent`: Detected intent of the query
- `timestamp`: When the query was submitted
- `userId`: Reference to the user (if available)

**RetrievedContext**
- `id`: Unique identifier for the retrieved context
- `queryId`: Reference to the query that triggered retrieval
- `chunks`: Array of retrieved document chunk IDs
- `relevanceScores`: Array of relevance scores for each chunk
- `contextText`: Combined text of all retrieved chunks
- `retrievalMethod`: Method used for retrieval (e.g., "semantic", "keyword")
- `timestamp`: When the context was retrieved

**Response**
- `id`: Unique identifier for the response
- `queryId`: Reference to the query
- `responseText`: The text response to the user
- `sourceChunks`: Array of chunk IDs used to generate the response
- `confidenceScore`: Confidence level of the response
- `timestamp`: When the response was generated
- `isGrounded`: Boolean indicating if response is grounded in textbook content

### 3. User Interaction Data

**ChatSession**
- `id`: Unique identifier for the session
- `userId`: Reference to the user (anonymous if not logged in)
- `startTime`: When the session started
- `endTime`: When the session ended (null if ongoing)
- `queries`: Array of query IDs in this session
- `sessionMetadata`: Additional metadata about the session

**LearningProgress**
- `id`: Unique identifier for the progress record
- `userId`: Reference to the user
- `chapterId`: Reference to the chapter
- `completionStatus`: Status (e.g., "not-started", "in-progress", "completed")
- `progressPercentage`: Numeric progress percentage
- `timeSpent`: Time spent in seconds
- `lastAccessed`: Last time the chapter was accessed
- `learningNotes`: User's notes on the chapter
- `quizResults`: Array of quiz results for this chapter

### 4. System Configuration

**SystemSettings**
- `id`: Unique identifier (singleton)
- `ragEnabled`: Whether RAG chatbot is enabled
- `contentChunkSize`: Size of content chunks for RAG (in tokens)
- `maxQueryLength`: Maximum allowed query length
- `responseTimeout`: Timeout for response generation in seconds
- `contentValidationEnabled`: Whether content validation is enabled
- `updatedAt`: Last configuration update

## Relationships

- Module 1-* Chapter (one module contains many chapters)
- Chapter 1-* Diagram (one chapter contains many diagrams)
- Chapter 1-* DocumentChunk (one chapter produces many document chunks)
- Query 1-* RetrievedContext (one query generates one context)
- Query 1-* Response (one query generates one response)
- ChatSession 1-* Query (one session contains many queries)
- User 1-* LearningProgress (one user has progress on many chapters)
- Chapter 1-* LearningProgress (one chapter has progress from many users)

## Validation Rules

### Module Validation
- Title must be between 5-100 characters
- Must have at least one learning objective
- Order must be a positive integer
- Description must be present and 10-500 characters

### Chapter Validation
- Title must be between 5-100 characters
- Content must be present and not empty
- Estimated reading time must be 5-60 minutes
- Learning objectives must be between 1-5 items

### DocumentChunk Validation
- Content must be between 100-1500 tokens
- Must be associated with a valid chapter
- Chunk order must be sequential within a chapter

### Query Validation
- User query must be between 5-500 characters
- Must not contain prohibited content
- Must be related to Physical AI or robotics topics

## State Transitions

### LearningProgress States
- `not-started` → `in-progress` → `completed`
- User accesses chapter → progresses to `in-progress` → finishes chapter → `completed`
- `completed` can revert to `in-progress` if user revisits

## Indexes for Performance

- Module: `order` field for efficient sorting
- Chapter: `moduleId` and `order` for efficient retrieval within modules
- DocumentChunk: `chapterId` for efficient chunk retrieval by chapter
- Query: `sessionId` and `timestamp` for session query retrieval
- LearningProgress: `userId` and `chapterId` for user progress tracking