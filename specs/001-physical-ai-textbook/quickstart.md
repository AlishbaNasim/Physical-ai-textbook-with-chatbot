# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

## Overview
This guide provides a quick introduction to setting up and getting started with the Physical AI & Humanoid Robotics textbook project, including both the documentation site and the RAG-powered chatbot.

## Prerequisites

- Node.js 20.x LTS or higher
- npm or yarn package manager
- Python 3.11+ (for RAG processing)
- Git version control system
- Text editor or IDE (VS Code recommended)

## Environment Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd book-ai
```

### 2. Set Up the Documentation Website (Docusaurus)

Navigate to the documentation directory:
```bash
cd my-website
```

Install dependencies:
```bash
npm install
# or
yarn install
```

### 3. Set Up the RAG Backend Service

Navigate to the backend directory:
```bash
cd ../backend  # assuming backend directory exists alongside my-website
```

Install backend dependencies:
```bash
npm install
# or
pip install -r requirements.txt  # if using Python backend
```

### 4. Configure Environment Variables

Create a `.env` file in both the `my-website` and `backend` directories:

**For my-website/.env:**
```env
# Frontend configuration
CHATBOT_API_URL=http://localhost:8080/api
```

**For backend/.env:**
```env
# Backend configuration
PORT=8080
LLM_PROVIDER=openai  # or ollama/local for privacy
OPENAI_API_KEY=your_openai_api_key_here
VECTOR_STORE=pinecone  # or chromadb for local development
PINECONE_API_KEY=your_pinecone_api_key
PINECONE_ENVIRONMENT=gcp-starter
PINECONE_INDEX=textbook-content
```

## Running the Application

### 1. Start the RAG Backend Service

From the backend directory:
```bash
npm start
# or
node server.js
```

The backend service will start on `http://localhost:8080`.

### 2. Start the Documentation Website

From the my-website directory:
```bash
npm start
```

The documentation website will start on `http://localhost:3000`.

## Content Development Workflow

### 1. Adding New Modules

Create a new directory in `my-website/docs/`:
```bash
mkdir my-website/docs/module-5-advanced-topics
```

### 2. Adding New Chapters

Within a module directory, create new markdown files:
```bash
touch my-website/docs/module-5-advanced-topics/chapter-1-new-concepts.md
```

### 3. Content Format Requirements

Each chapter file should follow this structure:

```markdown
---
title: Chapter Title
sidebar_label: Chapter Label
description: Brief description of the chapter content
keywords: [list, of, relevant, keywords]
---

import Component from '@site/src/components/Component';

# Chapter Title

## Learning Objectives

- Objective 1
- Objective 2
- Objective 3

## Main Content

Your chapter content goes here...

## Key Diagrams

![Diagram Title](/img/diagram-path.png)

## Summary

Summary of the chapter content...

## Further Reading

- [Related Resource 1](link)
- [Related Resource 2](link)
```

### 4. Processing Content for RAG

To update the RAG system with new content:

```bash
cd backend
npm run process-content
```

This command will:
- Parse all markdown files in the documentation
- Extract text content and diagrams descriptions
- Create vector embeddings for retrieval
- Store in the configured vector database

## Using the RAG Chatbot

The chatbot widget is integrated into the documentation pages. Users can:

1. Click on the floating chatbot icon
2. Type questions about the textbook content
3. Receive answers based only on the textbook content
4. Navigate to relevant sections when referenced

### Chatbot API Endpoints

The backend provides the following API endpoints:

- `POST /api/chat` - Submit a query and receive a response
- `GET /api/health` - Check the health of the RAG service
- `POST /api/validate-query` - Validate if a query is within textbook scope

Example API call:
```javascript
fetch('http://localhost:8080/api/chat', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
  },
  body: JSON.stringify({
    query: 'What is the role of ROS 2 in humanoid robotics?',
    sessionId: 'unique-session-id'
  })
})
.then(response => response.json())
.then(data => console.log(data));
```

## Building for Production

### 1. Build the Documentation Site

```bash
cd my-website
npm run build
```

### 2. Deploy Configuration

The built site will be in the `build/` directory and can be served as a static site.

## Testing

### Frontend Tests
```bash
cd my-website
npm test
```

### Backend Tests
```bash
cd backend
npm test
```

## Troubleshooting

### Common Issues

1. **Port Already in Use**: Change the port in the configuration files
2. **Missing Dependencies**: Run `npm install` again
3. **Content Not Appearing in Chatbot**: Run the content processing script again
4. **Slow Chatbot Response**: Check vector database connection and API keys

### API Keys Not Working

- Verify API keys are correctly entered in the `.env` file
- Check that the services (OpenAI, Pinecone, etc.) are accessible
- Ensure the API keys have the required permissions

## Next Steps

1. Begin adding content to the documentation structure
2. Customize the Docusaurus theme to match design requirements
3. Expand the RAG system with additional content processing capabilities
4. Add interactive components for enhanced learning experience