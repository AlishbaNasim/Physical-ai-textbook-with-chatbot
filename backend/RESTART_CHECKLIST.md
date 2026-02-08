# 🔄 Server Restart Checklist

## ✅ Pre-Restart Verification (COMPLETED)

- [x] Qdrant Cloud connection tested
- [x] OpenAI API key configured
- [x] Content loading path fixed
- [x] Query handler updated (strict mode)
- [x] Vector store enhanced (Qdrant integration)
- [x] All code syntax verified

## 📋 Restart Steps

### Step 1: Stop Current Server
In the terminal where backend is running:
```
Press: Ctrl + C
```

### Step 2: Start Server with New Configuration
```bash
cd backend
npm run dev
```

### Step 3: Watch for These Success Messages
```
✓ Physical AI RAG Service server running on port 8080
✓ Initializing Qdrant vector store...
✓ Connected to Qdrant Cloud successfully
✓ Collection textbook-content exists
✓ Vector store initialized successfully
✓ Processing textbook content from: D:\book-ai\my-website\docs
✓ Found 28 markdown files to process
✓ Textbook content loaded successfully
✓ Vector store stats after loading: { documentCount: 304 }
```

### Step 4: Verify Health (In New Terminal)
```bash
curl http://localhost:8080/health
```

**Expected Response:**
```json
{
  "status": "healthy",
  "dependencies": {
    "vectorStore": "connected",
    "llmProvider": "connected",
    "contentIndex": "ready"
  }
}
```

### Step 5: Test Chat Endpoint
```bash
curl -X POST http://localhost:8080/api/chat \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer dummy-key" \
  -d "{\"query\":\"What is ROS 2?\",\"sessionId\":\"test-123\"}"
```

**Expected:** JSON response with textbook answer and sources

## ✅ Success Indicators

- [ ] Server starts without errors
- [ ] Qdrant connection successful
- [ ] 304 documents loaded
- [ ] Health check shows all "connected"
- [ ] Chat endpoint returns textbook answer
- [ ] Response includes sources from textbook

## 🐛 If Something Goes Wrong

### Error: "Cannot connect to Qdrant"
- Check internet connection
- Verify Qdrant URL in .env
- Check Qdrant API key

### Error: "OpenAI API error"
- Verify OpenAI API key in .env
- Check API key has credits
- Ensure key starts with "sk-proj-"

### Error: "No markdown files found"
- Verify path: D:\book-ai\my-website\docs
- Check that docs folder exists
- Ensure markdown files are present

## 📞 Need Help?

If you see any errors during restart, share the error message and I'll help fix it!
