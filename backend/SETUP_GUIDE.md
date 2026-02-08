# AI Assistant Setup Guide - Textbook Content Adherence

## ✅ Completed Fixes

1. **Query Handler** - Strict textbook-only responses
2. **Vector Store** - Enhanced search algorithm  
3. **Content Loading** - Path fixed, 304 chunks ready
4. **Response Validation** - Stricter grounding checks

## 🔧 Required Actions

### Step 1: Fix OpenAI API Key

**Current Issue:** Wrong API key type detected
```
Current: AIzaSyCwXJ71RA5_FsZ6UKK_7GR1o7_6xkwj7fs (Google API key)
Required: sk-... (OpenAI API key)
```

**Action Required:**
1. Get your OpenAI API key from: https://platform.openai.com/api-keys
2. Edit `backend/.env`
3. Replace the line:
   ```
   OPENAI_API_KEY=AIzaSyCwXJ71RA5_FsZ6UKK_7GR1o7_6xkwj7fs
   ```
   With:
   ```
   OPENAI_API_KEY=sk-your-actual-openai-key-here
   ```
4. Save the file

### Step 2: Restart Backend Server

**Why:** The running server has 0 documents. Restart loads all 304 chunks.

**Action Required:**
1. Go to the terminal running the backend server
2. Press `Ctrl+C` to stop it
3. Run: `npm run dev` (or `npm start`)
4. Wait for: "Textbook content loaded successfully"

**Expected Output:**
```
Physical AI RAG Service server running on port 8080
Vector store initialized successfully
Textbook content loaded successfully
Vector store stats after loading: { documentCount: 304 }
```

### Step 3: Verify Everything Works

Run the verification script:
```bash
node verify-setup.js
```

## 📊 Health Check Endpoints

After restart, check:
```bash
curl http://localhost:8080/health
```

**Expected Response:**
```json
{
  "status": "healthy",
  "dependencies": {
    "vectorStore": "connected",
    "llmProvider": "connected",  ← Should be "connected" after API key fix
    "contentIndex": "ready"       ← Should be "ready" after restart
  }
}
```

## 🧪 Test Queries

Once everything is running:

**In-Scope Query (should answer from textbook):**
```bash
curl -X POST http://localhost:8080/api/chat \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer dummy-key" \
  -d '{"query":"What is ROS 2?","sessionId":"test-123"}'
```

**Out-of-Scope Query (should refuse):**
```bash
curl -X POST http://localhost:8080/api/chat \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer dummy-key" \
  -d '{"query":"How do I cook pasta?","sessionId":"test-123"}'
```

## 📝 Summary of Changes

### query-handler.js
- Temperature: 0.7 → 0.2 (more factual)
- Added relevance threshold: 0.3
- Stricter system prompt (8 critical rules)
- Enhanced grounding validation (15% overlap required)

### vector-store.js
- Stop word filtering
- Multi-factor relevance scoring
- Metadata consideration
- Document length optimization

### load-textbook-content.js
- Fixed path to textbook docs
- Now correctly loads from: `book-ai/my-website/docs`

## 🎯 Expected Behavior

### ✅ Assistant WILL:
- Answer ONLY from textbook content
- Quote/paraphrase directly from textbook
- Reference specific chapters/modules
- Clearly state when info is not in textbook
- Admit partial coverage when applicable

### ❌ Assistant WILL NOT:
- Add external knowledge
- Make assumptions beyond textbook
- Provide creative examples not in textbook
- Answer off-topic questions with general knowledge

## 🐛 Troubleshooting

**Problem:** "documentCount: 0" after restart
- **Solution:** Check server logs for path errors
- **Verify:** Path should be `D:\book-ai\my-website\docs`

**Problem:** "llmProvider: disconnected"
- **Solution:** Fix OpenAI API key in `.env`
- **Verify:** Key should start with `sk-`

**Problem:** Empty responses
- **Solution:** Ensure content is loaded (documentCount: 304)
- **Verify:** Run `/api/retrieve` endpoint test

## 📞 Support

If issues persist:
1. Check server logs for errors
2. Verify all 28 markdown files exist in `my-website/docs`
3. Ensure OpenAI API key has credits
4. Test with simple queries first
