# Chapter 14: RAG & AI Chatbot for Robotics

## Learning Objectives

After completing this chapter, you will be able to:

- **Understand** Retrieval-Augmented Generation (RAG) systems and knowledge bases
- **Build** intelligent chatbots that answer robotics questions using textbook content
- **Implement** vector databases for semantic search
- **Deploy** RAG systems that ground LLM responses in factual content

## Overview

Large Language Models hallucinate—they confidently generate plausible-sounding but incorrect information. Retrieval-Augmented Generation (RAG) solves this by grounding LLM responses in retrieved knowledge from a trusted source (like this textbook). This chapter covers building practical chatbot systems that answer robotics questions with citations.

## Why RAG for Robotics?

Roboticists need accurate, up-to-date information:
- **Safety-critical**: Wrong information about motor torque limits can damage robots
- **Evolution**: New frameworks (ROS 2), algorithms (RT-2), tools appear constantly
- **Specificity**: Questions are domain-specific, not general knowledge
- **Reproducibility**: Want to know *which paper* or *chapter* the answer comes from

RAG combines:
- **Retrieval**: Find relevant knowledge from a corpus
- **Augmentation**: Add retrieved content to LLM prompt
- **Generation**: LLM generates answer grounded in retrieved content

## Architecture

### System Components

```
User Question
    ↓
[Embedding Layer] → Convert question to vector
    ↓
[Vector Database] ← Query for top-K similar documents
    ↓
[Retrieved Context] → Most relevant chapters, code examples
    ↓
[LLM Prompt] ← Augment with retrieved context
    ↓
[Response Generation] → Answer with citations
    ↓
User sees: Answer + "Source: Chapter 7, Section 3.2"
```

### Vector Database Setup

```python
import chromadb
from sentence_transformers import SentenceTransformer

class RoboticsKnowledgeBase:
    def __init__(self):
        # Initialize embedding model
        self.embedder = SentenceTransformer('all-MiniLM-L6-v2')

        # Initialize vector database
        self.client = chromadb.Client()
        self.collection = self.client.create_collection(name="robotics_textbook")

    def ingest_chapters(self, chapters):
        """Add textbook content to knowledge base"""
        for chapter_id, chapter_content in chapters.items():
            # Split into semantic chunks (paragraphs, code blocks)
            chunks = self.split_into_chunks(chapter_content)

            for chunk_id, chunk_text in enumerate(chunks):
                # Embed chunk
                embedding = self.embedder.encode(chunk_text, normalize_embeddings=True)

                # Store in database
                self.collection.add(
                    ids=[f"{chapter_id}_chunk_{chunk_id}"],
                    embeddings=[embedding.tolist()],
                    documents=[chunk_text],
                    metadatas=[{
                        "chapter": chapter_id,
                        "chunk": chunk_id,
                        "source": f"Chapter {chapter_id}, Section {chunk_id}"
                    }]
                )

    def retrieve(self, query: str, top_k: int = 3) -> List[dict]:
        """Retrieve most relevant chunks for query"""
        # Embed query
        query_embedding = self.embedder.encode(query, normalize_embeddings=True)

        # Search vector database
        results = self.collection.query(
            query_embeddings=[query_embedding.tolist()],
            n_results=top_k
        )

        # Format results
        retrieved = []
        for doc, metadata in zip(results['documents'][0], results['metadatas'][0]):
            retrieved.append({
                'content': doc,
                'source': metadata['source']
            })

        return retrieved

    @staticmethod
    def split_into_chunks(text, chunk_size=300, overlap=50):
        """Split text into overlapping semantic chunks"""
        chunks = []
        words = text.split()

        for i in range(0, len(words), chunk_size - overlap):
            chunk = ' '.join(words[i:i + chunk_size])
            chunks.append(chunk)

        return chunks
```

## Building the Chatbot

### RAG-Augmented LLM

```python
import openai
from typing import List, Dict

class RoboticsLLMChatbot:
    def __init__(self, knowledge_base: RoboticsKnowledgeBase):
        self.kb = knowledge_base
        self.conversation_history = []

    def answer_question(self, question: str) -> Dict[str, str]:
        """Answer user question using RAG"""

        # 1. Retrieve relevant context
        retrieved_chunks = self.kb.retrieve(question, top_k=3)

        # 2. Build augmented prompt
        context_str = "\n\n---\n\n".join([
            f"**{chunk['source']}**\n{chunk['content']}"
            for chunk in retrieved_chunks
        ])

        system_prompt = """You are a knowledgeable robotics instructor answering questions from students.
ALWAYS:
1. Answer ONLY using the provided textbook content
2. Cite the specific chapter/section for your answer
3. If the answer is not in the provided content, say "This topic is not covered in the textbook"
4. Be concise and clear"""

        augmented_prompt = f"""Textbook Content:

{context_str}

---

Student Question: {question}

Instructor Response:"""

        # 3. Generate response with LLM
        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": augmented_prompt}
            ],
            temperature=0.5  # Lower for consistency
        )

        answer = response.choices[0].message.content

        # 4. Add to history and return with citations
        self.conversation_history.append({
            'question': question,
            'answer': answer,
            'sources': [chunk['source'] for chunk in retrieved_chunks]
        })

        return {
            'answer': answer,
            'sources': [chunk['source'] for chunk in retrieved_chunks],
            'retrieved_context': retrieved_chunks
        }

    def multi_turn_conversation(self):
        """Interactive chatbot loop"""
        print("Robotics Textbook Chatbot")
        print("Type 'quit' to exit\n")

        while True:
            question = input("You: ").strip()
            if question.lower() == 'quit':
                break

            result = self.answer_question(question)
            print(f"\nBot: {result['answer']}\n")
            print(f"Sources: {', '.join(result['sources'])}\n")
```

### Example Conversation

```
User: How do I implement forward kinematics for a 2-DOF arm?

Bot: Forward kinematics computes the end-effector position and orientation given
joint angles. For a 2-DOF arm with links of length L1 and L2:

1. Apply rotation matrices for each joint
2. Accumulate transformations: T_base_to_ee = T1 * T2
3. Extract position and orientation from final transformation matrix

Here's the implementation:
[Python code from Chapter 8]

Sources: Chapter 8: Humanoid Robot Modeling, Section 4.2
```

## Advanced Techniques

### Semantic Query Expansion

```python
def expand_query(query: str) -> List[str]:
    """Generate similar queries to improve retrieval"""
    expansion_prompt = f"""Generate 2 alternative ways to ask this question:
Query: {query}

Alternative 1:
Alternative 2:"""

    response = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",
        messages=[{"role": "user", "content": expansion_prompt}],
        temperature=0.7
    )

    alternatives = response.choices[0].message.content.split('\n')
    alternatives = [q.split(':')[1].strip() for q in alternatives if ':' in q]

    return [query] + alternatives

# Use in retrieval
def retrieve_with_expansion(kb, question, top_k=5):
    expanded_queries = expand_query(question)
    all_results = []

    for q in expanded_queries:
        results = kb.retrieve(q, top_k=top_k//len(expanded_queries))
        all_results.extend(results)

    # Deduplicate and rank
    unique_results = {r['content']: r for r in all_results}.values()
    return sorted(list(unique_results), key=lambda x: x['score'], reverse=True)[:top_k]
```

### Evaluation Metrics

```python
def evaluate_chatbot(test_questions: List[str], expected_answers: List[str]):
    """Evaluate chatbot accuracy"""

    correct = 0
    total = len(test_questions)

    for question, expected in zip(test_questions, expected_answers):
        response = chatbot.answer_question(question)
        answer = response['answer'].lower()

        # Check if expected content is in response
        if expected.lower() in answer:
            correct += 1
            print(f"✓ {question}")
        else:
            print(f"✗ {question}")
            print(f"  Expected: {expected}")
            print(f"  Got: {answer[:200]}...")

    accuracy = correct / total * 100
    print(f"\nAccuracy: {accuracy:.1f}%")
    return accuracy
```

## Deployment

### FastAPI Backend

```python
from fastapi import FastAPI
from pydantic import BaseModel

app = FastAPI()
chatbot = RoboticsLLMChatbot(knowledge_base)

class QueryRequest(BaseModel):
    question: str

class QueryResponse(BaseModel):
    answer: str
    sources: List[str]

@app.post("/ask")
async def ask_question(request: QueryRequest) -> QueryResponse:
    """Answer robotics question"""
    result = chatbot.answer_question(request.question)
    return QueryResponse(
        answer=result['answer'],
        sources=result['sources']
    )

@app.get("/health")
async def health_check():
    return {"status": "ok"}
```

### Frontend Integration

```javascript
async function askQuestion(question) {
    const response = await fetch('/ask', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({question: question})
    });

    const result = await response.json();
    displayAnswer(result.answer, result.sources);
}

function displayAnswer(answer, sources) {
    const answerDiv = document.getElementById('answer');
    answerDiv.innerHTML = `
        <p>${answer}</p>
        <p><strong>Sources:</strong> ${sources.join(', ')}</p>
    `;
}
```

## Exercises

**Exercise 14.1**: Build a RAG system using the textbook chapters
**Exercise 14.2**: Create test cases and measure chatbot accuracy
**Exercise 14.3**: Deploy RAG chatbot as FastAPI service with web interface

## References

1. Retrieval-Augmented Generation (Lewis et al., 2020): https://arxiv.org/abs/2005.11401
2. LLMChain (LangChain): https://python.langchain.com/
3. ChromaDB Documentation: https://docs.trychroma.com/
4. In-Context Learning (Brown et al., 2020): https://arxiv.org/abs/2005.14165
5. Evaluating RAG Systems: https://arxiv.org/abs/2309.01697
