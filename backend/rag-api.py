import os
import qdrant_client
from qdrant_client import QdrantClient, models
from sentence_transformers import SentenceTransformer
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Configuration
QDRANT_HOST = os.getenv("QDRANT_HOST")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_textbook")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY") # For LLM

# Initialize FastAPI app
app = FastAPI()

# Initialize SentenceTransformer model for embeddings
# This should be the same model used in rag-ingest.py
model = SentenceTransformer('all-MiniLM-L6-v2') 

# Initialize Qdrant client
qdrant_client = QdrantClient(
    host=QDRANT_HOST,
    api_key=QDRANT_API_KEY,
)

# Pydantic models for request and response
class QueryRequest(BaseModel):
    query: str
    limit: int = 3
    min_score: float = 0.7 # Minimum relevance score for retrieved chunks

class QueryResponse(BaseModel):
    answer: str
    source_chapters: list[str]
    retrieved_chunks: list[dict]

class SelectedTextQueryRequest(BaseModel):
    query: str
    selected_text: str # The text selected by the user
    limit: int = 1
    min_score: float = 0.8 # Higher score for selected text mode

def get_embedding(text: str) -> list[float]:
    """Generates a vector embedding for the given text."""
    return model.encode(text).tolist()

async def generate_llm_answer(query: str, context: str) -> str:
    """
    Generates an answer using an LLM based on the query and provided context.
    This is a placeholder. In a real application, you'd integrate with
    OpenAI, Anthropic, Gemini, etc.
    """
    if not OPENAI_API_KEY:
        return "LLM API key not configured. Cannot generate answer."

    # Using a dummy response for now. Integrate with a real LLM here.
    # from openai import AsyncOpenAI
    # client = AsyncOpenAI(api_key=OPENAI_API_KEY)
    # messages = [
    #     {"role": "system", "content": "You are a helpful assistant that answers questions based on the provided text context."},
    #     {"role": "user", "content": f"Context: {context}\n\nQuestion: {query}\n\nAnswer:"}
    # ]
    # response = await client.chat.completions.create(
    #     model="gpt-3.5-turbo", # Or your preferred LLM
    #     messages=messages,
    #     max_tokens=500
    # )
    # return response.choices[0].message.content.strip()
    
    return f"Based on the book content:\n'{context}'\n\nI can tell you about: {query}"


@app.post("/query", response_model=QueryResponse)
async def query_book(request: QueryRequest):
    """
    Answers a query based on the ingested book content using RAG.
    """
    query_embedding = get_embedding(request.query)

    search_result = qdrant_client.search(
        collection_name=COLLECTION_NAME,
        query_vector=query_embedding,
        limit=request.limit,
        score_threshold=request.min_score,
        with_payload=True,
    )

    if not search_result:
        raise HTTPException(status_code=404, detail="No relevant content found in the book.")

    context_chunks = []
    source_chapters = set()
    retrieved_data = []

    for hit in search_result:
        context_chunks.append(hit.payload["text"])
        source_chapters.add(hit.payload["chapter_name"])
        retrieved_data.append({
            "text": hit.payload["text"],
            "chapter_name": hit.payload["chapter_name"],
            "chapter_title": hit.payload["chapter_title"],
            "chunk_id": hit.payload["chunk_id"],
            "score": hit.score
        })

    context = "\n---".join(context_chunks)
    answer = await generate_llm_answer(request.query, context)

    # Enforce book-only answering and citation
    final_answer = f"Based on the book content, specifically from chapters: {', '.join(source_chapters)}.\n\n{answer}"

    return QueryResponse(
        answer=final_answer,
        source_chapters=list(source_chapters),
        retrieved_chunks=retrieved_data
    )

@app.post("/query_selected_text", response_model=QueryResponse)
async def query_with_selected_text(request: SelectedTextQueryRequest):
    """
    Answers a query based ONLY on the provided selected text by the user.
    If the query cannot be answered from the selected text, it will be rejected.
    """
    # Directly use the selected text as the context.
    # We can still search Qdrant for very high relevance to confirm it's from the book
    # but the primary context is the selected_text.

    # Optional: Verify selected_text is actually from the book and relevant to query
    # by embedding both and checking similarity, or by searching Qdrant with selected_text as query
    selected_text_embedding = get_embedding(request.selected_text)
    query_embedding = get_embedding(request.query)

    # Check if the selected text itself is very relevant to the query
    # (Simplified check: in a real system, use more robust similarity or search)
    combined_embedding = get_embedding(request.query + " " + request.selected_text)
    
    search_result = qdrant_client.search(
        collection_name=COLLECTION_NAME,
        query_vector=combined_embedding,
        limit=request.limit,
        score_threshold=request.min_score, # Use higher score threshold
        with_payload=True,
    )

    is_relevant_and_from_book = False
    source_chapters = set()
    retrieved_data = []

    if search_result:
        for hit in search_result:
            # Check if the retrieved text is very similar or contains the selected_text
            # (Simple substring check, a more robust semantic check needed for production)
            if request.selected_text in hit.payload["text"] or hit.score > request.min_score:
                is_relevant_and_from_book = True
                source_chapters.add(hit.payload["chapter_name"])
                retrieved_data.append({
                    "text": hit.payload["text"],
                    "chapter_name": hit.payload["chapter_name"],
                    "chapter_title": hit.payload["chapter_title"],
                    "chunk_id": hit.payload["chunk_id"],
                    "score": hit.score
                })
        
    if not is_relevant_and_from_book:
         raise HTTPException(status_code=400, detail="The question cannot be answered solely from the selected text provided, or the selected text is not sufficiently relevant to the question.")
    
    # Use only the selected text as context for the LLM
    answer = await generate_llm_answer(request.query, request.selected_text)

    final_answer = f"Based on your selected text (from chapters: {', '.join(source_chapters)}):\n\n{answer}"

    return QueryResponse(
        answer=final_answer,
        source_chapters=list(source_chapters),
        retrieved_chunks=retrieved_data
    )

@app.get("/")
async def root():
    return {"message": "RAG Chatbot API for Physical AI Textbook"}
