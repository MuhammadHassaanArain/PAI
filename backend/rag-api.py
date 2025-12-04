import os
from fastapi import FastAPI, HTTPException, status
from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any
from qdrant_client import QdrantClient, models # models added for ScoredPoint
from qdrant_client.http.models import Filter, FieldCondition, MatchValue
import google.generativeai as genai

from backend.src.models.user_profile import StudentProfile, ProgrammingLevel, AIKnowledgeLevel, HardwareAvailability, LearningSpeedPreference # Import StudentProfile

# --- Configuration ---
QDRANT_HOST = os.getenv("QDRANT_HOST", "localhost")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", None)
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "textbook_chapters")
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY", None)
EMBEDDING_MODEL_NAME = os.getenv("EMBEDDING_MODEL_NAME", "models/embedding-001") # Actual Gemini embedding model name

# Ensure Gemini API key is set
if not GEMINI_API_KEY:
    raise ValueError("GEMINI_API_KEY environment variable not set.")
genai.configure(api_key=GEMINI_API_KEY)

# --- Initialize Clients ---
app = FastAPI(title="AI-Native Textbook RAG API", version="1.0.0")
qdrant_client = QdrantClient(host=QDRANT_HOST, api_key=QDRANT_API_KEY, port=6333, prefer_grpc=True)
gemini_model = genai.GenerativeModel('gemini-pro') # For text generation

# --- Pydantic Models ---
class ChatContext(BaseModel):
    selected_text: Optional[str] = None
    chapter_id: Optional[str] = None
    section_id: Optional[str] = None

class ChatQuery(BaseModel):
    student_id: Optional[str] = None
    query_text: str = Field(..., min_length=1, max_length=1000)
    context: Optional[ChatContext] = None
    student_profile: Optional[StudentProfile] = None # Added student_profile to ChatQuery

class Citation(BaseModel):
    chapter_id: str
    section_id: str

class ChatResponse(BaseModel):
    response_id: str
    query_id: str
    response_text: str
    timestamp: str
    citation: Optional[Citation] = None
    retrieval_trace: Optional[Dict[str, Any]] = None
    agent_type: str = "explanation"
    refusal_reason: Optional[str] = None

# --- Helper Functions ---
def generate_embedding(text: str) -> List[float]:
    """Generates an embedding for the given text using Gemini."""
    try:
        response = genai.embed_content(model=EMBEDDING_MODEL_NAME, content=text)
        return response['embedding']
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to generate embedding: {e}"
        )

# --- Agent Placeholders (will be integrated with OpenAI Agent SDK) ---
class RetrievalAgent:
    def __init__(self, qdrant_client: QdrantClient):
        self.qdrant_client = qdrant_client

    def retrieve_chunks(self, query_embedding: List[float], filters: Optional[Filter] = None, limit: int = 5) -> List[str]:
        try:
            search_result = self.qdrant_client.search(
                collection_name=QDRANT_COLLECTION_NAME,
                query_vector=query_embedding,
                query_filter=filters,
                limit=limit
            )
            chunks = [hit.payload["text"] for hit in search_result if hit.payload and "text" in hit.payload]
            trace = [{"id": hit.id, "score": hit.score, "payload": hit.payload} for hit in search_result]
            return chunks, trace
        except Exception as e:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Retrieval failed: {e}"
            )

class SafetyRefusalAgent:
    def check_query_safety_and_scope(self, query_text: str, chapter_id: Optional[str] = None, section_id: Optional[str] = None) -> Optional[str]:
        # Placeholder for actual safety and scope check logic
        # For now, a very basic check
        if "unsafe_keyword" in query_text.lower():
            return "Query contains unsafe content."
        
        # If specific context is given, assume it's in scope unless specified otherwise
        # More advanced check would involve comparing query_text to available content topics
        if chapter_id and "out_of_scope_topic" in query_text.lower():
            return "Query is out of scope for the selected content."
        
        return None # No refusal needed

class ExplanationAgent:
    def generate_response(self, query_text: str, context_chunks: List[str], selected_text: Optional[str] = None, student_profile: Optional[StudentProfile] = None) -> str: # Added student_profile
        full_context = "\n\n".join(context_chunks)
        
        personalization_instruction = ""
        if student_profile:
            personalization_instruction = f"""
            Adapt your response to a student with the following profile:
            - Programming Level: {student_profile.programming_level.value if student_profile.programming_level else 'unknown'}
            - AI Knowledge Level: {student_profile.ai_knowledge_level.value if student_profile.ai_knowledge_level else 'unknown'}
            - Learning Speed Preference: {student_profile.learning_speed_preference.value if student_profile.learning_speed_preference else 'unknown'}
            
            Focus on {student_profile.learning_speed_preference.value} pace, use analogies suitable for a {student_profile.programming_level.value} programmer, and tailor explanations based on {student_profile.ai_knowledge_level.value} AI knowledge.
            """

        # Adjust prompt based on whether selected_text was provided
        if selected_text:
            prompt = f"""You are an AI assistant for a Physical AI and Humanoid Robotics textbook.
            Answer the following question STRICTLY based on the SELECTED TEXT provided,
            which is also contained within the Context. If the question cannot be answered from
            the SELECTED TEXT, state "I cannot answer this question based on the selected text."
            {personalization_instruction}
            
            Selected Text: {selected_text}
            
            Question: {query_text}
            
            Context (full relevant chunks from textbook):
            {full_context}
            
            Answer:"""
        else:
            prompt = f"""You are an AI assistant for a Physical AI and Humanoid Robotics textbook.
            Answer the following question STRICTLY based on the provided Context from the textbook.
            If the question cannot be answered from the Context, state "I cannot answer this question based on the provided textbook content."
            {personalization_instruction}
            
            Question: {query_text}
            
            Context:
            {full_context}
            
            Answer:"""
        
        try:
            # Check for LLM quota (conceptual)
            # if genai.get_quota_status() == "exceeded":
            #     raise HTTPException(status_code=status.HTTP_429_TOO_MANY_REQUESTS, detail="LLM quota exceeded.")
            
            response = gemini_model.generate_content(prompt)
            return response.text
        except Exception as e:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Failed to generate content from LLM: {e}"
            )

class VerificationAgent:
    def __init__(self, llm_model):
        self.llm_model = llm_model

    def verify_and_cite(self, generated_text: str, source_chunks_with_payload: List[Dict[str, Any]], query_text: str) -> (str, Optional[Citation]):
        """
        Verifies if the generated_text is supported by source_chunks_with_payload and extracts a citation.
        Uses LLM to perform verification.
        """
        # Extract text content from the source_chunks_with_payload for LLM context
        source_texts = [chunk.payload.get("text", "") for chunk in source_chunks_with_payload]
        context_for_llm = "\n\n".join(source_texts)
        
        # Prompt the LLM to verify and extract citation
        prompt = f"""You are a verification agent. Your task is to check if the 'Generated Answer' is directly supported by the 'Source Context'.
        You must also identify the most relevant chapter_id and section_id from the 'Source Context' that supports the answer.
        
        If the 'Generated Answer' contains information NOT present in the 'Source Context', mark it as a hallucination.
        If the 'Generated Answer' is supported, provide the 'Generated Answer' as is, and the citation.
        If the 'Generated Answer' is not supported, respond with "I cannot verify this answer based on the provided sources."
        
        Query: {query_text}
        
        Generated Answer: {generated_text}
        
        Source Context (raw text and metadata from chunks):
        {context_for_llm}
        
        Available Sources with IDs (for citation extraction):
        {json.dumps([{"id": chunk.id, "chapter_id": chunk.payload.get("chapter_id"), "section_id": chunk.payload.get("section_id")} for chunk in source_chunks_with_payload], indent=2)}
        
        Your response should be in JSON format with 'verified_answer', 'chapter_id', 'section_id', and 'is_hallucination' keys.
        Example:
        {{
            "verified_answer": "...",
            "chapter_id": "...",
            "section_id": "...",
            "is_hallucination": false
        }}
        """
        
        try:
            response = self.llm_model.generate_content(prompt)
            # Extract only the JSON part from the response if the LLM adds extra text
            json_match = re.search(r"```json\n(.*)\n```", response.text, re.DOTALL)
            if json_match:
                response_json = json.loads(json_match.group(1))
            else:
                response_json = json.loads(response.text) # Assume direct JSON

            verified_answer = response_json.get("verified_answer", "I cannot verify this answer based on the provided sources.")
            is_hallucination = response_json.get("is_hallucination", True)
            chapter_id = response_json.get("chapter_id")
            section_id = response_json.get("section_id")

            citation = None
            if chapter_id and section_id and not is_hallucination:
                citation = Citation(chapter_id=chapter_id, section_id=section_id)
            
            return verified_answer, citation
            
        except Exception as e:
            print(f"Error during verification: {e}")
            return "An error occurred during verification. Please try again.", None


# --- Initialize Agent Instances ---
retrieval_agent = RetrievalAgent(qdrant_client)
safety_agent = SafetyRefusalAgent()
explanation_agent = ExplanationAgent()
verification_agent = VerificationAgent(gemini_model) # Initialize verification agent

# --- API Endpoints ---
@app.post("/api/v1/chat/query", response_model=ChatResponse)
async def chat_query(query: ChatQuery):
    query_id = str(os.urandom(16).hex()) # Dummy query ID
    response_id = str(os.urandom(16).hex()) # Dummy response ID
    timestamp = genai.types.Timestamp.now().isoformat()

    # 1. Safety & Refusal Agent Check
    refusal_reason = safety_agent.check_query_safety_and_scope(
        query.query_text,
        query.context.chapter_id if query.context else None,
        query.context.section_id if query.context else None
    )
    if refusal_reason:
        return ChatResponse(
            response_id=response_id,
            query_id=query_id,
            response_text="I'm sorry, I cannot answer this query because it is out of scope or contains unsafe content.",
            timestamp=timestamp,
            refusal_reason=refusal_reason
        )

    # 2. Generate Embedding for the query (or selected text)
    query_embedding = generate_embedding(query.query_text)
    
    qdrant_filters = None
    if query.context and query.context.selected_text:
        query_embedding = generate_embedding(query.context.selected_text) # Use selected text for embedding
        # Apply filters if specific context is provided
        if query.context.chapter_id or query.context.section_id:
            if qdrant_filters is None:
                qdrant_filters = Filter(must=[])
            if query.context.chapter_id:
                qdrant_filters.must.append(FieldCondition(key="chapter_id", match=MatchValue(value=query.context.chapter_id)))
            if query.context.section_id:
                qdrant_filters.must.append(FieldCondition(key="section_id", match=MatchValue(value=query.context.section_id)))
    else:
        query_embedding = generate_embedding(query.query_text)

    # 3. Retrieval Agent
    # Modify RetrievalAgent.retrieve_chunks to return full hit objects for verification
    class RetrievalAgent: # Re-define temporarily for the replace block
        def __init__(self, qdrant_client: QdrantClient):
            self.qdrant_client = qdrant_client

        def retrieve_chunks(self, query_embedding: List[float], filters: Optional[Filter] = None, limit: int = 5) -> List[models.ScoredPoint]:
            try:
                search_result = self.qdrant_client.search(
                    collection_name=QDRANT_COLLECTION_NAME,
                    query_vector=query_embedding,
                    query_filter=filters,
                    limit=limit
                )
                return search_result
            except Exception as e:
                raise HTTPException(
                    status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                    detail=f"Retrieval failed: {e}"
                )
    retrieval_agent = RetrievalAgent(qdrant_client) # Re-instantiate

    retrieved_hits = retrieval_agent.retrieve_chunks(query_embedding, filters=qdrant_filters)
    context_chunks_text = [hit.payload["text"] for hit in retrieved_hits if hit.payload and "text" in hit.payload]
    retrieval_trace = [{"id": hit.id, "score": hit.score, "payload": hit.payload} for hit in retrieved_hits]

    if not context_chunks_text:
        return ChatResponse(
            response_id=response_id,
            query_id=query_id,
            response_text="I cannot find relevant information in the textbook (or selected text) to answer your question.",
            timestamp=timestamp,
            retrieval_trace=retrieval_trace
        )

    # 4. Explanation Agent - get initial LLM response
    initial_llm_response = explanation_agent.generate_response(query.query_text, context_chunks_text, query.context.selected_text, query.student_profile) # Pass student_profile

    # 5. Verification Agent - verify and get citation
    verified_response, citation = verification_agent.verify_and_cite(
        initial_llm_response,
        retrieved_hits, # Pass full hit objects
        query.query_text
    )

    return ChatResponse(
        response_id=response_id,
        query_id=query_id,
        response_text=verified_response,
        timestamp=timestamp,
        citation=citation,
        retrieval_trace=retrieval_trace
    )

# --- Root Endpoint (for health check) ---
@app.get("/")
async def root():
    return {"message": "AI-Native Textbook RAG API is running!"}

# --- Example of how to run (for local development) ---
# To run this file:
# 1. pip install fastapi "uvicorn[standard]" qdrant-client google-generativeai pydantic
# 2. Set GEMINI_API_KEY environment variable
# 3. uvicorn rag-api:app --reload

    