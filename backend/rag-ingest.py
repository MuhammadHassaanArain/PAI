import os
import re
from typing import List, Dict, Any
from qdrant_client import QdrantClient, models
from qdrant_client.http.models import Distance, VectorParams
import markdown
import hashlib
import json

# --- Configuration ---
QDRANT_HOST = os.getenv("QDRANT_HOST", "localhost")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", None)
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "textbook_chapters")
EMBEDDING_MODEL_NAME = os.getenv("EMBEDDING_MODEL_NAME", "text-embedding-004") # Example Gemini embedding model
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY", None) # For embedding model if hosted by Gemini

CONTENT_DIR = "content/sections" # Relative path to markdown content

# --- Initialize Qdrant Client ---
def initialize_qdrant_client():
    client = QdrantClient(host=QDRANT_HOST, api_key=QDRANT_API_KEY, port=6333, prefer_grpc=True)
    return client

# --- Text Chunking ---
def chunk_markdown(markdown_text: str, chapter_id: str, section_id: str, max_chunk_size: int = 500) -> List[Dict[str, Any]]:
    """
    Chunks markdown text into smaller pieces, preserving context and adding metadata.
    This is a simplified chunking strategy. More advanced methods could use AST parsing.
    """
    chunks = []
    current_chunk = ""
    current_chunk_metadata = {"chapter_id": chapter_id, "section_id": section_id, "version_metadata": "v1.0"}
    line_num = 0

    # Split by paragraphs/headings, then by sentences as fallback
    lines = markdown_text.split('\n')
    for line in lines:
        line_num += 1
        # Simple heuristic: treat headings and empty lines as chunk breaks
        if line.strip().startswith("#") or not line.strip():
            if current_chunk:
                chunks.append({"text": current_chunk.strip(), "metadata": current_chunk_metadata.copy(), "start_line": current_chunk_metadata.get("start_line")})
                current_chunk = ""
            current_chunk_metadata["start_line"] = line_num
            current_chunk += line + "\n"
        elif len(current_chunk.split()) + len(line.split()) <= max_chunk_size:
            current_chunk += line + "\n"
        else:
            if current_chunk:
                chunks.append({"text": current_chunk.strip(), "metadata": current_chunk_metadata.copy(), "start_line": current_chunk_metadata.get("start_line")})
            current_chunk = line + "\n"
            current_chunk_metadata["start_line"] = line_num

    if current_chunk:
        chunks.append({"text": current_chunk.strip(), "metadata": current_chunk_metadata.copy(), "start_line": current_chunk_metadata.get("start_line")})

    # Generate content_id and embedding_id for each chunk
    for i, chunk in enumerate(chunks):
        # A simple unique ID for the chunk
        chunk["metadata"]["content_id"] = hashlib.md5((chunk["text"] + chapter_id + section_id + str(i)).encode()).hexdigest()
        chunk["metadata"]["chunk_order"] = i

    return chunks

# --- Embedding Generation ---
# This would typically involve an API call to a service like Google's Gemini API
# or OpenAI's API. For simplicity and to avoid direct API key handling in this snippet,
# we'll use a placeholder. In a real scenario, ensure secure API key management.
def generate_embedding(text: str) -> List[float]:
    """
    Generates a vector embedding for the given text using the specified model.
    Replace with actual API call to Gemini or OpenAI embedding service.
    """
    if not GEMINI_API_KEY:
        print("Warning: GEMINI_API_KEY not set. Using dummy embeddings.")
        # Return a dummy embedding for local testing without API key
        return [0.1] * 1536 # Common embedding dimension

    # Placeholder for actual API call
    # Example using a hypothetical Gemini client
    # from gemini_client import GeminiEmbeddings
    # client = GeminiEmbeddings(api_key=GEMINI_API_KEY, model=EMBEDDING_MODEL_NAME)
    # response = client.embed_content(text)
    # return response.embedding

    # For now, return a dummy embedding if no actual API call is integrated
    # In a real system, you'd make an HTTP request or use an SDK
    print(f"Generating dummy embedding for: '{text[:50]}...'")
    # A realistic embedding size should match the model's output, e.g., 768, 1024, 1536
    return [hash(text) % 1000 * 0.001] * 768 # Dummy embedding (replace with actual)


# --- Ingestion into Qdrant ---
def ingest_chunks_to_qdrant(client: QdrantClient, chunks: List[Dict[str, Any]], vector_size: int):
    # Ensure collection exists
    try:
        client.recreate_collection(
            collection_name=QDRANT_COLLECTION_NAME,
            vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE),
        )
        print(f"Recreated collection '{QDRANT_COLLECTION_NAME}'")
    except Exception as e:
        print(f"Could not recreate collection (might already exist): {e}. Ensuring configuration...")
        # Check if vectors_config is correct if collection already exists
        collection_info = client.get_collection(collection_name=QDRANT_COLLECTION_NAME)
        if collection_info.config.vectors.size != vector_size or collection_info.config.vectors.distance != Distance.COSINE:
            print(f"Warning: Existing collection '{QDRANT_COLLECTION_NAME}' has mismatching vector config. Consider manual update or deletion.")


    points = []
    for chunk in chunks:
        # Generate a unique Qdrant ID for the point
        point_id = hashlib.md5(chunk["metadata"]["content_id"].encode()).hexdigest()

        points.append(
            models.PointStruct(
                id=point_id,
                vector=generate_embedding(chunk["text"]),
                payload=chunk["metadata"]
            )
        )
    
    if points:
        client.upsert(
            collection_name=QDRANT_COLLECTION_NAME,
            points=points,
            wait=True
        )
        print(f"Ingested {len(points)} chunks into '{QDRANT_COLLECTION_NAME}'.")
    else:
        print("No chunks to ingest.")

# --- Main Ingestion Logic ---
def main():
    if not GEMINI_API_KEY:
        print("WARNING: GEMINI_API_KEY environment variable not set. Embeddings will be dummy values.")
        print("Please set GEMINI_API_KEY for real embedding generation.")

    qdrant_client = initialize_qdrant_client()
    all_chunks = []

    # Walk through the content directory to find markdown files
    for root, _, files in os.walk(CONTENT_DIR):
        for file in files:
            if file.endswith(".md"):
                file_path = os.path.join(root, file)
                # Extract chapter_id and section_id from path (e.g., content/sections/1-ros2/module-1-ros2-core.md)
                path_parts = os.path.relpath(file_path, CONTENT_DIR).split(os.sep)
                
                # Heuristic for chapter_id/section_id based on typical path structure
                chapter_id_match = re.match(r"(\d+)-.*", path_parts[0])
                chapter_id = chapter_id_match.group(1) if chapter_id_match else "unknown_chapter"
                
                section_id = file.replace(".md", "") # Use filename as section_id

                print(f"Processing {file_path} (Chapter: {chapter_id}, Section: {section_id})...")
                with open(file_path, 'r', encoding='utf-8') as f:
                    markdown_content = f.read()
                
                chunks = chunk_markdown(markdown_content, chapter_id, section_id)
                all_chunks.extend(chunks)

    # Assuming a fixed vector size for dummy embeddings or from actual model
    # Replace 768 with the actual size of your embedding model's output
    vector_size = 768 
    
    if all_chunks:
        print(f"Total chunks generated: {len(all_chunks)}")
        ingest_chunks_to_qdrant(qdrant_client, all_chunks, vector_size)
    else:
        print("No markdown files found or no chunks generated for ingestion.")

if __name__ == "__main__":
    main()