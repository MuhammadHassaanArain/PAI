import os
import qdrant_client
from qdrant_client.http.models import Distance, VectorParams
from qdrant_client import QdrantClient, models
from sentence_transformers import SentenceTransformer
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Configuration
QDRANT_HOST = os.getenv("QDRANT_HOST")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_textbook")
CHAPTERS_DIR = os.getenv("CHAPTERS_DIR", "content") # Directory where your markdown chapters are

# Initialize SentenceTransformer model for embeddings
# Using a model suitable for general purpose embeddings
model = SentenceTransformer('all-MiniLM-L6-v2') 

def get_embedding(text: str) -> list[float]:
    """Generates a vector embedding for the given text."""
    return model.encode(text).tolist()

def chunk_text(text: str, chunk_size: int = 512, chunk_overlap: int = 64) -> list[str]:
    """Simple text chunking logic."""
    chunks = []
    # A more sophisticated chunking mechanism (e.g., recursive character text splitter) 
    # would be used in a production system, but for now, we'll do a simple split.
    words = text.split()
    i = 0
    while i < len(words):
        chunk = " ".join(words[i:i + chunk_size])
        chunks.append(chunk)
        i += chunk_size - chunk_overlap
        if i < 0: # Ensure i doesn't go negative on first iteration if chunk_overlap > chunk_size
            i = 0
    return chunks

def ingest_chapters_to_qdrant(qdrant_client: QdrantClient):
    """
    Reads markdown chapters, chunks them, generates embeddings,
    and uploads them to Qdrant.
    """
    if not os.path.exists(CHAPTERS_DIR):
        print(f"Error: Chapters directory '{CHAPTERS_DIR}' not found.")
        return

    # Ensure collection exists or create it
    try:
        qdrant_client.recreate_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(size=model.get_sentence_embedding_dimension(), distance=Distance.COSINE),
        )
        print(f"Collection '{COLLECTION_NAME}' recreated successfully.")
    except Exception as e:
        print(f"Could not recreate collection, it might already exist or an error occurred: {e}")
        # If collection already exists, we might want to just proceed
        # or handle differently depending on desired behavior.
        # For simplicity, we'll try to create it and if it fails, assume it exists.
        qdrant_client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(size=model.get_sentence_embedding_dimension(), distance=Distance.COSINE),
        )
        print(f"Collection '{COLLECTION_NAME}' created (or already exists).")


    points = []
    point_id_counter = 0

    for filename in os.listdir(CHAPTERS_DIR):
        if filename.endswith(".md"):
            file_path = os.path.join(CHAPTERS_DIR, filename)
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            chapter_name = os.path.splitext(filename)[0]
            print(f"Processing chapter: {chapter_name}")

            # Extract title (assuming first H1 is the title)
            title_match = next((line for line in content.split('\n') if line.startswith('# ')), None)
            chapter_title = title_match[2:].strip() if title_match else chapter_name

            chunks = chunk_text(content)

            for i, chunk in enumerate(chunks):
                embedding = get_embedding(chunk)
                payload = {
                    "text": chunk,
                    "chapter_name": chapter_name,
                    "chapter_title": chapter_title,
                    "chunk_id": i
                }
                points.append(
                    models.PointStruct(
                        id=point_id_counter,
                        vector=embedding,
                        payload=payload
                    )
                )
                point_id_counter += 1
    
    if points:
        # Upload points to Qdrant in batches
        batch_size = 100
        for i in range(0, len(points), batch_size):
            qdrant_client.upsert(
                collection_name=COLLECTION_NAME,
                wait=True,
                points=points[i:i + batch_size]
            )
        print(f"Ingested {len(points)} chunks into Qdrant collection '{COLLECTION_NAME}'.")
    else:
        print("No chapters processed or no chunks generated.")

def main():
    if not all([QDRANT_HOST, QDRANT_API_KEY]):
        print("Error: QDRANT_HOST and QDRANT_API_KEY must be set in the .env file.")
        return

    qdrant_client = QdrantClient(
        host=QDRANT_HOST,
        api_key=QDRANT_API_KEY,
    )

    ingest_chapters_to_qdrant(qdrant_client)

if __name__ == "__main__":
    main()
