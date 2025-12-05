# Quickstart Guide: AI-Native Textbook Platform

**Feature Branch**: `001-ai-robotics-textbook`  
**Created**: December 4, 2025  
**Spec**: [specs/001-ai-robotics-textbook/spec.md](specs/001-ai-robotics-textbook/spec.md)
**Plan**: [specs/001-ai-robotics-textbook/plan.md](specs/001-ai-robotics-textbook/plan.md)
**Research**: [specs/001-ai-robotics-textbook/research.md](specs/001-ai-robotics-textbook/research.md)
**Data Model**: [specs/001-ai-robotics-textbook/data-model.md](specs/001-ai-robotics-textbook/data-model.md)
**API Contracts**: [specs/001-ai-robotics-textbook/contracts/api-v1.md](specs/001-ai-robotics-textbook/contracts/api-v1.md)

This guide provides a high-level overview to get the AI-Native Textbook Platform up and running. More detailed instructions will be available in the book's introductory chapters.

## Prerequisites

*   **Operating System**: Ubuntu 22.04 LTS
*   **Python**: 3.8+ (with `pip` and `venv`)
*   **Node.js**: 18+ (with `npm` or `yarn`)
*   **Git**
*   **Docker** (for local Qdrant/PostgreSQL setup, if not using cloud services)
*   **API Keys**: OpenAI (for Agent SDK), Gemini (for LLM), Qdrant (for cloud), Neon Serverless PostgreSQL credentials.

## 1. Clone the Repository

```bash
git clone https://github.com/your-org/your-repo.git 001-ai-robotics-textbook
cd 001-ai-robotics-textbook
```

## 2. Environment Setup

### Backend (FastAPI & Agents)

1.  Navigate to the `backend` directory:
    ```bash
    cd backend
    ```
2.  Create and activate a Python virtual environment:
    ```bash
    python3 -m venv venv
    source venv/bin/activate
    ```
3.  Install Python dependencies:
    ```bash
    pip install -r requirements.txt # (requirements.txt to be generated)
    ```
4.  Create a `.env` file in the `backend` directory and add your API keys and database credentials:
    ```
    OPENAI_API_KEY=your_openai_key
    GEMINI_API_KEY=your_gemini_key
    QDRANT_API_KEY=your_qdrant_key
    QDRANT_HOST=your_qdrant_host
    NEON_PG_CONN_STR="your_neon_postgresql_connection_string"
    ```

### Frontend (Docusaurus)

1.  Navigate to the `frontend` directory:
    ```bash
    cd ../frontend
    ```
2.  Install Node.js dependencies:
    ```bash
    npm install # or yarn install
    ```

## 3. Database Initialization

### Vector Database (Qdrant)

*   **Cloud (Recommended)**: Ensure your Qdrant Cloud instance is provisioned and accessible via the host and API key in your `backend/.env` file.
*   **Local (Docker)**:
    ```bash
    docker run -p 6333:6333 -p 6334:6334 \
        -v $(pwd)/qdrant_storage:/qdrant/storage \
        qdrant/qdrant
    ```

### User Database (Neon Serverless PostgreSQL)

*   Ensure your Neon Serverless PostgreSQL instance is provisioned and your connection string is in `backend/.env`.
*   Run database migrations (commands to be defined later, e.g., using `alembic` or `SQLAlchemy` ORM tools).

## 4. Run the Application

### Backend Server

1.  Activate your Python virtual environment (if not already active) and navigate to `backend`:
    ```bash
    cd backend
    source venv/bin/activate
    ```
2.  Start the FastAPI server:
    ```bash
    uvicorn main:app --reload # (main.py to be generated)
    ```
    The backend will typically run on `http://127.0.0.1:8000`.

### Frontend Website

1.  Navigate to the `frontend` directory:
    ```bash
    cd frontend
    ```
2.  Start the Docusaurus development server:
    ```bash
    npm start # or yarn start
    ```
    The frontend will typically run on `http://localhost:3000`.

## 5. Ingest Textbook Content into RAG System

(Detailed ingestion script to be provided)
Once the backend and Qdrant are running, use a dedicated script to parse the Markdown chapters (`content/`) and ingest them as embeddings into Qdrant.

```bash
# Example command (script to be developed)
python backend/scripts/ingest_content.py --content-path ../content/
```

## 6. Interact with the Chatbot

Once both frontend and backend are running, and content is ingested:

1.  Access the Docusaurus site in your browser (`http://localhost:3000`).
2.  Locate the embedded chatbot UI (to be developed).
3.  Start asking questions about the textbook content!

This quickstart assumes a basic understanding of development environments and command-line interfaces. For specific issues, refer to the detailed documentation within the project.
