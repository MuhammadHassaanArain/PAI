# API Contracts: RAG Chatbot and User Management

**Feature Branch**: `001-ai-robotics-textbook`  
**Created**: December 4, 2025  
**Spec**: [specs/001-ai-robotics-textbook/spec.md](specs/001-ai-robotics-textbook/spec.md)
**Plan**: [specs/001-ai-robotics-textbook/plan.md](specs/001-ai-robotics-textbook/plan.md)
**Data Model**: [specs/001-ai-robotics-textbook/data-model.md](specs/001-ai-robotics-textbook/data-model.md)

## Overview

This document outlines the API contracts for the backend FastAPI server, focusing on the RAG chatbot interactions and user profile management (for personalization). All endpoints are designed to be consumed by the Docusaurus frontend.

## API Versioning

*   **Strategy**: Path-based versioning (e.g., `/api/v1/...`).
*   **Current Version**: `v1`

## Authentication

*   If user authentication is implemented (bonus scope), Better-Auth will be used. API endpoints will require a valid authentication token. For initial development, some endpoints might be unauthenticated.

## Endpoints

### 1. Chatbot Interaction

**Endpoint**: `/api/v1/chat/query`
**Method**: `POST`
**Description**: Sends a user query to the RAG chatbot and receives a response.

*   **Request Body**: `application/json`
    ```json
    {
      "student_id": "string",  # Optional, if user is authenticated/tracked
      "query_text": "string",  # The user's question
      "context": {             # Optional, e.g., selected text from the book
        "selected_text": "string",
        "chapter_id": "string",
        "section_id": "string"
      }
    }
    ```
*   **Response Body**: `application/json` (HTTP 200 OK)
    ```json
    {
      "response_id": "string",
      "query_id": "string",
      "response_text": "string",
      "timestamp": "string",    # ISO 8601 format
      "citation": {
        "chapter_id": "string",
        "section_id": "string"
      },
      "retrieval_trace": "object", # Detailed trace for auditing
      "agent_type": "string",    # e.g., "explanation"
      "refusal_reason": "string" # Present if query was refused
    }
    ```
*   **Error Responses**:
    *   `HTTP 400 Bad Request`: Invalid input or malformed request.
    *   `HTTP 429 Too Many Requests`: LLM quota exceeded (with `refusal_reason`).
    *   `HTTP 500 Internal Server Error`: Unexpected server error.

### 2. User Profile Management (Bonus Scope)

**Endpoint**: `/api/v1/user/profile`
**Method**: `POST`
**Description**: Creates or updates a student's profile for personalization.

*   **Request Body**: `application/json`
    ```json
    {
      "student_id": "string",        # Required for update, optional for create
      "programming_level": "string", # "beginner", "intermediate", "advanced"
      "ai_knowledge_level": "string", # "none", "basic", "intermediate", "advanced"
      "hardware_availability": "string", # "local_gpu", "no_gpu", "cloud_access"
      "learning_speed_preference": "string" # "fast", "moderate", "slow"
    }
    ```
*   **Response Body**: `application/json` (HTTP 200 OK)
    ```json
    {
      "student_id": "string",
      "message": "User profile updated successfully."
    }
    ```
*   **Error Responses**:
    *   `HTTP 400 Bad Request`: Invalid input.
    *   `HTTP 401 Unauthorized`: No valid authentication token.
    *   `HTTP 500 Internal Server Error`.

**Endpoint**: `/api/v1/user/profile/{student_id}`
**Method**: `GET`
**Description**: Retrieves a student's profile.

*   **Path Parameters**:
    *   `student_id`: string (UUID)
*   **Response Body**: `application/json` (HTTP 200 OK)
    ```json
    {
      "student_id": "string",
      "programming_level": "string",
      "ai_knowledge_level": "string",
      "hardware_availability": "string",
      "learning_speed_preference": "string"
    }
    ```
*   **Error Responses**:
    *   `HTTP 401 Unauthorized`: No valid authentication token.
    *   `HTTP 404 Not Found`: Student ID not found.
    *   `HTTP 500 Internal Server Error`.

### 3. Content Retrieval (For Agent/Backend Use)

**Endpoint**: `/api/v1/content/{chapter_id}/{section_id}`
**Method**: `GET`
**Description**: Retrieves the raw Markdown content for a specific chapter/section. This endpoint is primarily for internal backend agent use to fetch source content for chunking/embedding or for RAG context building.

*   **Path Parameters**:
    *   `chapter_id`: string (UUID)
    *   `section_id`: string
*   **Response Body**: `application/json` (HTTP 200 OK)
    ```json
    {
      "content_id": "string",
      "chapter_id": "string",
      "section_id": "string",
      "title": "string",
      "markdown_content": "string",
      "version_metadata": "string"
    }
    ```
*   **Error Responses**:
    *   `HTTP 404 Not Found`: Content not found.
    *   `HTTP 500 Internal Server Error`.
