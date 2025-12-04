# Project Plan: AI-Native Textbook on Physical AI & Humanoid Robotics

This document outlines the project plan for the AI-Native Textbook on Physical AI & Humanoid Robotics.

## 1. Project Architecture

### Frontend
- **Framework**: Docusaurus for the static site.
- **Chatbot UI**: An embedded chatbot widget.
- **User Profiles**: Optional login and user profile UI (Post-MVP).

### Backend
- **Framework**: FastAPI.
- **Chatbot SDK**: OpenAI Agents / ChatKit SDK.
- **Database**: Neon Serverless PostgreSQL for user data and logs.
- **Vector Store**: Qdrant Cloud (Free Tier) for RAG embeddings.

### AI Pipeline
- **Ingestion**: Markdown content will be chunked, embedded, and stored in Qdrant with chapter-level metadata.
- **Retrieval**: User queries will trigger a vector search to retrieve context for the LLM to generate an answer.
- **Restricted Mode**: A "selected-text-only" mode will be implemented to restrict the RAG context.

### Deployment
- **Frontend**: GitHub Pages or Vercel.
- **Backend**: Railway, Render, or Fly.io.
- **Databases**: Cloud-hosted.

## 2. Book Structure and Content

### Modules
- **Section 0**: Foundations of Physical AI
- **Section 1**: ROS 2 – Robotic Nervous System
- **Section 2**: Digital Twin – Gazebo & Unity
- **Section 3**: AI Brain – NVIDIA Isaac
- **Section 4**: Vision-Language-Action Systems
- **Section 5**: Capstone – Autonomous Humanoid
- **Section 6**: Cloud vs On-Prem Lab Setup
- **Section 7**: Safety, Ethics & Deployment

### Chapter Structure
Each chapter will include:
- Learning Objectives
- Core Concepts
- Step-by-Step Lab
- Code Examples
- Hardware/Cloud Alternative
- Summary
- Assessment / Mini Project

## 3. Execution and Development Plan

### Phases
1.  **Specification Lock**: Finalize all decisions in `research.md`.
2.  **Content Architecture Design**: Create templates and outlines for each chapter.
3.  **Chapter-by-Chapter Generation**: Write and validate one module at a time.
4.  **RAG System Development**: Implement the chatbot ingestion and retrieval pipeline.
5.  **Personalization & Auth (Bonus)**: Implement user profiles and personalized content.
6.  **Deployment & Validation**: Deploy the platform and perform end-to-end testing.
7.  **Demo & Submission**: Create the demo video and submit the project.

### RAG Chatbot Development
1.  Convert chapters to vector embeddings.
2.  Store in Qdrant with chapter-level metadata.
3.  Implement global Q&A and selected-text-only modes.
4.  Enforce book-only answering and chapter citations.
5.  Add out-of-scope rejection and an answer confidence indicator.

## 4. Testing and Validation

- **Content Validation**: All technical claims will be cross-checked with official documentation.
- **Code Validation**: All code will be tested on a fresh Ubuntu 22.04 VM.
- **RAG Validation**: The chatbot will be tested with a set of 20 random questions to verify accuracy and citation.
- **Platform Validation**: The deployed platform will be tested for functionality, performance, and mobile responsiveness.

## 5. Risk Mitigation

- **No RTX GPU**: Switch to a cloud-based Isaac Sim environment.
- **No physical robot**: The capstone will be simulated-only.
- **API quota issues**: Implement a local embedding fallback.
- **Cloud DB downtime**: Maintain a local development backup.