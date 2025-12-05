# Implementation Plan: AI-Native Textbook on Physical AI & Humanoid Robotics

**Branch**: `001-ai-robotics-textbook` | **Date**: December 4, 2025 | **Spec**: specs/001-ai-robotics-textbook/spec.md
**Input**: Feature specification from `specs/001-ai-robotics-textbook/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This project aims to create a comprehensive AI-native textbook on Physical AI and Humanoid Robotics, delivered as an interactive learning platform. The platform will feature a Docusaurus static site for content and an embedded agent-powered RAG chatbot for enhanced learning. The technical approach involves an OpenAI Agent SDK orchestrated backend with Gemini LLM for reasoning, Qdrant for vector search, and Neon Serverless PostgreSQL for user data. The plan includes detailed content architecture, agentic RAG system development, and a focus on reproducible, simulation-first labs culminating in a humanoid robotics capstone project.

## Technical Context

**Language/Version**: Python 3.x (primary), C++ (optional), JavaScript/TypeScript (Docusaurus)
**Primary Dependencies**: FastAPI, OpenAI Agent SDK, Gemini LLM, Qdrant Client, Neon Serverless PostgreSQL client libraries, Docusaurus, React
**Storage**: Neon Serverless PostgreSQL (users, logs, sessions, user preferences), Qdrant Cloud (vector embeddings for RAG)
**Testing**: Pytest (backend), Playwright/Cypress (frontend E2E), unit tests for agents. All code examples will be verified on Ubuntu 22.04.
**Target Platform**: Web application (Frontend deployed on GitHub Pages/Vercel, Backend on Railway/Render/Fly.io)
**Project Type**: Web application (Frontend + Backend)
**Performance Goals**:
- RAG chatbot response latency: under 3 seconds for 90% of requests (for personalized responses).
- Website load time: standard static site performance.
**Constraints**:
- Content Length: 8,000–15,000 words.
- Format: Markdown source for Docusaurus.
- Operating System: Ubuntu 22.04 LTS for all code examples and setup.
- Simulation First: All projects must be testable without physical robots.
- Timeline: Must be completed before Nov 30, 2025.
- Free-tier cloud services must be sufficient for live demo.
- No direct frontend calls to Gemini APIs.
- AI interactions exclusively via OpenAI Agent SDK.
- LLM Provider: Gemini.
- Vector Search: Qdrant Cloud (Free Tier).
- User Data: Neon Serverless PostgreSQL.
**Scale/Scope**:
- Covers 4 official modules, 12-15 chapters, 20+ labs, 1 capstone project.
- Supports agentic RAG chatbot features (in-book answers, citation, out-of-scope refusal).

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Technical accuracy over assumptions**: This plan prioritizes verification of facts and code.
- **Clarity for beginners without oversimplifying**: Content architecture emphasizes clear learning objectives and step-by-step labs.
- **Practical, hands-on learning orientation**: Strong emphasis on labs and a capstone project.
- **Simulation-first before real hardware**: Explicitly called out as a constraint and supported.
- **Ethical and safe use of AI and robotics**: Safety & Refusal agent, and safety notes in chapters.
- **AI-native learning (content + chatbot + personalization)**: Core to the project definition and architecture.
- **Open-source first mindset**: Project deliverables include a public GitHub repository.
- **All technical explanations must be verifiable from trusted sources**: Addressed in research approach.
- **Code examples must be runnable and tested**: Addressed in testing strategy.
- **ROS 2 examples must target Ubuntu 22.04 LTS**: Explicit constraint.
- **Python will be the primary programming language**: Explicit constraint.
- **All hardware references must include realistic alternatives**: Addressed in content structure.
- **Safety warnings must be included for physical robotics**: Addressed in content structure.
- **Every chapter must include**: Learning objectives, Concept explanation, Practical exercise, Summary, Assessment or mini-project. Addressed in content structure.
- **Simple language, no unnecessary jargon**: Guiding principle for content generation.
- **No plagiarism (0% tolerance)**: Part of quality metrics.
- **AI-generated text must be reviewed and verified**: Part of quality metrics.
- **Avoid vendor lock-in where possible**: Considered, but some choices (Gemini, Qdrant, Neon) are explicitly constrained.
- **Real-world use cases must be included**: Implicit in industry-aligned approach.
- **Book Framework: Docusaurus**: Explicit constraint.
- **Chatbot: OpenAI Agents / ChatKit SDK**: Explicit constraint (OpenAI Agent SDK).
- **Backend: FastAPI**: Explicit constraint.
- **Database: Neon Serverless PostgreSQL**: Explicit constraint.
- **Vector Store: Qdrant Cloud (Free Tier)**: Explicit constraint.
- **Deployment: GitHub Pages or Vercel**: Explicit constraint.
- **RAG chatbot requirements (all met)**: Must answer only from the book content, support question answering from user-selected text, cite the chapter, reject out-of-scope questions.

All principles and standards from the constitution are addressed or justified by explicit user constraints.

## Project Structure

### Documentation (this feature)

```text
specs/001-ai-robotics-textbook/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── agents/          # OpenAI Agent SDK implementations (Retrieval, Verification, etc.)
│   ├── api/             # FastAPI endpoints for chatbot, user management, etc.
│   ├── models/          # Data models for user preferences, session data, etc.
│   ├── services/        # Integrations with Qdrant, Neon, Gemini API
│   └── core/            # Core RAG logic, utilities
└── tests/               # Unit and integration tests for backend

content/                 # Docusaurus Markdown content (chapters, labs)
├── sections/
│   ├── 0-foundations/
│   ├── 1-ros2/
│   ├── ...
│   └── 8-agentic-ai/
└── assets/              # Images, videos, etc.

frontend/                # Docusaurus site
├── src/
│   ├── components/      # React components (Homepage, Chatbot UI, etc.)
│   ├── pages/           # Docusaurus pages
│   ├── css/             # Custom CSS
│   └── hooks/           # React hooks for state management, API calls
└── static/              # Static assets for Docusaurus

# Other top-level files
├── docusaurus.config.ts
├── package.json
├── sidebars.ts
├── tsconfig.json
├── README.md
├── .gitignore
```

**Structure Decision**: The project will follow a monorepo structure with `backend/` for the FastAPI server and agent logic, `frontend/` for the Docusaurus application, and `content/` for the textbook's Markdown source. This aligns with the "Web application" option and clearly separates concerns.

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|---|---|---|
| Multiple Spec Directories (001-ai-robotics-textbook and 001-create-robotics-textbook) | New feature requires its own spec. | Consolidating would mix concerns of two distinct features, making tracking and development harder. |