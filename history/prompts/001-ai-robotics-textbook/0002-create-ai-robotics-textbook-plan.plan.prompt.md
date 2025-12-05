---
id: 0002
title: create ai robotics textbook plan
stage: plan
date: 2025-12-04
surface: agent
model: gemini-1.5-flash
feature: 001-ai-robotics-textbook
branch: 001-ai-robotics-textbook
user: user
command: /sp.plan
labels: []
links:
  spec: specs/001-ai-robotics-textbook/spec.md
  plan: specs/001-ai-robotics-textbook/plan.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-ai-robotics-textbook/plan.md
  - specs/001-ai-robotics-textbook/research.md
  - specs/001-ai-robotics-textbook/data-model.md
  - specs/001-ai-robotics-textbook/contracts/api-v1.md
  - specs/001-ai-robotics-textbook/quickstart.md
  - GEMINI.md
tests:
  - N/A
---

## Prompt

Version: 2.0  
Last Updated: 2025-12-04  
Architecture: OpenAI Agent SDK + Gemini LLM + Agentic RAG

------------------------------------
PROJECT CREATION SCOPE
------------------------------------
Create:
- Full book architecture (Docusaurus + FastAPI + Agentic RAG)
- Detailed chapter & module structure
- Hands-on lab framework
- Agent-controlled RAG ingestion + retrieval pipeline
- Safety, verification, and citation control via agents
- Personalization & translation flow (bonus scope)
- Capstone project pipeline (Simulation → Edge → Humanoid)

------------------------------------
EXECUTION PHASES (AGENT-FIRST)
------------------------------------
1. Specification Lock (sp.constitution + sp.specify + sp.plan)
2. Agent System Architecture Design
3. Content Architecture Design
4. Chapter-by-Chapter Generation
5. Agentic RAG System Development
6. Personalization & Auth (Bonus)
7. Deployment & Validation
8. Demo & Submission

All phases require checkpoint approval before moving forward.

------------------------------------
ARCHITECTURE SKETCH (AGENTIC)
------------------------------------
Frontend:
- Docusaurus static site
- Embedded agent-powered chatbot UI
- Optional login & user profile UI
- Selected-text interaction overlay

Backend:
- FastAPI server
- OpenAI Agent SDK (orchestration layer)
- Gemini LLM (via agent abstraction)
- Neon Serverless PostgreSQL (users, logs, sessions)
- Qdrant Cloud (vector embeddings)

Agent Layer:
- Retrieval Agent (RAG orchestration)
- Verification Agent (anti-hallucination)
- Explanation Agent (pedagogical generation)
- Assessment Agent (quizzes & feedback)
- Safety & Refusal Agent (unsafe queries)

AI Pipeline (Agent-Controlled):
- Markdown → Chunking → Embedding → Qdrant
- User query → Agent → Vector search → Context → Gemini → Verification → Final Answer
- Selected-text-only mode → Restricted agent context
- Fallback routing when Gemini quota is exceeded

Deployment:
- Frontend: GitHub Pages / Vercel
- Backend: Railway / Render / Fly.io
- Databases: Cloud-hosted only
- Secrets: Environment variables only

------------------------------------
SECTION & MODULE STRUCTURE
------------------------------------
Book Sections:
- Section 0: Foundations of Physical AI
- Section 1: ROS 2 – Robotic Nervous System
- Section 2: Digital Twin – Gazebo & Unity
- Section 3: AI Brain – NVIDIA Isaac
- Section 4: Vision-Language-Action Systems
- Section 5: Capstone – Autonomous Humanoid
- Section 6: Cloud vs On-Prem Lab Setup
- Section 7: Safety, Ethics & Responsible Deployment
- Section 8: Agentic AI for Robotics Learning (New)

Each Chapter Must Include:
- Learning Objectives
- Core Concepts
- Step-by-Step Lab
- Code Examples
- Hardware / Cloud Alternative
- Safety Notes
- Summary
- Assessment / Mini Project
- Agent Interaction Examples (where applicable)

------------------------------------
RESEARCH APPROACH
------------------------------------
- Research runs in parallel with writing
- Primary sources:
  - ROS 2 official docs
  - Gazebo documentation
  - NVIDIA Isaac documentation
  - OpenAI Agent SDK specs
  - Gemini API documentation
- Secondary sources:
  - Robotics textbooks
  - IEEE research papers
- Mandatory verification:
  - All commands
  - All installation steps
  - All code snippets
  - All agent tool calls

------------------------------------
DECISIONS REQUIRING DOCUMENTATION
------------------------------------
1. ROS Distribution:
   - Option A: Humble (LTS)
   - Option B: Iron
   - Tradeoff: Stability vs latest features

2. Simulation Priority:
   - Option A: Gazebo-first
   - Option B: Isaac Sim-first
   - Tradeoff: Accessibility vs realism

3. Cloud vs On-Prem Default:
   - Option A: Local workstation
   - Option B: Cloud GPU
   - Tradeoff: Cost vs control

4. Capstone Complexity:
   - Option A: Navigation + Vision only
   - Option B: Full VLA + manipulation
   - Tradeoff: Reproducibility vs demo impact

5. Agent Autonomy Level:
   - Option A: Strict tool-only responses
   - Option B: Hybrid reasoning + tools
   - Tradeoff: Safety vs flexibility

6. Personalization Depth:
   - Option A: Explanations only
   - Option B: Explanations + Labs
   - Tradeoff: Complexity vs learner value

All final choices must be frozen before large-scale generation.

------------------------------------
CONTENT GENERATION STRATEGY
------------------------------------
- Generate one module at a time
- Validate each module before starting the next
- Lock:
  - Commands
  - APIs
  - Simulation assets
  - Agent prompts
- Freeze earlier chapters after validation
- Maintain full changelog with version tags
- Track AI-assisted vs human-written content

------------------------------------
AGENTIC RAG DEVELOPMENT PLAN
------------------------------------
1. Convert all validated chapters to structured chunks
2. Generate embeddings using approved embedding model
3. Store in Qdrant with:
   - Chapter ID
   - Section ID
   - Version metadata
4. Implement agent workflow:
   - Query → Retrieval Agent
   - Context → Verification Agent
   - Response → Explanation Agent
5. Enforce:
   - Book-only answering
   - Mandatory citation
   - Out-of-scope rejection
6. Add:
   - Safety filtering
   - Gemini quota fallback
   - Retrieval trace logging

------------------------------------
PERSONALIZATION & TRANSLATION PLAN (BONUS)
------------------------------------
Personalization Inputs:
- Programming level
- AI knowledge level
- Hardware availability
- Learning speed preference

Adaptable Content:
- Explanation depth
- Lab complexity
- Simulation vs cloud emphasis

Urdu Translation:
- Button-based per chapter
- Preserve:
  - Robotics terminology
  - CLI commands
  - Mathematical expressions
- Agent-assisted translation with human review

------------------------------------
TESTING & VALIDATION STRATEGY
------------------------------------
Content Validation:
- All facts cross-checked with official docs
- All safety warnings reviewed
- All assumptions explicitly logged

Code Validation:
- Fresh Ubuntu 22.04 VM test
- Dry-run validation of:
  - ROS 2 installation
  - Gazebo simulation
  - Isaac basic scenes
  - VLA inference pipeline
- Python linting where applicable

Agent Validation:
- Prompt injection testing
- Hallucination stress testing
- Tool-call failure testing
- Gemini quota failure testing

RAG Validation:
- Ask at least 20 randomized questions
- Verify:
  - Correct chapter citation
  - Zero hallucinated content
  - Selected-text restriction enforcement

Platform Validation:
- Login/logout flow
- Chat persistence
- Vector retrieval latency
- Mobile responsiveness
- Deployed API health checks

------------------------------------
ACCEPTANCE CRITERIA (GO / NO-GO)
------------------------------------
- Book builds without errors
- Deployed site loads globally
- Agentic chatbot answers from book only
- Mandatory citations enforced
- At least one full module runnable end-to-end
- One agent-driven assessment functional
- Capstone walkthrough logically complete
- Demo video shows:
  - Book
  - Agent chatbot
  - One full simulation
- Zero plagiarism confirmed

------------------------------------
RISK MITIGATION
------------------------------------
- No RTX GPU → switch to cloud Isaac
- No physical robot → simulation-only capstone
- Gemini API quota issues → fallback model via agent
- Qdrant downtime → local vector dev mode
- Backend crash → stateless recovery via logs
- Prompt injection risk → safety agent hard refusals

------------------------------------
FINAL OUTPUT
------------------------------------
A complete, validated, agent-powered AI-native Physical AI & Humanoid Robotics textbook with:
- Published learning portal
- Embedded agentic RAG chatbot
- Reproducible labs
- Industry-aligned capstone project
- Fully auditable AI agent orchestration

## Response snapshot

The implementation plan for the "AI-Native Textbook on Physical AI & Humanoid Robotics" feature has been successfully created. This involved:
- Running the `setup-plan.sh` script to retrieve necessary paths.
- Populating the `specs/001-ai-robotics-textbook/plan.md` file with a summary, technical context, constitution check, and project structure based on the user's prompt and constitution.
- Resolving "DECISIONS REQUIRING DOCUMENTATION" by creating `specs/001-ai-robotics-textbook/research.md` with reasoned decisions.
- Generating `specs/001-ai-robotics-textbook/data-model.md` to define key data entities.
- Creating API contracts in `specs/001-ai-robotics-textbook/contracts/api-v1.md`.
- Providing a high-level `specs/001-ai-robotics-textbook/quickstart.md` guide.
- Updating the agent context in `GEMINI.md` with relevant technical details.
The plan is now complete, and all required design artifacts have been generated.

## Outcome

- ✅ Impact: Implementation plan created, architectural decisions resolved, and design artifacts generated.
- 🧪 Tests: N/A
- 📁 Files: Generated `plan.md`, `research.md`, `data-model.md`, `api-v1.md`, `quickstart.md`, and updated `GEMINI.md`.
- 🔁 Next prompts: Ready for task breakdown (`/sp.tasks`).
- 🧠 Reflection: Successfully navigated the planning workflow, made architectural decisions, and generated design artifacts based on user input and constitution. Handled script execution for agent context update.

## Evaluation notes (flywheel)

- Failure modes observed: Encountered a warning about multiple spec directories with the same prefix ("001-ai-robotics-textbook" and "001-create-robotics-textbook") during `setup-plan.sh` and `update-agent-context.sh` execution. This issue did not prevent the successful generation of artifacts but was noted in the `plan.md`'s "Complexity Tracking" section. This should be addressed by the user in future steps to prevent potential conflicts or confusion with feature numbering.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Propose a solution for handling conflicting spec directories, possibly by suggesting renaming or archiving the older conflicting directory.