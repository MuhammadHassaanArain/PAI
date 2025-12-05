---
id: 0001
title: create ai robotics textbook spec
stage: spec
date: 2025-12-04
surface: agent
model: gemini-1.5-flash
feature: 001-ai-robotics-textbook
branch: 001-ai-robotics-textbook
user: user
command: /sp.specify
labels: []
links:
  spec: specs/001-ai-robotics-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-ai-robotics-textbook/spec.md
  - specs/001-ai-robotics-textbook/checklists/requirements.md
tests:
  - N/A
---

## Prompt

Project: AI-Native Textbook on Physical AI & Humanoid Robotics (Agent-Orchestrated with Gemini LLM)

Target Audience:
- University students in AI, Robotics, and Mechatronics
- Software engineers transitioning into robotics
- AI practitioners learning Physical AI
- Technical educators and professional trainers
- Hackathon and research-based robotics learners

Primary Focus:
- Teaching Physical AI and embodied intelligence from zero to capstone level
- Bridging digital AI with real-world robotics using simulation-first workflows
- Hands-on learning with ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action (VLA) systems
- Preparing students for industry-grade humanoid robotics development
- Introducing **agentic AI systems for robotics learning using OpenAI Agent SDK with Gemini LLM**

Success Criteria:
- Covers all four official modules:
  1. ROS 2 (Robot Nervous System)
  2. Gazebo & Unity (Digital Twin)
  3. NVIDIA Isaac (AI-Robot Brain)
  4. Vision-Language-Action (VLA)
- Includes at least:
  - 12–15 instructional chapters
  - 20+ hands-on labs and guided exercises
  - 1 complete humanoid robotics capstone project
- Agentic RAG chatbot can:
  - Answer questions only from book content
  - Answer questions based on user-selected text
  - Cite source chapter and section
  - Reject out-of-scope questions
- Students can:
  - Build ROS 2 packages independently
  - Run humanoid simulations successfully
  - Deploy inference pipelines to Jetson-class edge devices
  - Interact with agent-powered robotics tutors
- Book is successfully deployed on GitHub Pages or Vercel
- All code is reproducible on a fresh Ubuntu 22.04 system
- OpenAI Agent SDK successfully orchestrates Gemini as the reasoning model

Core AI & Agent Capabilities:
- All AI interactions must be handled through OpenAI Agent SDK
- Gemini must serve as the primary reasoning and generation LLM
- No direct frontend calls to Gemini APIs are permitted
- Agent system must include:
  - Retrieval Agent (RAG orchestration)
  - Verification Agent (anti-hallucination & citation checks)
  - Explanation Agent (pedagogical response generation)
  - Assessment Agent (quiz, grading, feedback)
  - Safety & Refusal Agent (unsafe prompt filtering)
- Agents must support:
  - Tool calling
  - Memory (short-term session memory)
  - Deterministic fallback behavior when quota is exceeded
- Agent prompts, tools, and memory scopes must be auditable

Constraints:
- Content Length: 8,000–15,000 words
- Format: Markdown source for Docusaurus
- Language: English (Urdu translation optional as bonus)
- Programming Language: Python (primary), C++ optional
- Operating System: Ubuntu 22.04 LTS
- Simulation First: All projects must be testable without physical robots
- Hardware Dependency: Must provide both on-prem and cloud alternatives
- Timeline: Must be completed before Nov 30, 2025
- Free-tier cloud services must be sufficient for live demo

Evidence & Validation Standards:
- All robotics and AI claims must be technically verifiable
- Vendor documentation (ROS, NVIDIA, Gazebo, Isaac) allowed as primary references
- No unverified performance claims
- All safety-critical steps must be clearly marked
- All agent behaviors must be deterministic, logged, and debuggable
- Code examples must be:
  - Executed OR
  - Verified through dry-run testing
- No hallucinated citations or fabricated APIs are allowed

RAG, Agent & Platform Constraints:
- Backend must be FastAPI-based
- Agent orchestration must use OpenAI Agent SDK
- LLM Provider must be Gemini (via API key through agents)
- Vector search must use Qdrant (Cloud Free Tier)
- User data must be stored in Neon Serverless PostgreSQL
- Authentication (if implemented) must use Better-Auth
- Chatbot must not use any external knowledge beyond the book corpus
- All retrieval must be:
  - Chunked
  - Embedded
  - Version-controlled
- All agent outputs must carry:
  - Retrieval trace
  - Source chunk ID
  - Chapter citation

Personalization Constraints (Bonus Scope):
- Personalization must be rule-based or agent-based
- User background questions limited to:
  - Programming level
  - AI knowledge level
  - Hardware availability
  - Learning speed preference
- Personalized responses must:
  - Not modify core technical facts
  - Not break the canonical learning path
- Gemini outputs must be adapted in tone and depth via agent control
- Urdu localization must preserve:
  - Mathematical meaning
  - Robotics terminology
  - Command-line accuracy

Security, Privacy & Safety:
- No user data may be logged without explicit consent
- No unsafe robot control instructions without warnings
- No real-world hazardous actuation steps without simulation gating
- AI agents must enforce:
  - Refusal of unsafe queries
  - Refusal of weapons or military use cases
- Prompt injection protection must be implemented at agent layer
- All API keys must be stored in environment variables

Not Building:
- A general-purpose robotics encyclopedia
- A full mathematical robotics derivation textbook
- Commercial production-ready robot firmware
- Military or surveillance robotics systems
- Proprietary or closed-source integrations
- Full humanoid hardware manufacturing manuals

Out of Scope:
- Ethical, legal, and societal impacts (future volume only)
- Advanced control theory proofs
- Low-level motor driver electronics
- FPGA or custom silicon development
- Medical or surgical robotics
- Real-world autonomous weapon systems

Deliverables:
- Public GitHub repository with:
  - Full Docusaurus source
  - All Markdown chapters
  - Agent orchestration layer
  - FastAPI backend
- Live deployed book link
- Working embedded agentic RAG chatbot
- Reproducible setup instructions
- Vector DB initialization scripts
- 90-second demo video

Final Outcome:
A complete AI-native, agent-powered, hands-on, industry-aligned textbook and learning platform that enables students to go from zero knowledge to building a **simulated conversational humanoid robot using Physical AI principles**, with all learning interactions orchestrated through **OpenAI Agent SDK and powered by Gemini LLM**.

## Response snapshot

The specification for the "AI-Native Textbook on Physical AI & Humanoid Robotics" has been created at `specs/001-ai-robotics-textbook/spec.md`. A corresponding quality checklist has also been generated at `specs/001-ai-robotics-textbook/checklists/requirements.md` and updated with validation results, noting minor deviations due to explicit user-defined technology constraints. The feature branch `001-ai-robotics-textbook` has been created.

## Outcome

- ✅ Impact: Specification for new feature created and validated.
- 🧪 Tests: N/A
- 📁 Files: Created `specs/001-ai-robotics-textbook/spec.md` and `specs/001-ai-robotics-textbook/checklists/requirements.md`.
- 🔁 Next prompts: Ready for `/sp.plan` or `/sp.clarify` if needed.
- 🧠 Reflection: Initial issues with `grep` and `bash` script execution on Windows were resolved by explicitly calling `bash` and using PowerShell equivalents where necessary. The spec was generated and validated against the checklist, incorporating user-provided constraints on technology.

## Evaluation notes (flywheel)

- Failure modes observed: Initial attempts to run `grep` and the `create-new-feature.sh` script failed due to environment differences (PowerShell vs. Bash, and script execution methods).
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Improve cross-platform compatibility of shell scripts or provide PowerShell equivalents directly in the `.specify` tools.