---
id: 0003
title: generate ai robotics textbook tasks
stage: tasks
date: 2025-12-04
surface: agent
model: gemini-1.5-flash
feature: 001-ai-robotics-textbook
branch: 001-ai-robotics-textbook
user: user
command: /sp.tasks
labels: []
links:
  spec: specs/001-ai-robotics-textbook/spec.md
  plan: specs/001-ai-robotics-textbook/plan.md
  tasks: specs/001-ai-robotics-textbook/tasks.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-ai-robotics-textbook/tasks.md
tests:
  - N/A
---

## Prompt

# sp.tasks.md — Atomic Task Execution Plan (Agentic System with Checkpoints)

Version: 2.0  
Last Updated: 2025-12-04  
Architecture: OpenAI Agent SDK + Gemini LLM + Agentic RAG

----------------------------------------------------------------
PURPOSE
----------------------------------------------------------------
This file defines all atomic tasks required to complete the AI-native Physical AI & Humanoid Robotics textbook and the **agent-powered RAG chatbot** using a **strict checkpoint-controlled workflow**.  
Each task:
- Takes 15–45 minutes
- Has one clear output
- One acceptance criterion
- Explicit dependencies
- Requires Git commit after checkpoint approval

----------------------------------------------------------------
PHASE 0: PROJECT INITIALIZATION
----------------------------------------------------------------

### Task 0.1 — Initialize Git Repository & Docusaurus
- Duration: 20 min
- Depends on: None
- Description: Create public Git repository, initialize Docusaurus (TypeScript), add Spec-Kit files.
- Acceptance Criterion:
  "Repo is public, Docusaurus builds cleanly, all sp files present."
- Output: Public GitHub repo with initial commit

---

### Task 0.2 — Verify Spec-Kit Plus + Claude Code
- Duration: 15 min
- Depends on: Task 0.1
- Description: Verify Spec-Kit Plus workflow runs inside the repo.
- Acceptance Criterion:
  "Spec-Kit commands execute without error."
- Output: Verified Spec-Kit environment

---

### ✅ CHECKPOINT 0 — Project Bootstrap Review
Human verifies:
- Repo visibility
- Clean Docusaurus build
- Presence of updated sp.constitution, sp.specify, sp.plan, sp.tasks  
Approval required before Phase 1.

----------------------------------------------------------------
PHASE 1: ARCHITECTURE & AGENT SYSTEM DESIGN
----------------------------------------------------------------

### Task 1.1 — Finalize Table of Contents (TOC)
- Duration: 30 min
- Depends on: ✅ Checkpoint 0
- Description: Define chapter-by-chapter outline mapped to 4 modules + capstone + agent chapter.
- Acceptance Criterion:
  "All modules + capstone + agent systems mapped logically."
- Output: `toc.md`

---

### Task 1.2 — Define Chapter Template (Agent-Compatible)
- Duration: 15 min
- Depends on: Task 1.1
- Description: Create standard chapter structure including agent interaction examples.
- Acceptance Criterion:
  "Template includes objectives, lab, safety, assessment, agent usage."
- Output: `chapter-template.md`

---

### Task 1.3 — Define Capstone Scope (Agent-Integrated)
- Duration: 15 min
- Depends on: Task 1.1
- Description: Define humanoid capstone with VLA + conversational agent.
- Acceptance Criterion:
  "Capstone includes voice, vision, navigation, manipulation, and agent control."
- Output: `capstone-scope.md`

---

### Task 1.4 — Define Agent Roles & Tool Contracts
- Duration: 30 min
- Depends on: Task 1.1
- Description: Specify Retrieval, Verification, Explanation, Assessment, and Safety agents.
- Acceptance Criterion:
  "Each agent has role, tools, memory scope, and failure behavior defined."
- Output: `sp.agent-system.md`

---

### ✅ CHECKPOINT 1 — Architecture & Agent System Review
Human verifies:
- Full module coverage
- Realistic capstone scope
- Clear agent separation of concerns  
Approval required before Phase 2.

----------------------------------------------------------------
PHASE 2: MODULE 0–1 CONTENT (FOUNDATIONS + ROS 2)
----------------------------------------------------------------

### Task 2.1 — Write Foundations of Physical AI
- Duration: 45 min
- Depends on: ✅ Checkpoint 1
- Description: Introduction to Physical AI and embodied intelligence.
- Acceptance Criterion:
  "Concepts clear, verified, beginner-friendly, zero plagiarism."
- Output: `module-0-foundations.md`

---

### Task 2.2 — Write ROS 2 Core Concepts
- Duration: 45 min
- Depends on: Task 2.1
- Description: Nodes, topics, services, actions.
- Acceptance Criterion:
  "All ROS 2 basics explained with verified examples."
- Output: `module-1-ros2-core.md`

---

### Task 2.3 — Create ROS 2 Hands-On Lab
- Duration: 30 min
- Depends on: Task 2.2
- Description: Publisher/subscriber Python lab.
- Acceptance Criterion:
  "Lab runs on Ubuntu 22.04 (ROS 2 Humble)."
- Output: `lab-ros2-basic.md`

---

### ✅ CHECKPOINT 2 — Foundations & ROS Review
Human verifies:
- Technical correctness
- Lab reproducibility
- Beginner clarity  
Approval required before Phase 3.

----------------------------------------------------------------
PHASE 3: MODULE 2 CONTENT (GAZEBO & UNITY)
----------------------------------------------------------------

### Task 3.1 — Write Gazebo Simulation Chapter
- Duration: 45 min
- Depends on: ✅ Checkpoint 2
- Description: Physics simulation, sensors, collisions.
- Acceptance Criterion:
  "Gazebo workflow explained step-by-step."
- Output: `module-2-gazebo.md`

---

### Task 3.2 — Create URDF Modeling Lab
- Duration: 30 min
- Depends on: Task 3.1
- Description: Small humanoid URDF exercise.
- Acceptance Criterion:
  "URDF loads successfully in Gazebo."
- Output: `lab-urdf.md`

---

### Task 3.3 — Write Unity Visualization Chapter
- Duration: 30 min
- Depends on: Task 3.1
- Description: Gazebo → Unity visualization pipeline.
- Acceptance Criterion:
  "Clear visualization integration pipeline."
- Output: `module-2-unity.md`

---

### ✅ CHECKPOINT 3 — Digital Twin Review
Human verifies:
- Sensor simulation
- URDF correctness
- Visualization clarity  
Approval required before Phase 4.

----------------------------------------------------------------
PHASE 4: MODULE 3 CONTENT (NVIDIA ISAAC)
----------------------------------------------------------------

### Task 4.1 — Write Isaac Sim Fundamentals
- Duration: 45 min
- Depends on: ✅ Checkpoint 3
- Description: Omniverse, synthetic data, physics.
- Acceptance Criterion:
  "Isaac Sim workflow documented clearly."
- Output: `module-3-isaac-sim.md`

---

### Task 4.2 — Write Isaac ROS Perception
- Duration: 45 min
- Depends on: Task 4.1
- Description: VSLAM, stereo vision, navigation.
- Acceptance Criterion:
  "Perception pipeline fully explained."
- Output: `module-3-isaac-ros.md`

---

### Task 4.3 — Create Sim-to-Edge Lab
- Duration: 30 min
- Depends on: Task 4.2
- Description: Simulation → Jetson inference pipeline.
- Acceptance Criterion:
  "Inference runs on Jetson Orin class device."
- Output: `lab-sim2edge.md`

---

### ✅ CHECKPOINT 4 — Isaac Platform Review
Human verifies:
- GPU requirements
- Jetson steps realism
- Safety warnings  
Approval required before Phase 5.

----------------------------------------------------------------
PHASE 5: MODULE 4 CONTENT (VISION-LANGUAGE-ACTION)
----------------------------------------------------------------

### Task 5.1 — Write Voice-to-Action Chapter
- Duration: 30 min
- Depends on: ✅ Checkpoint 4
- Description: Speech → ROS 2 command pipeline.
- Acceptance Criterion:
  "Voice pipeline logically complete."
- Output: `module-4-voice.md`

---

### Task 5.2 — Write LLM Cognitive Planning (Agent-Controlled)
- Duration: 45 min
- Depends on: Task 5.1
- Description: Natural language → task planning via agents.
- Acceptance Criterion:
  "Language → ROS actions mapped via agent."
- Output: `module-4-planning.md`

---

### Task 5.3 — Write Multimodal VLA Chapter
- Duration: 30 min
- Depends on: Task 5.2
- Description: Speech + vision + navigation + manipulation.
- Acceptance Criterion:
  "Full VLA pipeline explained."
- Output: `module-4-vla.md`

---

### ✅ CHECKPOINT 5 — VLA Review
Human verifies:
- Agent usage clarity
- ROS integration logic
- Safety enforcement  
Approval required before Phase 6.

----------------------------------------------------------------
PHASE 6: CAPSTONE PROJECT DEVELOPMENT
----------------------------------------------------------------

### Task 6.1 — Write Capstone Architecture
- Duration: 30 min
- Depends on: ✅ Checkpoint 5
- Description: Full agent-powered humanoid system design.
- Acceptance Criterion:
  "Architecture includes perception, planning, control, and agents."
- Output: `capstone-architecture.md`

---

### Task 6.2 — Write Capstone Implementation
- Duration: 45 min
- Depends on: Task 6.1
- Description: Voice → Agent → Planning → Actuation pipeline.
- Acceptance Criterion:
  "Steps reproducible fully in simulation."
- Output: `capstone-implementation.md`

---

### ✅ CHECKPOINT 6 — Capstone Review
Human verifies:
- Logical correctness
- Full reproducibility
- Skill-level alignment  
Approval required before Phase 7.

----------------------------------------------------------------
PHASE 7: AGENTIC RAG SYSTEM DEVELOPMENT
----------------------------------------------------------------

### Task 7.1 — Implement Vector Ingestion Pipeline
- Duration: 30 min
- Depends on: ✅ Checkpoint 6
- Description: Markdown → Chunk → Embed → Store in Qdrant.
- Acceptance Criterion:
  "All chapters indexed with metadata."
- Output: `rag-ingest.py`

---

### Task 7.2 — Implement Agent-Orchestrated Query API
- Duration: 30 min
- Depends on: Task 7.1
- Description: FastAPI + OpenAI Agent SDK orchestration.
- Acceptance Criterion:
  "All responses pass through agents only."
- Output: `agent-api.py`

---

### Task 7.3 — Implement Selected-Text-Only Agent Mode
- Duration: 20 min
- Depends on: Task 7.2
- Description: Restrict RAG context to selected text.
- Acceptance Criterion:
  "Out-of-text questions rejected by safety agent."
- Output: Updated agent pipeline

---

### Task 7.4 — Implement Verification & Citation Agent
- Duration: 30 min
- Depends on: Task 7.2
- Description: Anti-hallucination + forced citation enforcement.
- Acceptance Criterion:
  "All answers include validated chapter citations."
- Output: `verification-agent.py`

---

### ✅ CHECKPOINT 7 — Agentic RAG Review
Human verifies:
- Zero hallucinations
- Mandatory citations
- Selected-text enforcement
- Gemini fallback  
Approval required before Phase 8.

----------------------------------------------------------------
PHASE 8: PERSONALIZATION & SAFETY (BONUS)
----------------------------------------------------------------

### Task 8.1 — Implement User Profile Schema
- Duration: 20 min
- Depends on: ✅ Checkpoint 7
- Description: User learning profile in Neon DB.
- Acceptance Criterion:
  "User preferences stored & retrieved correctly."
- Output: `user-schema.sql`

---

### Task 8.2 — Implement Agent-Based Personalization
- Duration: 30 min
- Depends on: Task 8.1
- Description: Agents adapt output based on user profile.
- Acceptance Criterion:
  "Beginner vs advanced output visibly different."
- Output: Personalized agent pipeline

---

### ✅ CHECKPOINT 8 — Personalization Review
Human verifies:
- No factual distortion
- Core learning flow preserved  
Approval required before Phase 9.

----------------------------------------------------------------
PHASE 9: DEPLOYMENT & DEMO
----------------------------------------------------------------

### Task 9.1 — Deploy Book Frontend
- Duration: 20 min
- Depends on: ✅ Checkpoint 8
- Description: Deploy Docusaurus to GitHub Pages / Vercel.
- Acceptance Criterion:
  "Public book URL accessible globally."
- Output: Live book link

---

### Task 9.2 — Deploy Backend & Agents
- Duration: 20 min
- Depends on: Task 9.1
- Description: Deploy FastAPI + Agents + Qdrant + Neon.
- Acceptance Criterion:
  "Live agent chatbot responds correctly."
- Output: Live API endpoint

---

### Task 9.3 — Record 90-Second Demo Video
- Duration: 30 min
- Depends on: Task 9.2
- Description: Show book, agent chatbot, one simulation.
- Acceptance Criterion:
  "Video under 90 seconds, clear and reproducible."
- Output: Demo video link

---

### ✅ CHECKPOINT 9 — FINAL SUBMISSION REVIEW
Human verifies:
- Book quality
- Agent correctness
- Demo clarity  
Final commit marks project completion.

----------------------------------------------------------------
DEPENDENCY SUMMARY
----------------------------------------------------------------
0.1 → 0.2 → ✅0  
✅0 → 1.1 → 1.2 → 1.3 → 1.4 → ✅1  
✅1 → 2.1 → 2.2 → 2.3 → ✅2  
✅2 → 3.1 → 3.2 → 3.3 → ✅3  
✅3 → 4.1 → 4.2 → 4.3 → ✅4  
✅4 → 5.1 → 5.2 → 5.3 → ✅5  
✅5 → 6.1 → 6.2 → ✅6  
✅6 → 7.1 → 7.2 → 7.3 → 7.4 → ✅7  
✅7 → 8.1 → 8.2 → ✅8  
✅8 → 9.1 → 9.2 → 9.3 → ✅9  

----------------------------------------------------------------
EXECUTION RULE (STRICT)
----------------------------------------------------------------
No task may begin unless:
1. All dependencies are complete
2. The previous checkpoint is approved by a human
3. Approved work is committed to Git with a clear message

## Response snapshot

The `tasks.md` file has been successfully generated at `specs/001-ai-robotics-textbook/tasks.md`.

**Summary of Task Generation:**
*   **Total Tasks**: 27
*   **Tasks per User Story**:
    *   User Story 1 (Learn Physical AI from Zero to Capstone): 17 tasks
    *   User Story 2 (Interact with the Agentic RAG Chatbot): 4 tasks
    *   User Story 3 (Access Reproducible Code Examples): Tasks related to reproducibility are integrated throughout the content and lab creation tasks.
*   **Parallel Opportunities**: 23 tasks are marked as parallelizable.
*   **Independent Test Criteria**: Defined in `specs/001-ai-robotics-textbook/spec.md`.
*   **Suggested MVP Scope**: User Story 1 (Learning Path) and User Story 2 (RAG Chatbot) form the core MVP, leading to a functional textbook with an interactive agent.

The project is now ready for execution based on these defined tasks.

## Outcome

- ✅ Impact: Detailed, dependency-ordered tasks generated for the feature.
- 🧪 Tests: N/A
- 📁 Files: Generated `tasks.md`.
- 🔁 Next prompts: Ready for implementation of tasks.
- 🧠 Reflection: Successfully generated granular, dependency-ordered tasks following the strict checklist format, based on the spec, plan, and other design documents. Managed to integrate user stories and parallelization markers effectively.

## Evaluation notes (flywheel)

- Failure modes observed: Continued to encounter a warning about multiple spec directories with the same prefix during `check-prerequisites.sh` execution. This did not prevent task generation but is a recurring issue.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): None immediately, as task generation is complete. The next step is execution of these tasks.