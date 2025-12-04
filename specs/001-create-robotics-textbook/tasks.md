# Atomic Task Execution Plan: AI-Native Textbook

## Phase 0: Project Initialization (30–45 minutes)

- [ ] T001 Initialize Git repository and Docusaurus project in `/`
- [ ] T002 Verify Spec-Kit + Claude Code integration in `/`

**CHECKPOINT 0 — Project Bootstrap Review**

## Phase 1: Book Architecture & Outline (60–90 minutes)

- [X] T101 Define full table of contents in `specs/001-create-robotics-textbook/toc.md`
- [X] T102 Define chapter template in `specs/001-create-robotics-textbook/chapter-template.md`
- [X] T103 Define capstone project scope in `specs/001-create-robotics-textbook/capstone-scope.md`

**CHECKPOINT 1 — Architecture & Scope Review**

## Phase 2: Module 0–1 Content Generation (Foundations + ROS 2)

- [X] T201 [US1] Write Foundations of Physical AI chapter in `content/module-0-foundations.md`
- [X] T202 [US1] Write ROS 2 Core Concepts chapter in `content/module-1-ros2-core.md`
- [X] T203 [US1] Create ROS 2 Hands-On Lab in `content/lab-ros2-basic.md`

**CHECKPOINT 2 — Foundations & ROS 2 Review**

## Phase 3: Module 2 Content (Gazebo & Unity Digital Twin)

- [X] T301 [US2] Write Gazebo Simulation chapter in `content/module-2-gazebo.md`
- [X] T302 [US2] Create URDF Robot Modeling Lab in `content/lab-urdf.md`
- [X] T303 [US2] Write Unity Visualization chapter in `content/module-2-unity.md`

**CHECKPOINT 3 — Digital Twin Review**

## Phase 4: Module 3 Content (NVIDIA Isaac AI Brain)

- [X] T401 [US3] Write Isaac Sim Fundamentals chapter in `content/module-3-isaac-sim.md`
- [X] T402 [US3] Write Isaac ROS Perception chapter in `content/module-3-isaac-ros.md`
- [X] T403 [US3] Create Sim-to-Real Transfer Lab in `content/lab-sim2real.md`

**CHECKPOINT 4 — Isaac Platform Review**

## Phase 5: Module 4 Content (Vision-Language-Action)

- [X] T501 [US4] Write Voice-to-Action chapter in `content/module-4-voice.md`
- [X] T502 [US4] Write LLM Cognitive Planning chapter in `content/module-4-planning.md`
- [X] T503 [US4] Write Multimodal Interaction chapter in `content/module-4-vla.md`

**CHECKPOINT 5 — VLA Review**

## Phase 6: Capstone Project Development

- [X] T601 [US4] Write Capstone Architecture in `content/capstone-architecture.md`
- [X] T602 [US4] Write Step-by-Step Capstone Implementation in `content/capstone-implementation.md`

**CHECKPOINT 6 — Capstone Review**

## Phase 7: RAG Chatbot Development

- [X] T701 Implement Vector Ingestion Pipeline in `backend/rag-ingest.py`
- [X] T702 Implement RAG Query API in `backend/rag-api.py`
- [X] T703 Implement Selected-Text-Only Mode in `backend/rag-api.py`

**CHECKPOINT 7 — RAG Review**

## Phase 8: Deployment & Demo

- [ ] T801 Deploy Book to GitHub Pages / Vercel
- [ ] T802 Deploy Backend APIs
- [ ] T803 Record 90-Second Demo Video

**CHECKPOINT 8 — Final Submission Review**
