---
id: 0001
title: Update Project Constitution
stage: constitution
date: 2025-12-04
surface: agent
model: gemini-pro
feature: none
branch: master
user: User
command: sp.constitution
labels: [constitution]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- .specify/memory/constitution.md
tests:
null
---

## Prompt

@.gemini/commands/sp.constitution.md Project: AI-Native Textbook on Physical AI & Humanoid Robotics

Purpose:
To create a production-quality, AI-native technical textbook that teaches Physical AI & Humanoid Robotics using ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action systems, and deploy it as an interactive learning platform with an embedded RAG chatbot.

Target Audience:
- University-level students
- Robotics and AI beginners
- Engineers transitioning into robotics
- Self-learners with Python and basic AI knowledge

Core Principles:
- Technical accuracy over assumptions
- Clarity for beginners without oversimplifying
- Practical, hands-on learning orientation
- Simulation-first before real hardware
- Ethical and safe use of AI and robotics
- AI-native learning (content + chatbot + personalization)
- Open-source first mindset

Key Standards:
- All technical explanations must be verifiable from trusted sources
- Code examples must be runnable and tested
- ROS 2 examples must target Ubuntu 22.04 LTS
- Python will be the primary programming language
- All hardware references must include realistic alternatives
- Safety warnings must be included for physical robotics
- Every chapter must include:
  - Learning objectives
  - Concept explanation
  - Practical exercise
  - Summary
  - Assessment or mini-project

Writing & Content Rules:
- Simple language, no unnecessary jargon
- Step-by-step explanations
- Diagrams preferred over long theory where possible
- No plagiarism (0% tolerance)
- AI-generated text must be reviewed and verified
- Avoid vendor lock-in where possible
- Real-world use cases must be included

Technical Stack Requirements:
- Book Framework: Docusaurus
- Content Generation: Claude Code + Spec-Kit Plus
- Chatbot: OpenAI Agents / ChatKit SDK
- Backend: FastAPI
- Database: Neon Serverless PostgreSQL
- Vector Store: Qdrant Cloud (Free Tier)
- Deployment: GitHub Pages or Vercel

RAG Chatbot Requirements:
- Must answer only from the book content
- Must support question answering from user-selected text
- Must cite the chapter it answers from
- Must reject out-of-scope questions
- Must support logged-in user personalization (if implemented)

Personalization & Localization Rules (Bonus Features):
- User background must influence explanations
- Beginner vs advanced users should see different depth
- Urdu translation must preserve technical meaning
- Content must remain consistent across all languages

Constraints:
- Must follow the official hackathon course outline
- Must cover all 4 modules in sequence
- Must include a complete Capstone Project
- Must be deployable on low-budget setups using cloud alternatives
- Demo video must be under 90 seconds
- Entire project must be public and open-source

Assessment Requirements:
- ROS 2 package development project
- Gazebo simulation project
- Isaac perception pipeline project
- Full humanoid simulation capstone with conversational AI

Success Criteria:
- Book deploys successfully on GitHub Pages or Vercel
- RAG chatbot answers correctly from book content
- All modules match the an official course outline
- All code runs without critical errors
- Zero plagiarism confirmed
- Clear beginner-to-advanced learning progression
- Judges can reproduce the capstone demo
- Project is suitable for real classroom deployment

Quality Metrics:
- Technical accuracy verified
- Code execution success rate > 95%
- Readability at Flesch-Kincaid grade 10–12
- Clear separation between theory and practice
- No broken links or missing assets

Ethics & Safety:
- No unsafe robot control instructions without warnings
- No military or weaponized use cases
- Emphasis on human safety and responsible AI
- Dataset and simulation bias must be discussed where relevant

Final Objective:
To deliver a complete, AI-native, industry-ready Physical AI & Humanoid Robotics textbook with an embedded intelligent chatbot that can be used for real-world education, training, and startup incubation.

## Response snapshot

Updated the project constitution at `.specify/memory/constitution.md` and created a Prompt History Record.

## Outcome

- ✅ Impact: The project constitution has been updated with the new principles and standards.
- 🧪 Tests: No tests were run.
- 📁 Files: 
  - `history/prompts/constitution/0001-update-project-constitution.constitution.prompt.md`
- 🔁 Next prompts: null
- 🧠 Reflection: The `create-phr.sh` script is not reliable on Windows. Creating the PHR file manually is a better approach.

## Evaluation notes (flywheel)

- Failure modes observed: The `create-phr.sh` script failed to create the PHR file.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
