# Project Plan: AI-Native Textbook on Physical AI & Humanoid Robotics

This document outlines the project plan based on the clarifications of the project specification.

## 1. Target Audience and Content

- **Target Audience**: The textbook is for individuals with a solid understanding of Python but who are new to robotics and AI. It will not cover Python basics.
- **Content Focus**: The curriculum will focus on system integration—teaching students how to combine existing AI models and robotics tools to build a complete system.
- **Learning Style**: The book will adopt a "practical textbook" approach, merging conceptual explanations with hands-on, project-based learning to build a strong portfolio.

## 2. Technical and Environmental Decisions

- **Development Environment**: Docker will be the officially supported environment to ensure consistency.
- **Operating System**: All content and code will be tailored for Ubuntu 22.04 LTS. Windows and other operating systems are not officially supported.
- **Hardware Requirements**: All core exercises will be achievable in a simulated environment. A machine with an NVIDIA GPU (at least 8GB of VRAM) is required.
- **ROS 2 Version**: All ROS 2 examples and projects will use ROS 2 Humble Hawksbill.

## 3. Content Structure and Deliverables

- **Structure**: The textbook will be divided into 4 modules, containing a total of 12-15 chapters.
- **Chapter Content**: Each chapter will include learning objectives, concept explanations, a hands-on coding exercise, a summary, and a mini-project or assessment questions.
- **Capstone Project**: The mandatory capstone project will involve building a simulated humanoid robot that can navigate, perceive objects, and perform a pick-and-place task based on natural language commands.

## 4. RAG Chatbot and Platform

- **Chatbot Functionality**: The RAG chatbot will only answer questions based on the textbook's content. It will not generate new code but will provide summaries and citations.
- **Platform**: The textbook will be deployed online using Docusaurus on Vercel or GitHub Pages.

## 5. Evaluation

- **Scoring**: Project evaluation will be weighted 60% on engineering quality and 40% on content quality.
- **Bonus Features**: UI/UX, chatbot accuracy, and advanced features like voice input will be considered bonus components.
