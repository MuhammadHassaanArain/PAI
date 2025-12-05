# Research and Decisions

This document records the key decisions made during the planning phase of the AI-Native Textbook project.

## 1. ROS Distribution

- **Decision**: Humble (LTS)
- **Rationale**: As a Long-Term Support (LTS) release, Humble provides stability and a longer support window, which is crucial for a textbook that needs to remain relevant for several years. While Iron has newer features, stability is prioritized for the learning environment.

## 2. Simulation Priority

- **Decision**: Gazebo-first
- **Rationale**: Gazebo is more accessible and has a lower barrier to entry than Isaac Sim. The "simulation-first" principle is best served by a tool that the widest possible audience can use. Isaac Sim will be introduced in later chapters as a more advanced, high-fidelity option.

## 3. Cloud vs On-Prem Default

- **Decision**: Local workstation
- **Rationale**: To minimize costs for students, the default setup will be a local workstation. Cloud GPU options will be presented as an alternative for those who do not have the required hardware.

## 4. Capstone Complexity

- **Decision**: Full VLA + manipulation
- **Rationale**: To align with the project's goal of being "industry-aligned" and preparing students for real-world development, the capstone project must be comprehensive. A full Vision-Language-Action (VLA) pipeline with manipulation provides a much stronger portfolio piece.

## 5. Personalization Depth

- **Decision**: Explanations only
- **Rationale**: As a "bonus" feature, personalization should not add significant complexity to the core content development. Limiting personalization to the depth of explanations provides value without requiring multiple versions of labs and exercises.
