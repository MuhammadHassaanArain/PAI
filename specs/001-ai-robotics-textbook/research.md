# Research Findings: AI-Native Textbook on Physical AI & Humanoid Robotics

**Feature Branch**: `001-ai-robotics-textbook`  
**Created**: December 4, 2025  
**Spec**: [specs/001-ai-robotics-textbook/spec.md](specs/001-ai-robotics-textbook/spec.md)
**Plan**: [specs/001-ai-robotics-textbook/plan.md](specs/001-ai-robotics-textbook/plan.md)

## Decisions

### 1. ROS Distribution

*   **Decision**: Option A: Humble (LTS)
*   **Rationale**: Given the target audience (university students, beginners) and the need for long-term stability and support, choosing a Long Term Support (LTS) release like Humble provides a more consistent and reliable learning experience. It minimizes the need for frequent updates and allows learners to focus on core concepts rather than managing breaking changes.
*   **Alternatives Considered**:
    *   **Option B: Iron**: While offering the latest features, Iron is a non-LTS release with a shorter support window, which could lead to a less stable learning platform and require more maintenance for both authors and students.

### 2. Simulation Priority

*   **Decision**: Option B: Isaac Sim-first
*   **Rationale**: The project emphasizes Physical AI and industry-grade humanoid robotics. NVIDIA Isaac Sim provides higher realism, advanced physics, and better integration with NVIDIA's AI ecosystem (e.g., Isaac ROS, Omniverse), which aligns more closely with the project's goals of preparing students for industry. While potentially less accessible for absolute beginners due to its feature set, it offers a more powerful and relevant learning environment for advanced robotics.
*   **Alternatives Considered**:
    *   **Option A: Gazebo-first**: Gazebo is more widely adopted in academic settings and generally more accessible. However, its realism and integration with advanced AI tools might not be as robust as Isaac Sim, potentially limiting the "industry-grade" aspect of the capstone project. We will still include Gazebo content as per FR-001.

### 3. Cloud vs On-Prem Default

*   **Decision**: Option A: Local workstation (with clear Cloud GPU alternatives)
*   **Rationale**: To ensure maximum accessibility for the target audience (university students, self-learners) and minimize initial cost barriers, the default setup should be optimized for a local workstation. This acknowledges that not all students will have immediate access to cloud computing resources. However, the plan explicitly includes "Cloud vs On-Prem Lab Setup" and "Hardware / Cloud Alternative" in each chapter to cater to those who prefer or require cloud resources (e.g., for GPU-intensive tasks).
*   **Alternatives Considered**:
    *   **Option B: Cloud GPU-first**: While offering powerful resources, making Cloud GPU the default would introduce an immediate cost barrier for many students, potentially limiting the reach and accessibility of the textbook.

### 4. Capstone Complexity

*   **Decision**: Option B: Full VLA + manipulation
*   **Rationale**: The project's "Final Outcome" explicitly mentions building a "simulated conversational humanoid robot using Physical AI principles." Achieving this requires more than just navigation and vision; it necessitates Vision-Language-Action (VLA) capabilities and manipulation to demonstrate true embodied intelligence. While more complex, this decision aligns with the "industry-grade" and "capstone-level" goals, providing a more impactful learning experience and a stronger demo. The "simulation-first" approach will mitigate reproducibility risks.
*   **Alternatives Considered**:
    *   **Option A: Navigation + Vision only**: This would be simpler to implement and potentially more reproducible, but would fall short of the project's ambitious goal of a "conversational humanoid robot" and would not fully leverage the VLA module.

### 5. Agent Autonomy Level

*   **Decision**: Option B: Hybrid reasoning + tools
*   **Rationale**: The project aims for an "AI-native" learning experience with an "agent-powered" RAG chatbot. A hybrid approach allows the Gemini LLM (via agent abstraction) to engage in more sophisticated pedagogical reasoning and response generation beyond strict tool outputs. This enhances the "explanation" and "assessment" agents, leading to a more intelligent and helpful tutor. Safety and refusal agents will provide the necessary guardrails.
*   **Alternatives Considered**:
    *   **Option A: Strict tool-only responses**: This would offer maximum control and predictability but would limit the generative capabilities of the Gemini LLM, potentially making the chatbot less "AI-native" and less engaging as a learning companion.

### 6. Personalization Depth

*   **Decision**: Option B: Explanations + Labs
*   **Rationale**: Personalization is a "Bonus Scope" but deeply impacts the learner's experience. Adapting both explanations and lab complexity provides a richer, more tailored learning path. This allows the agent to adjust not just *how* it explains concepts but also the *difficulty* and *focus* of practical exercises based on the student's background, hardware, and learning speed preferences. This maximizes the value of the personalization feature.
*   **Alternatives Considered**:
    *   **Option A: Explanations only**: This would be simpler to implement but would miss an opportunity to personalize the hands-on learning aspect, which is a core principle of the textbook.
    *   **Not implementing personalization**: While an option, the project explicitly lists it as a "Bonus Scope" feature, indicating a desire to explore this capability.

## Conclusion

These decisions prioritize the project's core goals of delivering an industry-aligned, AI-native, hands-on learning experience, while balancing accessibility and realism. The next steps will involve designing the data model and API contracts based on these established technical directions.