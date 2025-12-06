---
id: 0005
title: Implement AI-Native Homepage
stage: misc
date: 2025-12-06
surface: agent
model: gemini-1.5-flash
feature: 001-ai-robotics-textbook
branch: main
user: user
command: /sp.implement
labels: ["homepage", "docusaurus", "react"]
links:
  spec: null
  plan: null
  tasks: null
  ticket: null
  adr: null
  pr: null
files:
  - src/pages/index.tsx
  - src/pages/styles.module.css
tests:
  - N/A
---

## Prompt

/sp.homepage AI-Native Textbook on Physical AI & Humanoid Robotics

Objective:
Create a fully custom, AI-native, futuristic homepage for a digital textbook titled:
"Physical AI & Humanoid Robotics: From Algorithms to Embodied Intelligence"

Target Audience:
- University students (AI, Robotics, Mechatronics)
- Researchers in Physical AI & Embodied Intelligence
- Robotics engineers and startup founders
- Educators and curriculum designers

Theme & Visual Style:
- Dark futuristic cyber-physical lab aesthetic
- Neon blue, silver, and graphite color palette
- Glassmorphism + minimalistic scientific UI
- Subtle animated grid, robotic silhouettes, and neural network visuals
- Typeface style: modern sans-serif, technical, clean

Homepage Structure:

1. Hero Section (Above the Fold)
- Full-width cinematic hero banner
- Title with animated gradient glow:
  "Physical AI & Humanoid Robotics"
- Subtitle:
  "Designing Intelligence for the Physical World"
- Short tagline:
  "Where Algorithms Gain Bodies and Machines Learn to Think"
- Primary CTA buttons:
  - "Start Learning"
  - "View Curriculum"
- Secondary CTA:
  - "Download Syllabus (PDF)"
- Animated humanoid robot + sensor visualization in background

2. What is Physical AI? (Concept Introduction)
- 2-column layout:
  - Left: concise definition of Physical AI & Embodied Intelligence
  - Right: animated diagram of perception → cognition → action loop
- Bullet highlights:
  - Multi-modal perception
  - Real-world reasoning
  - Closed-loop control
  - Human-robot interaction

3. Book Learning Outcomes
Display as 6 interactive cards:
- Embodied intelligence foundations
- Robot perception & sensor fusion
- Manipulation & locomotion
- Humanoid robot design
- AI control systems
- Ethics and safety in physical AI

4. Curriculum Roadmap (Visual Timeline)
Interactive vertical roadmap:
- Volume 1: AI Fundamentals for Robotics
- Volume 2: Sensors, Vision & Perception
- Volume 3: Motion, Control & Locomotion
- Volume 4: Humanoid Systems Engineering
- Volume 5: Autonomous Decision Making
- Volume 6: Ethics, Safety & Future of Physical AI

5. Hands-On Practice Section
- Highlight practical components:
  - Simulations (Gazebo, MuJoCo, Isaac)
  - ROS2 projects
  - Reinforcement learning labs
  - Humanoid kinematics labs
- Show sample projects with icons:
  - Walking humanoid
  - Robotic grasping
  - Vision-based navigation

6. Target Learners Section
Four persona cards:
- Student
- Researcher
- Engineer
- Innovator / Founder

7. Why This Book Is Different
Comparison grid vs traditional robotics books:
- AI-first approach ✅
- Real-world embodiment ✅
- Simulation-to-real transfer ✅
- Research-backed ✅
- Industry-aligned ✅

8. Author / Project Vision Section
- Mission statement:
  "To build the next generation of embodied AI engineers for a humanoid future."
- Roadmap of future editions
- Open research contribution note

9. Call to Action Footer
- Big bold CTA:
  "Enter the World of Physical Intelligence"
- Buttons:
  - Start Reading
  - Join Research Community
  - GitHub Repository
- Newsletter subscription field

Technical Requirements:
- Fully responsive (desktop, tablet, mobile)
- SEO-optimized
- Dark-mode default
- Fast loading & accessible
- Compatible with Docusaurus custom homepage routing

Output Requirements:
- Generate complete homepage layout content
- Provide section-wise markdown structure for Docusaurus
- Include placeholder image references
- Include animation hints (but no hard dependencies)
- No default template remnants
- Must feel like an AI-native scientific platform, not a blog

Success Criteria:
- Homepage immediately communicates advanced Physical AI focus
- Visually feels like an MIT/DeepMind/Robotics lab site
- Clear learning pathway for students
- Strong credibility for researchers
- High engagement and modern UI feel

## Response snapshot

I have successfully implemented the new AI-native homepage for the Docusaurus website. I replaced the content of `src/pages/index.tsx` with a new React component structure that matches the user's detailed specifications. I also created a new CSS module at `src/pages/styles.module.css` to style the homepage with the requested dark, futuristic aesthetic.

## Outcome

- ✅ Impact: A new, fully custom homepage has been implemented, replacing the default Docusaurus template.
- 🧪 Tests: N/A
- 📁 Files: 
  - `src/pages/index.tsx` (overwritten)
  - `src/pages/styles.module.css` (created)
- 🔁 Next prompts: The homepage is complete. I will now create a PHR for this task.
- 🧠 Reflection: The task was a straightforward implementation of a React-based homepage. Breaking the page into components made the code clean and easy to manage. The use of CSS modules ensures that the styles are scoped locally and won't conflict with other parts of the Docusaurus site.
