---
id: 0001
title: update-constitution-physical-ai-textbook
stage: constitution
date: 2025-12-07
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: master
user: user
command: /sp.constitution
labels: ["constitution", "physical-ai", "textbook"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests: []
---

## Prompt

/sp.constitution

Project: AI-Native Textbook on Physical AI & Humanoid Robotics

Core principles:
- Technical accuracy through primary research, official documentation, and industry-standard sources
- Engineering-first clarity for robotics, AI, and computer science students
- Reproducibility of all simulations, experiments, and code pipelines
- Systems thinking across AI, robotics, perception, control, and simulation
- Safety-first robotics development and ethical AI deployment
- Sim-to-Real validity for all Physical AI workflows
- Vendor-neutral explanations where possible (ROS 2, Gazebo, NVIDIA Isaac, OpenAI, etc.)
- Docusaurus-first content structuring: all chapters, modules, and assets authored for static site generation

Key standards:
- All factual claims must be verifiable via:
  - Peer-reviewed research papers
  - Official ROS, NVIDIA Isaac, and hardware documentation
  - Reputable academic institutions or industry whitepapers
- Citation format: APA 7th Edition
- Source distribution:
  - Minimum 60% peer-reviewed academic sources
  - Maximum 40% industry documentation and whitepapers
- Code accuracy:
  - All code must be tested in ROS 2 Humble or Iron
  - Python code must follow PEP8
- Simulation validity:
  - Gazebo and Isaac Sim examples must be reproducible
  - URDF/SDF models must be physically consistent
- AI model usage must follow responsible AI guidelines
- Safety constraints applied to:
  - Robotics actuation
  - Reinforcement learning
  - Real-world sensor deployment

Constraints:
- Total book length: 10,000–18,000 words
- Minimum number of sources: 30
- Minimum number of diagrams/figures: 40
- Each module must include:
  - Learning objectives
  - Theoretical foundation
  - Practical lab exercises
  - Assessment tasks
- Format:
  - Docusaurus-based static site
  - CLI initializes project structure automatically
  - Deployed on GitHub Pages
  - All diagrams in SVG or PNG
- Code repositories must be linked where applicable
- No proprietary or NDA-restricted material allowed
- Cloud resources must be referenced with cost and latency limitations

Ethical & Safety Requirements:
- No autonomous weaponization examples
- No unsafe reinforcement learning policies for physical robots
- Clear separation between simulation and real-world deployment risks
- Mandatory safety disclaimers for:
  - Actuators
  - Power systems
  - Human-robot interaction
- AI bias, data privacy, and dataset provenance must be discussed in relevant chapters

Success criteria:
- Students can:
  - Explain Physical AI and embodied intelligence
  - Build ROS 2 nodes and control simulated robots
  - Deploy perception pipelines using NVIDIA Isaac
  - Implement Vision-Language-Action pipelines
  - Perform Sim-to-Real transfer on Jetson hardware
- All labs are reproducible on:
  - RTX-based workstations OR approved cloud instances
  - Jetson Orin Nano / NX edge devices
- Book successfully deployed on GitHub Pages
- All references validated
- Zero plagiarism detected
- Passes technical review by a ROS/Robotics expert
- Passes academic integrity review
- Meets capstone readiness for Autonomous Humanoid Project

Audience Definition:
- Advanced undergraduate and graduate students in:
  - Robotics
  - Artificial Intelligence
  - Computer Engineering
- Robotics educators and lab instructors
- AI engineers transitioning into robotics

Compliance:
- Open-source compliance (Apache 2.0, MIT, BSD only)
- Full attribution for:
  - Datasets
  - Simulation assets
  - Open-source libraries
- No proprietary SDK redistribution

Long-Term Maintainability:
- Content must support:
  - ROS 2 future LTS upgrades
  - NVIDIA Isaac platform version updates
  - Evolving LLM and VLA model stacks
- Versioning required for:
  - Hardware requirements
  - Software dependencies
  - Cloud infrastructure references

## Response snapshot

The project constitution has been successfully updated for the "AI-Native Textbook on Physical AI & Humanoid Robotics" project. The new constitution includes 8 core principles:

1. Technical Accuracy and Verification
2. Engineering-First Clarity
3. Reproducibility of Simulations and Code
4. Systems Thinking Across Disciplines
5. Safety-First Robotics Development
6. Sim-to-Real Validity
7. Vendor-Neutral Explanations
8. Docusaurus-First Content Structuring

Additional sections include Content Standards and Constraints, Audience and Success Criteria, and Governance. The constitution establishes foundational principles for the project with version 1.0.0, ratified on 2025-12-07.

## Outcome

- ✅ Impact: New project constitution established with 8 core principles for Physical AI textbook
- 🧪 Tests: No tests required for constitution update
- 📁 Files: Updated .specify/memory/constitution.md with new principles and governance
- 🔁 Next prompts: Project can now proceed with spec, plan, and task creation following new constitution
- 🧠 Reflection: Constitution provides clear framework for technical accuracy, safety, and reproducibility in Physical AI education

## Evaluation notes (flywheel)

- Failure modes observed: None - successful constitution creation
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin creating feature specifications following the new constitution
