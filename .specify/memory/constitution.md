<!--
Sync Impact Report:
Version change: N/A → 1.0.0
Modified principles: N/A (new constitution)
Added sections: All principles and sections (new constitution)
Removed sections: N/A
Templates requiring updates: N/A (first version)
Follow-up TODOs: None
-->

# AI-Native Textbook on Physical AI & Humanoid Robotics Constitution

## Core Principles

### Technical Accuracy and Verification
Technical accuracy through primary research, official documentation, and industry-standard sources. All factual claims must be verifiable via peer-reviewed research papers, official ROS, NVIDIA Isaac, and hardware documentation, or reputable academic institutions and industry whitepapers. Citation format follows APA 7th Edition with a minimum of 60% peer-reviewed academic sources and maximum 40% industry documentation and whitepapers.

### Engineering-First Clarity
Engineering-first clarity for robotics, AI, and computer science students. Content must prioritize practical understanding and implementation over theoretical concepts alone, ensuring students can build, test, and deploy the concepts covered in the textbook.

### Reproducibility of Simulations and Code
Reproducibility of all simulations, experiments, and code pipelines. All code must be tested in ROS 2 Humble or Iron, Python code must follow PEP8 standards, and Gazebo and Isaac Sim examples must be reproducible with URDF/SDF models that are physically consistent.

### Systems Thinking Across Disciplines
Systems thinking across AI, robotics, perception, control, and simulation. Content must demonstrate how different components interact as a complete system rather than isolated modules, emphasizing the integration of multiple technologies in physical AI workflows.

### Safety-First Robotics Development
Safety-first robotics development and ethical AI deployment. All examples and implementations must follow responsible AI guidelines, with safety constraints applied to robotics actuation, reinforcement learning, and real-world sensor deployment. No autonomous weaponization examples are permitted.

### Sim-to-Real Validity
Sim-to-Real validity for all Physical AI workflows. Content must maintain clear separation between simulation and real-world deployment risks, with mandatory safety disclaimers for actuators, power systems, and human-robot interaction.

### Vendor-Neutral Explanations
Vendor-neutral explanations where possible (ROS 2, Gazebo, NVIDIA Isaac, OpenAI, etc.). While specific tools are necessary for examples, explanations should focus on underlying principles rather than vendor-specific implementations to ensure long-term relevance.

### Docusaurus-First Content Structuring
Docusaurus-first content structuring: all chapters, modules, and assets authored for static site generation. Content must be structured for static site generation using Docusaurus, deployed on GitHub Pages, with all diagrams in SVG or PNG format.

## Content Standards and Constraints

All content must adhere to the following standards and constraints:
- Total book length: 10,000–18,000 words
- Minimum number of sources: 30
- Minimum number of diagrams/figures: 40
- Each module must include: Learning objectives, Theoretical foundation, Practical lab exercises, Assessment tasks
- Code repositories must be linked where applicable
- No proprietary or NDA-restricted material allowed
- Cloud resources must be referenced with cost and latency limitations
- AI bias, data privacy, and dataset provenance must be discussed in relevant chapters

## Audience and Success Criteria

Content is designed for advanced undergraduate and graduate students in Robotics, Artificial Intelligence, and Computer Engineering; Robotics educators and lab instructors; and AI engineers transitioning into robotics. Success is measured by students' ability to:
- Explain Physical AI and embodied intelligence
- Build ROS 2 nodes and control simulated robots
- Deploy perception pipelines using NVIDIA Isaac
- Implement Vision-Language-Action pipelines
- Perform Sim-to-Real transfer on Jetson hardware
- Complete labs reproducible on RTX-based workstations or Jetson Orin Nano/NX edge devices

## Governance

This constitution establishes the foundational principles for the AI-Native Textbook on Physical AI & Humanoid Robotics project. All content, code, and documentation must comply with these principles. The constitution supersedes all other practices and guidelines within the project. Amendments require documentation of the changes, approval from the project maintainers, and a migration plan for existing content. All contributions must verify compliance with these principles during review processes.

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07
