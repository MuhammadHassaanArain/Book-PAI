# Claude Skill: Physical AI & Humanoid Robotics — Frontend Builder

## Skill Name
physical-ai-frontend

## Purpose
This skill designs, builds, and maintains the **entire frontend UI/UX** for the
"Physical AI & Humanoid Robotics Textbook" using:

- Docusaurus
- React
- Tailwind / CSS Modules
- MDX
- Mermaid
- ShadCN UI (optional)
- Dark futuristic robotics theme

It outputs:
- Homepage UI
- Module landing pages
- Chapter layouts
- Interactive labs UI
- Assessment UI
- Navigation + sidebar configuration
- Performance & SEO optimization

---

## Project Context (Persistent)
Project: Hackathon I — Create a Textbook for Teaching Physical AI & Humanoid Robotics  
Frontend Stack:
- Docusaurus v3+
- React 18
- MDX
- TailwindCSS
- Mermaid
- Prism Code Highlighting

Target Users:
- Engineering Students
- Robotics Researchers
- Simulation Engineers
- AI Developers

Design Style:
- Futuristic
- Dark Mode Default
- Neon Accent Colors
- Clean Engineering Layout
- NVIDIA / Robotics Inspired

---

## Theme Tokens (Locked)
```yaml
primaryColor: "#00D9FF"
secondaryColor: "#B620E0"
accentColor: "#FF6B35"
backgroundColor: "#0A0E27"
surfaceColor: "#151B3B"
textColor: "#E8EAED"
mutedTextColor: "#9CA3AF"
Fonts:

Headings: Space Grotesk

Body: Inter

Code: JetBrains Mono

Directory Rules
All frontend outputs must match:

/src
/components
/themes
/pages
/layouts
/css
/utils

/static
/docusaurus.config.ts
/sidebars.ts

UI Content Rules
Mobile-first responsive design

WCAG AA accessibility

60fps animations

No blocking JS

Lighthouse score ≥ 90

Dark mode by default

Glassmorphism for cards

Subtle motion only (Framer Motion)

Supported Frontend Commands
1. Generate Homepage
Trigger:
"/skill ui homepage"

Output:

Hero section

Course overview

Modules grid

Tech stack banner

Call to action

Footer

React + CSS code

2. Generate Module UI
Trigger:
"/skill ui module <module-number>"

Output:

Module landing page

Chapter cards

Lab + project sections

Progress indicator

3. Generate Chapter Layout
Trigger:
"/skill ui chapter"

Output:

Reading layout

Code + diagram panels

Simulation tabs

Hardware tabs

4. Generate Lab UI
Trigger:
"/skill ui lab"

Output:

Stepper-based lab UI

Terminal blocks

Checkpoint system

Result validation UI

5. Generate Assessment UI
Trigger:
"/skill ui assessment"

Output:

MCQ UI

Coding test UI

Practical task UI

Auto-grading hooks

6. Generate Navigation & Sidebar
Trigger:
"/skill ui navigation"

Output:

Docusaurus sidebar config

Multi-level navigation

Search & filters

Versioning UI

7. Generate Design System
Trigger:
"/skill ui design-system"

Output:

Button system

Card system

Modal system

Alerts

Badges

Typography scale

8. Performance & SEO Optimization
Trigger:
"/skill ui seo"

Output:

Meta config

OG images guide

Page speed improvements

Lazy loading config

9. UI Review & Refactor
Trigger:
"/skill ui review <filepath>"

Output:

Accessibility issues

Performance issues

Responsive bugs

UI consistency score

Output Requirements
All outputs must:

Be copy-paste deployable

Be compatible with Docusaurus

Use modern React patterns

Avoid deprecated APIs

Avoid inline CSS unless necessary

Animation Rules
Use Framer Motion only when meaningful

No infinite animations

No seizure-inducing effects

Subtle parallax allowed

Example Usage
User:
"/skill ui homepage"

Claude:

Generates full React homepage

Tailwind config

CSS tokens

Responsive layout

Module cards

Deployment instructions

Safety Rules
No remote script injection

No insecure external CDNs

No hardcoded secrets

No inline eval

CSP compliant

Quality Standard
UI must look enterprise-grade:

NVIDIA developer quality

Clean research-lab aesthetic

Production ready

Fully responsive

Default Output Mode
React component

Tailwind classes

Supporting config files

Setup instructions

yaml
Copy code

---