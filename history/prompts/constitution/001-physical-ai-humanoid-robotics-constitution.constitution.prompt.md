---
id: 001
title: Physical AI & Humanoid Robotics Constitution
stage: constitution
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: unknown
command: /sp.constitution
labels: ["constitution","project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - history/prompts/constitution/001-physical-ai-humanoid-robotics-constitution.constitution.prompt.md
tests:

---

## Prompt

/sp.constitution

Project Name: Physical AI & Humanoid Robotics — Essentials

Purpose:
Create a professional AI-native textbook for Physical AI & Humanoid Robotics. The textbook must serve as a **fast, simple, high-quality learning resource**, built with **Docusaurus 3** UI and **fully integrated RAG chatbot** using Claude Code.

Scope:
- 13-week structured course
  1. Introduction to Physical AI
  2. ROS 2 Fundamentals
  3. Digital Twin Simulation (Gazebo + Unity)
  4. NVIDIA Isaac Sim
  5. Vision-Language-Action (VLA) Systems
  6. Capstone: Autonomous Humanoid Project
- Content includes markdown chapters, references, glossary, images.
- Free-tier friendly; minimal GPU or heavy resources.

Core Principles:
- Simplicity and clarity for fast comprehension.
- Accuracy in robotics, AI, and VLA concepts.
- Minimalist UI and lightweight builds.
- Spec-Driven Development (Spec-Kit Plus + Claude Code) for reproducibility.
- RAG chatbot answers **ONLY** from textbook content.
- Modular architecture: `docs/`, `src/`, `specs/`, `.claude/commands/`.
- CI/CD ready (GitHub Actions + GitHub Pages).

Key Features:
- Docusaurus textbook with dashboard, nested sidebar, and search.
- RAG chatbot integration via Claude Code commands.
- Select-text → Ask AI functionality.
- Optional features: Urdu translation, personalized chapters.
- Metadata validation for chapters (JSON Schema).
- Automated build, test, and deploy pipelines.

Constraints:
- No heavy GPU usage required; free-tier friendly embeddings.
- Lightweight vector DB for RAG (Qdrant free-tier or alternative).
- Deployment to GitHub Pages; backend server optional (Claude commands or FastAPI if needed).

Success Criteria:
- Build completes without errors.
- Textbook content accurate and complete.
- RAG chatbot returns only text-based answers from book.
- Frontend UI clean, responsive, and interactive.
- CI/CD pipelines pass with quality gates.
- Full Spec-Driven workflow enabled (planning, tasks, history, constitution).

Deliverables:
- `/docs` folder with all chapter markdowns.
- `/src` folder with React/Docusaurus components.
- `/specs` + `/.specify` + `/.claude/commands` configured.
- GitHub Actions workflows for build, test, deploy.
- RAG chatbot integrated and functional.
- Quickstart guide for local development.

Notes:
- Keep project modular for easy updates and scaling.
- Follow Spec-Kit Plus methodology strictly.
- Ensure lightweight embeddings and free-tier compliance.
- RAG chatbot should never answer outside book text.

## Response snapshot

Created a Prompt History Record (PHR) for the project constitution.

## Outcome

- ✅ Impact: PHR created for project constitution.
- 🧪 Tests: N/A
- 📁 Files: 1
- 🔁 Next prompts: N/A
- 🧠 Reflection: Successfully recorded the project constitution as a PHR.

## Evaluation notes (flywheel)

- Failure modes observed: N/A
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
