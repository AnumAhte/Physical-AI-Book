# Physical AI & Humanoid Robotics — Essentials Constitution

## Core Principles

### I. Simplicity and Clarity
This constitution mandates simplicity and clarity in all aspects of the project to ensure fast comprehension for learners. Designs, code, and documentation must prioritize directness and ease of understanding.

### II. Accuracy and Veracity
All content related to robotics, AI, and Vision-Language-Action (VLA) concepts must be technically accurate and validated against authoritative sources. Misinformation is strictly prohibited.

### III. Minimalist Design
The user interface and overall build architecture must remain minimalist and lightweight. Avoid unnecessary complexity, heavy dependencies, or resource-intensive components to ensure free-tier friendliness.

### IV. Spec-Driven Development (SDD)
The project will strictly adhere to Spec-Driven Development (Spec-Kit Plus + Claude Code) methodology. All features, plans, and tasks must originate from and be traceable to clear specifications to ensure reproducibility and consistency.

### V. RAG Chatbot Integrity
The integrated RAG chatbot functionality must ONLY provide answers sourced directly and exclusively from the textbook content. It is strictly prohibited from generating responses outside the scope of the provided learning material.

### VI. Modular Architecture
The project's codebase will maintain a modular architecture, clearly separating concerns into `docs/`, `src/`, `specs/`, and `.claude/commands/` directories. This promotes maintainability, scalability, and independent development.

### VII. CI/CD Readiness
All development must ensure continuous integration and continuous deployment (CI/CD) readiness. Automated workflows using GitHub Actions and GitHub Pages are central to the project's deployment strategy.

## Key Features & Constraints

### Key Features
- Docusaurus textbook with dashboard, nested sidebar, and search.
- RAG chatbot integration via Claude Code commands.
- Select-text → Ask AI functionality.
- Optional features: Urdu translation, personalized chapters.
- Metadata validation for chapters (JSON Schema).
- Automated build, test, and deploy pipelines.

### Constraints
- No heavy GPU usage required; free-tier friendly embeddings.
- Lightweight vector DB for RAG (Qdrant free-tier or alternative).
- Deployment to GitHub Pages; backend server optional (Claude commands or FastAPI if needed).

## Success Criteria & Deliverables

### Success Criteria
- Build completes without errors.
- Textbook content accurate and complete.
- RAG chatbot returns only text-based answers from book.
- Frontend UI clean, responsive, and interactive.
- CI/CD pipelines pass with quality gates.
- Full Spec-Driven workflow enabled (planning, tasks, history, constitution).

### Deliverables
- `/docs` folder with all chapter markdowns.
- `/src` folder with React/Docusaurus components.
- `/specs` + `/.specify` + `/.claude/commands` configured.
- GitHub Actions workflows for build, test, deploy.
- RAG chatbot integrated and functional.
- Quickstart guide for local development.

## Governance
This Constitution outlines the foundational principles and operational guidelines for the "Physical AI & Humanoid Robotics — Essentials" project. It supersedes all other practices and documentation. Amendments to this constitution require a clear rationale, documented impact analysis, and approval from core maintainers. All pull requests and code reviews must verify compliance with these principles.

**Version**: 1.0.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06
