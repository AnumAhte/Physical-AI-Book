Feature: textbook-generation

Objective:
Define a complete, unambiguous specification for building the AI-native textbook with RAG chatbot, ensuring modularity, free-tier compliance, and Spec-Driven Development workflow.

Book Structure:
1. Introduction to Physical AI (Weeks 1-2)
   - Markdown chapters: intro.md, setup.md
   - Hardware setup instructions
   - Images / diagrams
2. ROS 2 Fundamentals (Weeks 3-5)
   - Markdown chapters for topics, URDF, ROS2 nodes, topics
   - Code examples
   - Embedded simulations
3. Digital Twin Simulation (Weeks 6-7)
   - Gazebo / Unity simulations
   - Setup instructions
   - Simulation examples
4. NVIDIA Isaac Sim (Weeks 8-10)
   - GPU-accelerated simulation instructions
   - Synthetic data & imitation learning
5. Vision-Language-Action Systems (Weeks 11-13)
   - Multimodal AI explanations
   - Transformer policies
   - Example pipelines
6. Capstone: Autonomous Humanoid Project (Week 13)
   - End-to-end project
   - Voice → Action pipeline
   - Challenge exercises

Technical Requirements:
- Docusaurus 3 frontend with:
  - Dashboard homepage
  - Nested collapsible sidebar
  - Search (Algolia + Flexsearch)
  - Metadata for chapters (prerequisites, learning objectives)
- RAG Chatbot integration:
  - Claude Code commands (`.claude/commands/`)
  - Vector DB (Qdrant free-tier or equivalent)
  - Select-text → Ask AI
  - Answers only from book text
- Spec-Driven Development:
  - `.specify/` memory + constitution
  - `/specs` feature specs + tasks + plan
  - History logs for prompts and AI actions
- CI/CD pipelines:
  - GitHub Actions for build, test, deploy
  - Quality gates (build, link validation, Lighthouse scores)
- Free-tier architecture:
  - Lightweight embeddings
  - Minimal GPU / server usage
- Optional:
  - Urdu translation
  - Personalized chapters

Constraints:
- RAG chatbot answers strictly from textbook content.
- Avoid heavy GPUs or paid cloud services.
- Maintain modular structure for updates and scaling.
- Ensure build speed and Docusaurus performance.

Deliverables:
- `/docs` folder with all markdown chapters and references
- `/src` folder with Docusaurus React components
- `/specs` + `/.specify` + `/.claude/commands` configured
- CI/CD workflows for build and GitHub Pages deployment
- Fully functional RAG chatbot integrated
- Quickstart guide for local development

Success Criteria:
- All chapters correctly formatted and complete
- RAG chatbot answers correctly based on selected text
- Build passes without errors
- Frontend UI clean, interactive, and responsive
- CI/CD pipelines pass with quality gates
- Spec-driven workflow maintained with updated history logs

End of Specification