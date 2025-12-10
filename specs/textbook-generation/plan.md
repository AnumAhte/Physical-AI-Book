objective:
  Provide a full execution plan for building the AI-Native Physical AI & Humanoid Robotics textbook
  using the existing sp.spec requirements. All tasks must be structured, sequenced, and labeled to
  support Spec-Driven Development using Claude Code + Spec-Kit Plus.

approach:
  - Break the entire project into phases.
  - Each phase must include tasks, subtasks, acceptance criteria, and responsible agent (if applicable).
  - Ensure all phases comply with free-tier architecture and RAG chatbot constraints defined in sp.spec.
  - Keep tasks atomic and traceable for Claude Code automation through `.claude/commands/*`.
  - Reference textbook sections from the official spec but do NOT rewrite the book here.
  - This plan must guide long-term development from repository setup to final deployment.

phases:

  - Phase 1: Repository Initialization
    tasks:
      - Create repository structure (/docs, /backend, /src, /specs, /.specify, /.claude)
      - Add README with project overview
      - Add sp.spec, constitution, and plan files
    acceptance:
      - Repository builds without errors
      - Structure matches spec

  - Phase 2: Docusaurus Frontend Setup
    tasks:
      - Initialize Docusaurus 3 project
      - Configure theme, sidebar, search (FlexSearch)
      - Create homepage + layout components
      - Add metadata system for chapters
    acceptance:
      - Docusaurus runs locally with no build errors
      - Sidebar autogenerates docs

  - Phase 3: Book Chapter Development
    tasks:
      - Create chapter folders per course outline (Weeks 1–13)
      - Write chapters in Markdown according to spec
      - Add images, diagrams, code blocks
      - Add learning objectives + prerequisites
    acceptance:
      - Chapters render correctly
      - Formatting consistent across book

  - Phase 4: RAG Backend Development
    tasks:
      - Initialize FastAPI server
      - Integrate Qdrant Cloud (free tier)
      - Integrate Neon (Postgres) for logs
      - Implement embeddings + context retrieval
      - Implement "select text → ask AI" endpoint
    acceptance:
      - RAG responses ONLY use textbook content
      - API endpoints pass basic tests

  - Phase 5: Chatbot Frontend Integration
    tasks:
      - Build React chatbot component
      - Connect to FastAPI backend
      - Implement select-text analysis
      - Add UI for answers, citations, and logs
    acceptance:
      - Chatbot fully functional
      - No open CORS errors

  - Phase 6: CI/CD Automation
    tasks:
      - GitHub Actions for Docusaurus build + deploy
      - Backend deploy workflow for Railway/Render
      - Add link checker + markdown linter
      - Add quality gates
    acceptance:
      - All workflows pass
      - Frontend deploys on GitHub Pages

  - Phase 7: Optional Enhancements
    tasks:
      - Urdu translation toggle
      - Personalized learning mode
      - Sub-agents and skills for writing chapters
    acceptance:
      - Enhancements do not break free-tier limits

  - Phase 8: Finalization & Launch
    tasks:
      - Validate all chapters
      - Run build + RAG checks
      - Prepare 90-second demo video
      - Write final documentation
    acceptance:
      - Project meets all success criteria from sp.spec
      - End-to-end system fully functional

success_criteria:
  - All phases completed
  - CI/CD green
  - RAG chatbot accurate and restricted to book
  - Free-tier infra functional
  - Textbook fully aligned with official course outline

notes:
  - This plan must remain synchronized with sp.spec.
  - Any structural change must update both sp.spec and sp.plan.
  - Claude Code agents may use this plan to execute tasks step by step.