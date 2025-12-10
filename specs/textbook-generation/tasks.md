# Tasks: AI-Native Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/textbook-generation/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: Tests are OPTIONAL - not explicitly requested in this specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/` for FastAPI, Docusaurus root for frontend
- `/docs/` for textbook chapters
- `/src/` for Docusaurus React components
- `/.specify/` and `/.claude/` for Spec-Driven Development

---

## Phase 1: Setup (Repository Initialization)

**Purpose**: Create the foundational repository structure

- [x] T001 Create repository structure (/docs, /backend, /src, /specs, /.specify, /.claude) in the root directory
- [x] T002 Add README with project overview to README.md
- [x] T003 [P] Add sp.spec to specs/textbook-generation/spec.md
- [x] T004 [P] Add constitution to .specify/memory/constitution.md
- [x] T005 [P] Add plan file to specs/textbook-generation/plan.md

**Checkpoint**: Repository structure complete and matches spec

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**CRITICAL**: No user story work can begin until this phase is complete

- [x] T006 Initialize Docusaurus 3 project in repository root with `npx create-docusaurus@latest . classic --typescript`
- [x] T007 [P] Install and configure FlexSearch plugin in docusaurus.config.ts
- [x] T008 [P] Create .env.example with required environment variables (QDRANT_URL, NEON_DATABASE_URL, ANTHROPIC_API_KEY)
- [x] T009 Configure base theme settings (dark/light mode, colors) in docusaurus.config.ts
- [x] T010 [P] Setup backend directory structure: backend/src/, backend/requirements.txt, backend/pyproject.toml
- [x] T011 [P] Initialize FastAPI project with base dependencies in backend/requirements.txt

**Checkpoint**: Foundation ready - Docusaurus runs locally, backend structure exists

---

## Phase 3: User Story 1 - Docusaurus Frontend with Book Content (Priority: P1) MVP

**Goal**: Deploy a fully functional Docusaurus textbook with all chapters, navigation, and search

**Independent Test**: Run `npm run build && npm run serve` - site loads with all chapters accessible via sidebar

### Implementation for User Story 1

#### Docusaurus Configuration
- [x] T012 [US1] Configure nested collapsible sidebar with chapter categories in sidebars.ts
- [x] T013 [US1] Create dashboard homepage component in src/pages/index.tsx
- [ ] T014 [P] [US1] Create base layout wrapper component in src/components/Layout/Layout.tsx
- [x] T015 [P] [US1] Add chapter metadata system (frontmatter: prerequisites, learning objectives) in docusaurus.config.ts

#### Chapter Structure Creation
- [x] T016 [P] [US1] Create chapter folder structure: docs/week-01-02-intro/, docs/week-03-05-ros2/, docs/week-06-07-digital-twin/, docs/week-08-10-isaac-sim/, docs/week-11-13-vla/, docs/week-13-capstone/
- [x] T017 [P] [US1] Create assets directory for images and diagrams in docs/assets/

#### Week 1-2: Introduction to Physical AI
- [x] T018 [P] [US1] Write intro.md for Introduction to Physical AI in docs/week-01-02-intro/intro.md
- [x] T019 [P] [US1] Write setup.md with hardware setup instructions in docs/week-01-02-intro/setup.md
- [ ] T020 [US1] Add images and diagrams for Week 1-2 chapters in docs/assets/intro/

#### Week 3-5: ROS 2 Fundamentals
- [x] T021 [P] [US1] Write ros2-overview.md for ROS 2 introduction in docs/week-03-05-ros2/ros2-overview.md
- [x] T022 [P] [US1] Write nodes-topics.md for ROS 2 nodes and topics in docs/week-03-05-ros2/nodes-topics.md
- [x] T023 [P] [US1] Write urdf.md for URDF robot descriptions in docs/week-03-05-ros2/urdf.md
- [ ] T024 [US1] Add code examples for ROS 2 chapters in docs/week-03-05-ros2/examples/
- [ ] T025 [US1] Embed simulation references in ROS 2 chapters

#### Week 6-7: Digital Twin Simulation
- [x] T026 [P] [US1] Write gazebo-intro.md for Gazebo simulation in docs/week-06-07-digital-twin/gazebo-intro.md
- [x] T027 [P] [US1] Write unity-simulation.md for Unity simulations in docs/week-06-07-digital-twin/unity-simulation.md
- [ ] T028 [US1] Add simulation setup instructions and examples in docs/week-06-07-digital-twin/

#### Week 8-10: NVIDIA Isaac Sim
- [x] T029 [P] [US1] Write isaac-intro.md for Isaac Sim introduction in docs/week-08-10-isaac-sim/isaac-intro.md
- [ ] T030 [P] [US1] Write gpu-simulation.md for GPU-accelerated simulation in docs/week-08-10-isaac-sim/gpu-simulation.md
- [ ] T031 [P] [US1] Write synthetic-data.md for synthetic data generation in docs/week-08-10-isaac-sim/synthetic-data.md
- [ ] T032 [US1] Write imitation-learning.md for imitation learning in docs/week-08-10-isaac-sim/imitation-learning.md

#### Week 11-13: Vision-Language-Action Systems
- [x] T033 [P] [US1] Write vla-overview.md for VLA systems introduction in docs/week-11-13-vla/vla-overview.md
- [ ] T034 [P] [US1] Write multimodal-ai.md for multimodal AI explanations in docs/week-11-13-vla/multimodal-ai.md
- [ ] T035 [P] [US1] Write transformer-policies.md for transformer policies in docs/week-11-13-vla/transformer-policies.md
- [ ] T036 [US1] Add example pipelines for VLA systems in docs/week-11-13-vla/examples/

#### Week 13: Capstone Project
- [x] T037 [P] [US1] Write capstone-overview.md for capstone introduction in docs/week-13-capstone/capstone-overview.md
- [ ] T038 [P] [US1] Write voice-action-pipeline.md for Voice-to-Action pipeline in docs/week-13-capstone/voice-action-pipeline.md
- [ ] T039 [US1] Write challenge-exercises.md with challenge exercises in docs/week-13-capstone/challenge-exercises.md

#### Final Chapter Tasks
- [ ] T040 [US1] Add learning objectives and prerequisites frontmatter to all chapter files
- [ ] T041 [US1] Verify all chapter links and cross-references work correctly

**Checkpoint**: User Story 1 complete - Full textbook renders with all chapters, navigation, and search

---

## Phase 4: User Story 2 - RAG Backend Development (Priority: P2)

**Goal**: Deploy a FastAPI backend with RAG capabilities that answers ONLY from textbook content

**Independent Test**: Send POST to `/api/ask` with selected text - receive contextually accurate response citing textbook content

### Implementation for User Story 2

#### FastAPI Core Setup
- [ ] T042 [US2] Create FastAPI application entry point in backend/src/main.py
- [ ] T043 [P] [US2] Create configuration module with environment variables in backend/src/config.py
- [ ] T044 [P] [US2] Create CORS middleware configuration in backend/src/middleware.py

#### Vector Database Integration
- [ ] T045 [US2] Implement Qdrant Cloud client connection in backend/src/vector_db.py
- [ ] T046 [US2] Create collection schema for textbook embeddings in backend/src/vector_db.py
- [ ] T047 [US2] Implement vector search function in backend/src/vector_db.py

#### Embeddings Pipeline
- [ ] T048 [US2] Implement lightweight embeddings generator (sentence-transformers) in backend/src/embeddings.py
- [ ] T049 [US2] Create textbook content ingestion script in backend/src/ingest.py
- [ ] T050 [US2] Implement chunk splitting for textbook markdown in backend/src/chunker.py

#### RAG Pipeline
- [ ] T051 [US2] Implement context retrieval logic in backend/src/rag_pipeline.py
- [ ] T052 [US2] Implement Claude API integration for response generation in backend/src/llm.py
- [ ] T053 [US2] Create response formatter with citations in backend/src/formatter.py

#### API Endpoints
- [ ] T054 [US2] Implement "select text -> ask AI" POST endpoint in backend/src/api.py
- [ ] T055 [US2] Implement health check endpoint in backend/src/api.py
- [ ] T056 [P] [US2] Integrate Neon (Postgres) for query logging in backend/src/logger.py

#### Validation
- [ ] T057 [US2] Verify RAG responses only use textbook content (no hallucination)
- [ ] T058 [US2] Test API endpoints with sample queries

**Checkpoint**: User Story 2 complete - RAG backend functional, returns textbook-grounded answers

---

## Phase 5: User Story 3 - Chatbot Frontend Integration (Priority: P3)

**Goal**: Integrate an interactive chatbot into the Docusaurus frontend with select-text functionality

**Independent Test**: Select text in any chapter, click "Ask AI", receive accurate response with citations displayed in chatbot UI

### Implementation for User Story 3

#### Chatbot Component
- [ ] T059 [US3] Create React chatbot container component in src/components/Chatbot/Chatbot.tsx
- [ ] T060 [P] [US3] Create chatbot message component in src/components/Chatbot/Message.tsx
- [ ] T061 [P] [US3] Create chatbot input component in src/components/Chatbot/Input.tsx
- [ ] T062 [US3] Add chatbot styling in src/components/Chatbot/Chatbot.module.css

#### API Integration
- [ ] T063 [US3] Create API client utility for backend communication in src/utils/api.ts
- [ ] T064 [US3] Implement error handling and loading states in src/utils/api.ts

#### Text Selection Feature
- [ ] T065 [US3] Implement text selection detection hook in src/hooks/useTextSelection.ts
- [ ] T066 [US3] Create "Ask AI" floating button component in src/components/AskAIButton/AskAIButton.tsx
- [ ] T067 [US3] Integrate text selection with chatbot in src/components/Chatbot/Chatbot.tsx

#### UI Enhancements
- [ ] T068 [US3] Add citations display in chatbot responses in src/components/Chatbot/Citation.tsx
- [ ] T069 [US3] Add conversation history display in chatbot UI
- [ ] T070 [US3] Integrate chatbot into Docusaurus layout in src/theme/Root.tsx

#### Validation
- [ ] T071 [US3] Verify no CORS errors in browser console
- [ ] T072 [US3] Test end-to-end flow: select text -> ask -> receive answer

**Checkpoint**: User Story 3 complete - Chatbot integrated, select-text-to-ask works seamlessly

---

## Phase 6: User Story 4 - CI/CD Automation (Priority: P4)

**Goal**: Automated build, test, and deployment pipelines for both frontend and backend

**Independent Test**: Push to main branch triggers successful build and deployment to GitHub Pages

### Implementation for User Story 4

#### Docusaurus CI/CD
- [ ] T073 [P] [US4] Create GitHub Actions workflow for Docusaurus build in .github/workflows/docusaurus-build.yml
- [ ] T074 [P] [US4] Create GitHub Actions workflow for GitHub Pages deployment in .github/workflows/docusaurus-deploy.yml
- [ ] T075 [US4] Add link checker step to build workflow in .github/workflows/docusaurus-build.yml
- [ ] T076 [US4] Add markdown linter step to build workflow in .github/workflows/docusaurus-build.yml

#### Backend CI/CD
- [ ] T077 [P] [US4] Create GitHub Actions workflow for backend tests in .github/workflows/backend-test.yml
- [ ] T078 [US4] Create backend deploy workflow for Railway/Render in .github/workflows/backend-deploy.yml

#### Quality Gates
- [ ] T079 [US4] Add Lighthouse CI for performance scoring in .github/workflows/lighthouse.yml
- [ ] T080 [US4] Configure branch protection rules with required checks

**Checkpoint**: User Story 4 complete - All CI/CD pipelines pass, frontend deploys automatically

---

## Phase 7: User Story 5 - Optional Enhancements (Priority: P5)

**Goal**: Add optional features without breaking free-tier limits

**Independent Test**: Optional features work when enabled, do not affect core functionality when disabled

### Implementation for User Story 5

#### Urdu Translation
- [ ] T081 [P] [US5] Create translation toggle component in src/components/TranslationToggle/TranslationToggle.tsx
- [ ] T082 [US5] Implement i18n configuration for Urdu in docusaurus.config.ts
- [ ] T083 [US5] Create Urdu translations for key UI strings in i18n/ur/

#### Personalized Learning
- [ ] T084 [P] [US5] Implement progress tracking localStorage utility in src/utils/progress.ts
- [ ] T085 [US5] Create personalized chapter recommendations component in src/components/Recommendations/Recommendations.tsx

#### Claude Code Integration
- [ ] T086 [P] [US5] Create Claude Code command for chapter generation in .claude/commands/generate-chapter.md
- [ ] T087 [P] [US5] Create Claude Code command for content review in .claude/commands/review-content.md
- [ ] T088 [US5] Document Claude Code workflows in .claude/README.md

**Checkpoint**: User Story 5 complete - Optional features functional, free-tier limits respected

---

## Phase 8: Polish & Finalization

**Purpose**: Final validation and launch preparation

- [ ] T089 Validate all chapters for content accuracy and formatting
- [ ] T090 Run full Docusaurus build and verify no errors
- [ ] T091 Run RAG backend integration tests
- [ ] T092 [P] Prepare 90-second demo video script
- [ ] T093 [P] Write final project documentation in docs/contributing.md
- [ ] T094 Create quickstart guide for local development in QUICKSTART.md
- [ ] T095 Final end-to-end system validation

**Checkpoint**: Project meets all success criteria from spec - ready for launch

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately [COMPLETE]
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-7)**: All depend on Foundational phase completion
  - User stories can proceed sequentially in priority order (P1 -> P2 -> P3 -> P4 -> P5)
  - US3 (Chatbot Frontend) depends on US2 (RAG Backend) being functional
  - US4 (CI/CD) can run in parallel with US2/US3
  - US5 (Optional) can be deferred or skipped entirely
- **Polish (Phase 8)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - No dependencies on US1
- **User Story 3 (P3)**: Depends on US2 (needs backend API to connect to)
- **User Story 4 (P4)**: Can start after Foundational (Phase 2) - Can run parallel to US2/US3
- **User Story 5 (P5)**: Can start after US1 completion - Optional, can be skipped

### Parallel Opportunities

**Within Phase 2 (Foundational):**
- T007, T008, T010, T011 can run in parallel

**Within User Story 1:**
- T014, T015, T016, T017 can run in parallel (different directories/files)
- All Week chapters (T018-T039) can be written in parallel by week
- Models marked [P] within each week can run in parallel

**Within User Story 2:**
- T043, T044 can run in parallel
- T048 and T056 can run in parallel

**Within User Story 3:**
- T060, T061 can run in parallel (different components)

**Within User Story 4:**
- T073, T074, T077 can run in parallel (different workflow files)

**Within User Story 5:**
- T081, T084, T086, T087 can run in parallel

---

## Parallel Example: User Story 1 Week Content

```bash
# Launch all Week 1-2 content together:
Task: "Write intro.md in docs/week-01-02-intro/intro.md"
Task: "Write setup.md in docs/week-01-02-intro/setup.md"

# Launch all ROS 2 chapters together:
Task: "Write ros2-overview.md in docs/week-03-05-ros2/ros2-overview.md"
Task: "Write nodes-topics.md in docs/week-03-05-ros2/nodes-topics.md"
Task: "Write urdf.md in docs/week-03-05-ros2/urdf.md"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup [DONE]
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 - Full textbook content
4. **STOP and VALIDATE**: Docusaurus builds, all chapters render correctly
5. Deploy to GitHub Pages for review

### Incremental Delivery

1. Complete Setup + Foundational -> Foundation ready
2. Add User Story 1 -> Test independently -> Deploy (MVP - Static Textbook!)
3. Add User Story 2 -> Test RAG backend independently
4. Add User Story 3 -> Test chatbot integration -> Deploy (Interactive Textbook!)
5. Add User Story 4 -> Automated CI/CD
6. Add User Story 5 -> Optional enhancements (if time permits)

### Suggested MVP Scope

**MVP = User Story 1 only** (Tasks T006-T041)
- Full Docusaurus site with all textbook chapters
- Navigation and search working
- Deployable to GitHub Pages
- Value delivered: Complete static textbook available online

---

## Summary

| Metric | Value |
|--------|-------|
| **Total Tasks** | 95 |
| **Phase 1 (Setup)** | 5 tasks (complete) |
| **Phase 2 (Foundational)** | 6 tasks |
| **User Story 1 (P1 - MVP)** | 30 tasks |
| **User Story 2 (P2)** | 17 tasks |
| **User Story 3 (P3)** | 14 tasks |
| **User Story 4 (P4)** | 8 tasks |
| **User Story 5 (P5)** | 8 tasks |
| **Phase 8 (Polish)** | 7 tasks |
| **Parallel Opportunities** | 35+ tasks marked [P] |

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Free-tier compliance: Use sentence-transformers (not OpenAI), Qdrant free tier, Neon free tier
