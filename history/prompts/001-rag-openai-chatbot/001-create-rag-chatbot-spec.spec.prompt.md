---
id: 001
title: Create RAG Chatbot Specification
stage: spec
date: 2025-12-14
surface: agent
model: claude-opus-4-5-20251101
feature: 001-rag-openai-chatbot
branch: 001-rag-openai-chatbot
user: CBM
command: /sp.specify
labels: ["rag", "chatbot", "openai", "specification"]
links:
  spec: specs/001-rag-openai-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-rag-openai-chatbot/spec.md
  - specs/001-rag-openai-chatbot/checklists/requirements.md
tests:
  - null
---

## Prompt

# RAG Chatbot Specification (ChatKit + OpenAI)

## Goal
Add a Retrieval-Augmented Generation (RAG) chatbot to the Physical AI Textbook project.
The chatbot must answer questions strictly using the embedded textbook content stored in Qdrant.

## Scope
- Use existing textbook embeddings already stored in Qdrant Cloud
- Expose a backend API for question answering
- Provide a frontend chatbot UI
- Use OpenAI as the only LLM provider
- No Anthropic / Claude dependency

## LLM Configuration
- Provider: OpenAI
- Model: gpt-4o-mini
- API Key: OPENAI_API_KEY from environment variables

## Retrieval (RAG)
- Vector database: Qdrant Cloud
- Collection: existing textbook embeddings
- Embedding model already used: Cohere embed-english-v3.0 (1024 dimensions)
- Retrieval: semantic similarity search
- Retrieved chunks must be injected as context into the LLM prompt

## Backend Requirements
- Create a POST endpoint: /api/ask
- Input: user question (string)
- Process:
  1. Convert question to vector
  2. Query Qdrant for relevant textbook chunks
  3. Send retrieved context + question to OpenAI
- Output: generated answer + optional sources
- Enable streaming responses if supported

## Frontend Requirements
- Route: /chat
- Chat UI using ChatKit
- Features:
  - Message bubbles (user / assistant)
  - Input field + send button
  - Loading / typing indicator
  - Streaming responses
- The UI must call /api/ask

## Constraints
- Do not re-embed textbook content
- Do not use Anthropic or Claude
- Answers must rely only on retrieved textbook context

## Non-Goals
- No fine-tuning
- No external knowledge beyond textbook

## Response snapshot

Created comprehensive feature specification for RAG Chatbot with OpenAI including:
- 4 prioritized user stories (P1: Ask questions, P2: View sources, P2: Handle out-of-scope, P3: Conversational experience)
- 15 functional requirements covering chat UI, API endpoint, embedding, retrieval, and LLM integration
- 7 measurable success criteria
- 5 edge cases identified
- Key entities defined (Question, Answer, TextbookChunk, Source, ChatMessage)
- Clear assumptions and out-of-scope items documented
- Quality checklist created and validated (all items passed)

## Outcome

- Impact: Created complete feature specification ready for planning phase
- Tests: N/A (specification phase)
- Files: 2 files created (spec.md, checklists/requirements.md)
- Next prompts: /sp.clarify or /sp.plan to proceed with implementation planning
- Reflection: User provided comprehensive requirements; no clarifications needed due to detailed input

## Evaluation notes (flywheel)

- Failure modes observed: None - user input was comprehensive
- Graders run and results (PASS/FAIL): Specification quality checklist PASS (all items checked)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
