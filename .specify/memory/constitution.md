<!--
Sync Impact Report:
- Version change: N/A → 1.0.0
- Added sections: All principles and sections based on project requirements
- Modified principles: N/A (new constitution)
- Removed sections: N/A (new constitution)
- Templates requiring updates:
  - .specify/templates/plan-template.md → ✅ No changes needed (constitution check section will work with new principles)
  - .specify/templates/spec-template.md → ✅ No changes needed
  - .specify/templates/tasks-template.md → ✅ No changes needed
- No existing principles modified (new constitution)
- No placeholders deferred (all filled)
-->

# AI/Spec-Driven Book with Embedded RAG Chatbot Constitution

## Core Principles

### Spec-First Workflow
All development follows a spec-first workflow using Spec-Kit Plus; Every feature starts with a well-defined specification before implementation; Specifications must be complete and approved before coding begins

### Technical Accuracy
All technical content must be grounded in official sources and verified documentation; No hallucinated responses or fabricated information; Technical accuracy takes precedence over speed of delivery

### Developer-Focused Writing
All documentation and code examples must be clear, practical, and developer-focused; Content should be actionable with runnable, well-documented code examples; Clear, concise explanations prioritized over verbose descriptions

### Reproducible Setup
All setups and deployments must be fully reproducible; End-to-end reproducibility is required for all features; Clear, step-by-step setup instructions with no hidden dependencies

### Quality Standards
All code must be runnable, well-documented, and production-ready; GitHub-based source control is mandatory; All implementations must follow the established technology stack

### RAG Integrity
RAG chatbot responses must be grounded only in book content or user-selected text; No responses based on external knowledge or hallucination; Content relevance and accuracy strictly enforced

## Technology Standards
Book written with Docusaurus and deployed on GitHub Pages; RAG chatbot stack: OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant Cloud; All technology choices must align with the specified stack

## Success Criteria
Live book deployed on GitHub Pages; Fully functional embedded RAG chatbot; All specifications implemented via Spec-Kit Plus methodology; All specs successfully implemented according to requirements

## Governance
This constitution governs all development activities for the AI/Spec-Driven Book project; All code changes must comply with the specified principles and standards; Amendments require documentation and approval following the established procedures

**Version**: 1.0.0 | **Ratified**: 2025-12-29 | **Last Amended**: 2025-12-29