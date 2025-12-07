# Physical AI & Humanoid Robotics Book Constitution

<!--
SYNC IMPACT REPORT - v1.0.0 (Initial)
- Version change: [Template] → 1.0.0 (Initial ratification)
- Added principles: 7 core principles + 3 sections (Constraints, Development Workflow, Governance)
- Key principles: Spec-First, Educational Clarity, Reproducibility, Modular Architecture, Quality Gates, Tool Stack Adherence, Open Integration
- Templates status: ✅ Plan template aligned | ✅ Spec template aligned | ✅ Tasks template aligned
- Follow-up TODOs: None
-->

## Core Principles

### I. Spec-First Development

Every feature, module, and deliverable begins with a written specification before any code or content is produced. Specifications include user scenarios, functional requirements, success criteria, and acceptance conditions. Ambiguity must be resolved in writing before implementation begins.

**Rationale**: Clarity-first reduces rework, ensures alignment with educational goals, and creates a reference document for validation and reproducibility.

---

### II. Educational Clarity

All code, tutorials, and documentation must be understandable by developers learning robotics and AI—not experts. Use plain language, step-by-step explanations, visual diagrams where helpful, and worked examples. Technical depth is mandatory, but accessibility is non-negotiable.

**Rationale**: The project's goal is to teach; unclear or overly terse code undermines that mission and reduces adoption.

---

### III. Full Reproducibility

Every code example, tutorial, and capstone project must be runnable end-to-end in the specified environment (ROS 2, Gazebo/Unity, NVIDIA Isaac Sim, or VLA tools) without undocumented steps, external credentials, or manual setup. Include seed data, environment files, Docker containers, or exact version pins as needed.

**Rationale**: Readers must be able to replicate projects to learn effectively. Reproducibility is the foundation of trust and learning.

---

### IV. Modular Architecture

The book, chatbot, and code examples must be organized into four independent but interconnected modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA). Each module should be learnable and implementable independently while also showing how to integrate with other modules.

**Rationale**: Modularity allows readers to focus on one topic without mastering all four; integration examples show the full picture for advanced learners.

---

### V. Quality Gates (Non-Negotiable)

- **Code Quality**: All code must be tested, linted, and documented before merging.
- **Documentation**: All new features/modules must include a quickstart guide and at least one worked example.
- **Integration Testing**: Chatbot must correctly retrieve and rank book content; examples must run in target environments.
- **Accessibility**: All content (text, code, diagrams) must be readable and understandable by the target audience.

**Rationale**: Quality gates ensure the final product is production-ready and educational, meeting the success criteria for live deployment and reader success.

---

### VI. Tool Stack Adherence

Use only the specified tools and frameworks:
- **Book**: Docusaurus deployed to GitHub Pages.
- **Chatbot**: OpenAI SDKs, FastAPI, Neon Postgres, Qdrant Cloud for embedding storage.
- **Code**: Python and C++ only for the specified robotics frameworks (ROS 2, Gazebo, NVIDIA Isaac, VLA).
- **Infrastructure**: GitHub for version control; Docker for reproducible environments.

**Rationale**: Strict adherence ensures the project stays focused, reduces scope creep, and creates a tightly integrated product that readers can understand end-to-end.

---

### VII. Open Integration & Extensibility

The book, code, and chatbot should be designed so advanced readers and contributors can extend modules, add new capstone projects, or integrate with additional robotics tools. Clear APIs, documented architecture, and modular structure enable this.

**Rationale**: Educational projects thrive when the community can build upon them; this principle ensures the project remains useful as robotics and AI evolve.

---

## Constraints & Non-Negotiables

- **Technology**: Only ROS 2, Gazebo/Unity, NVIDIA Isaac Sim, and VLA tools as specified. No framework substitutions without explicit approval.
- **Deployment**: Book must be live on GitHub Pages with zero manual deployment steps. CI/CD pipeline required.
- **Chatbot**: Must use OpenAI for embeddings, FastAPI for backend, Neon Postgres + Qdrant for storage. No substitutions.
- **Language**: English for all documentation; code comments in English.
- **Scope**: 4 modules + capstone project guide. No out-of-scope robotics platforms unless explicitly added.

---

## Development Workflow

### For Book Content
1. Create feature spec (user scenarios, requirements, acceptance criteria).
2. Write content in Docusaurus-compatible Markdown with working code examples.
3. Embed all code examples; test them in the target environment.
4. Link code to a GitHub repository with full instructions and Docker setup if needed.
5. Add to chatbot knowledge base once approved.

### For Code Examples & Tutorials
1. Write specification (what the example teaches, prerequisites, success criteria).
2. Implement the example with full comments explaining each step.
3. Test in the specified environment (ROS 2 container, Gazebo, Isaac Sim, or VLA).
4. Include a `quickstart.md` with step-by-step reproduction instructions.
5. Document any external dependencies, API keys, or configuration files (with example .env).

### For Chatbot Integration
1. Prepare content (book sections + user-selected text) as embeddings.
2. Test retrieval accuracy and ranking against known queries.
3. Validate responses are sourced correctly and cite book sections.
4. Deploy updated embeddings to Qdrant Cloud.

### For Capstone Project
1. Define the capstone scenario (e.g., autonomous humanoid navigation).
2. Break into milestones: perception → planning → control → integration.
3. Create specs for each milestone with working code.
4. Integrate all modules from the 4 core topics.
5. Document the full end-to-end solution with deployment instructions.

---

## Governance

### Constitution Authority
This constitution supersedes all other development practices and guides. All PRs, issues, and design discussions must verify compliance with these principles and constraints.

### Amendment Process
1. Propose amendment in an issue or PR with clear rationale.
2. Document impact on existing artifacts (specs, plans, tasks, code).
3. Require explicit sign-off from the project lead.
4. Update version number per semantic versioning rules.
5. Propagate changes to dependent templates and documentation.

### Version Policy
- **MAJOR**: Principle removal, redefinition, or backward-incompatible constraint change.
- **MINOR**: New principle, new section, or materially expanded guidance.
- **PATCH**: Wording clarification, typo fixes, non-semantic refinements.

### Compliance Review
- All PRs must reference which principles they uphold.
- Monthly review: check for principle violations in open issues/PRs.
- Annual review: full audit of book, chatbot, and code against constitution.

### Escalation & Deviations
- If a principle cannot be met, document the blocker in an issue with clear rationale.
- Require explicit approval from the project lead to proceed with deviation.
- Document the deviation in the PR and create an ADR if it affects architecture.

---

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07
