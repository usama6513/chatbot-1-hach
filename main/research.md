# Research & Decisions

This document summarizes the research and decisions made to resolve the "NEEDS CLARIFICATION" items from the implementation plan.

---

## 1. Documentation Modules

**Decision**: The four modules will be: 
1. ROS 2 Basics
2. Simulating Robots with Gazebo
3. Advanced Robotics with NVIDIA Isaac Sim
4. Vision and Language Agent (VLA) Integration

**Rationale**: This structure aligns with the constitution's specified modules (`IV. Modular Architecture`). It provides a logical learning progression, starting with the fundamentals of ROS 2, moving to simulation in Gazebo, then to more advanced simulation and AI with NVIDIA tools, and finally to cutting-edge VLA applications.

**Alternatives considered**: A different combination of modules was considered, but the chosen structure directly reflects the project's core technologies as defined in the constitution.

---

## 2. Embedding the RAG Chatbot

**Decision**: A custom React component will be developed for the chatbot's user interface. This component will be embedded directly into Docusaurus pages using MDX. It will communicate with a server-side API endpoint for the RAG logic.

**Rationale**: Docusaurus is built on React, allowing for seamless integration of custom components. This approach provides maximum flexibility for the chatbot's UI and user experience. Separating the frontend component from the backend API aligns with modern web development practices.

**Alternatives considered**: 
- A third-party chatbot widget: Rejected as it would offer less customization and might not meet the specific RAG requirements.
- A non-React solution: Rejected as it would be difficult to integrate into the Docusaurus (React) ecosystem.

---

## 3. Docusaurus Theme

**Decision**: The project will use the standard `@docusaurus/theme-classic` and leverage the `@docusaurus/plugin-content-docs` plugin for a technical documentation layout. The blog plugin will not be the primary focus.

**Rationale**: The user's request is for a structured set of tutorials and modules. This fits the features of the documentation plugin (sidebars, versioning, hierarchy) perfectly. The "theme" is the visual layer on top, and the classic theme is highly configurable to achieve a professional, technical look. A blog-first theme is unsuitable for this type of content.

**Alternatives considered**: 
- A blog-focused theme: Rejected because the content is not chronological and requires a more rigid structure.
- A completely custom theme: Rejected as it would add significant development overhead for little initial benefit.

---

## 4. Code Display Strategy

**Decision**: Code examples will be maintained in a separate repository (e.g., within the same monorepo but in a distinct `examples/` directory). A build-time script or a Docusaurus plugin (like `docusaurus-plugin-remote-content`) will be used to pull the code into the Docusaurus site.

**Rationale**: This approach establishes a "single source of truth" for all code. It allows code examples to be independently tested, linted, and maintained, which directly supports the "Full Reproducibility" principle (III) of the constitution. It avoids code duplication and prevents documentation from becoming out of sync with the actual working code.

**Alternatives considered**: 
- Inline code blocks: Rejected because it leads to code duplication and makes it difficult to verify that examples are correct and runnable. This is not suitable for a project focused on reproducibility.

---

## 5. Chatbot Architecture

**Decision**: A server-side RAG (Retrieval-Augmented Generation) architecture will be implemented.

**Rationale**: This decision directly aligns with the `VI. Tool Stack Adherence` principle of the constitution, which specifies a server-side stack (FastAPI, Neon Postgres, Qdrant Cloud). A server-side approach is necessary to handle large knowledge bases and run powerful, resource-intensive language models, ensuring the chatbot is scalable and effective.

**Alternatives considered**: 
- A client-side RAG architecture: Rejected due to the limitations of client devices (memory, processing power), which would restrict the size of the knowledge base and the quality of the generative model. It would also violate the specified tool stack in the constitution.
- A hybrid model: Considered too complex for the initial implementation. A pure server-side approach is more straightforward to build and maintain first.

---

## 6. Development Workflow

**Decision**: A parallel development workflow will be adopted for the content modules.

**Rationale**: The user's request explicitly stated a preference for parallel development ("Write modules in parallel using Claude Code"). The project's modular structure (4 distinct modules) is well-suited for this approach. Different teams or individuals can work on different modules concurrently, which can significantly accelerate the overall project timeline.

**Alternatives considered**: 
- A sequential workflow: Rejected because it would unnecessarily slow down the project. Since the modules are designed to be independent, there is no need to wait for one to be complete before starting the next.
