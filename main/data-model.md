# Data Model

This document outlines the data models for the primary entities in the project.

---

## 1. Documentation Module

This entity represents a top-level section of the documentation.

- **`id`**: `string` (e.g., "ros2-basics") - A unique identifier for the module.
- **`title`**: `string` (e.g., "ROS 2 Basics") - The human-readable title.
- **`summary`**: `string` - A brief description of the module's content.
- **`tutorials`**: `Array<Tutorial>` - A list of tutorials belonging to this module.

---

## 2. Tutorial

This entity represents a single tutorial or article within a module.

- **`id`**: `string` (e.g., "creating-a-ros2-node") - A unique identifier for the tutorial.
- **`title`**: `string` (e.g., "Creating a ROS 2 Node") - The human-readable title.
- **`moduleId`**: `string` - The ID of the parent module.
- **`content`**: `string` (Markdown) - The body of the tutorial.
- **`codeExamples`**: `Array<CodeExample>` - A list of associated code examples.
- **`estimatedReadingTime`**: `number` (minutes) - An automatically calculated reading time.

---

## 3. Code Example

This entity represents a code snippet or runnable example referenced in a tutorial.

- **`id`**: `string` (e.g., "hello-world-publisher-py") - A unique identifier for the code example.
- **`sourcePath`**: `string` - The path to the source file in the separate `examples/` repository.
- **`language`**: `string` (e.g., "python", "cpp") - The programming language.
- **`description`**: `string` - An optional description of what the code does.

---

## 4. Chatbot Knowledge Item

This entity represents a chunk of text indexed in the RAG chatbot's vector database.

- **`id`**: `string` (UUID) - A unique identifier for the chunk.
- **`sourceTutorialId`**: `string` - The ID of the tutorial this chunk was extracted from.
- **`text`**: `string` - The raw text content of the chunk.
- **`vector`**: `Array<number>` - The embedding vector representing the text.
- **`metadata`**: `object` - Additional metadata for filtering, such as `moduleId` and `tutorialTitle`.

**State Transitions**:
- **`unindexed`**: The text has been identified for indexing but has not been processed.
- **`indexed`**: The text has been converted to a vector and stored in the vector database.
- **`stale`**: The source tutorial has been updated, and this chunk needs to be re-indexed.
- **`archived`**: The source tutorial has been deleted, and this chunk should be removed from the index.
