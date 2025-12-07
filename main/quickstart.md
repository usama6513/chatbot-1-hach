# Quickstart

This document provides a high-level guide to setting up the development environment and running the Docusaurus site.

## Prerequisites

- Node.js (v18 or later)
- Yarn or npm
- Python 3.9+
- Docker

## Setup

### 1. Docusaurus Site

The documentation site is located in the `docs-site/` directory.

```bash
# Navigate to the docs site directory
cd docs-site

# Install dependencies
yarn install

# Start the development server
yarn start
```

The site will be available at `http://localhost:3000`.

### 2. Code Examples

The code examples reside in the `examples/` directory. Each example should have its own `README.md` with instructions on how to run it. Most examples will require a ROS 2 and Gazebo environment, which can be set up using the provided Docker containers.

```bash
# Navigate to the examples directory
cd examples

# Follow the instructions in the README of a specific example
cd ros2/my_first_node
# (Instructions to build and run will be here)
```

### 3. RAG Chatbot Backend

The RAG chatbot backend is a FastAPI application.

```bash
# Navigate to the backend directory (once created)
cd backend

# Create and activate a virtual environment
python -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Run the server
uvicorn main:app --reload
```

The API will be available at `http://localhost:8000`.

## End-to-End Testing

To test the full system:
1. Ensure the RAG chatbot backend is running.
2. Ensure the Docusaurus development server is running.
3. Open the Docusaurus site in your browser and navigate to a page with the chatbot component.
4. Interact with the chatbot and verify that it returns accurate responses based on the documentation content.
