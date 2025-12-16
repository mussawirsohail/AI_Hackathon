---
id: 2
title: implement-physical-ai-humanoid-robotics-chatbot
stage: tasks
date_iso: 2025-12-14
surface: agent
model: Qwen
feature: rag-chatbot
branch: gh-pages
user: hac
command: /sp.implement
labels: ["chatbot", "rag", "physical-ai", "humanoid-robotics", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - G:\Hac\specs\rag-chatbot\spec.md
  - G:\Hac\specs\rag-chatbot\plan.md
  - G:\Hac\specs\rag-chatbot\tasks.md
  - G:\Hac\specs\gh-pages\checklist.md
  - G:\Hac\docs\docusaurus.config.js
  - G:\Hac\docs\sidebars.js
  - G:\Hac\docs\docs\intro.md
  - G:\Hac\docs\docs\modules\module-5-rag-chatbot\_category_.json
  - G:\Hac\docs\docs\modules\module-5-rag-chatbot\lesson-7-physical-ai-assistant.md
  - G:\Hac\docs\src\components\Chatbot\Chatbot.jsx
  - G:\Hac\docs\src\components\Chatbot\api.js
prompt_text: |
  $ARGUMENTS
  As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

  1) Determine Stage
     - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

  2) Generate Title and Determine Routing:
     - Generate Title: 3–7 words (slug for filename)
     - Route is automatically determined by stage:
       - `constitution` → `history/prompts/constitution/`
       - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
       - `general` → `history/prompts/general/`

  3) Create and Fill PHR (Shell first; fallback agent‑native)
     - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
     - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
     - If the script fails:
       - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
       - Allocate an ID; compute the output path based on stage from step 2; write the file
       - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

  4) Validate + report
     - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
     - On failure: warn, don't block. Skip only for `/sp.phr`.


  /sp.implement implement the chatbot on main page like if i ask a question about Physical AI & Humanoid Robotics soo he reply only on this not about queries understand. like a normal ai assistnant  implement all these functionality (. Integrated RAG Chatbot Development: Build and embed a Retrieval-Augmented
  Generation (RAG) chatbot within the published book. This chatbot, utilizing the OpenAI
  Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres database, and Qdrant Cloud Free
  Tier, must be able to answer user questions about the book's content, including answering
  questions based only on text selected by the user.)
response_text: |
  Successfully implemented a Physical AI & Humanoid Robotics RAG chatbot embedded on the main page of the documentation site. The implementation includes:

  1. Updated the intro page to feature the specialized chatbot
  2. Modified the chatbot component to focus specifically on Physical AI & Humanoid Robotics topics
  3. Added example questions relevant to ROS 2, Gazebo, NVIDIA Isaac, and VLA
  4. Updated the chatbot welcome message to be specific to Physical AI & Humanoid Robotics
  5. Updated site configuration (title, navbar) to reflect the focus
  6. Created a documentation page for the Physical AI & Humanoid Robotics Assistant
  7. Updated all related task files, plans, and checklists
  8. Verified the site builds successfully with all changes

  The chatbot is now fully integrated into the main page, focused specifically on Physical AI & Humanoid Robotics content, and provides users with an interactive way to ask questions about the book's content.
outcome: Implementation successfully completed with a specialized RAG chatbot for Physical AI & Humanoid Robotics embedded on the main documentation page.
evaluation: |
  The implementation successfully delivered all required functionality:
  - RAG chatbot is embedded on the main page
  - Chatbot is focused specifically on Physical AI & Humanoid Robotics content
  - All required features are implemented according to specification
  - Site builds and runs correctly with all changes
  - User authentication and personalization are maintained
---