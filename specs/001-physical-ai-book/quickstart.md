# Quickstart Guide: Physical AI & Humanoid Robotics Book

## Getting Started

This guide will help you set up and start contributing to the Physical AI & Humanoid Robotics book project.

### Prerequisites

- Node.js v18 or higher
- npm or yarn package manager
- Git for version control
- Basic knowledge of Markdown syntax
- Basic knowledge of Docusaurus (optional but helpful)

### Setup Instructions

1. **Clone the repository**
   ```bash
   git clone [repository-url]
   cd [repository-name]
   ```

2. **Install dependencies**
   ```bash
   npm install
   # or
   yarn install
   ```

3. **Start the development server**
   ```bash
   npm run start
   # or
   yarn start
   ```
   This will start a local development server at `http://localhost:3000` with hot reloading enabled.

4. **Verify the setup**
   - Open your browser and navigate to `http://localhost:3000`
   - You should see the Docusaurus site with the Physical AI & Humanoid Robotics book content

### Project Structure Overview

The main content files are organized as follows:

```
docs/
├── module-1-the-robotic-nervous-system/
│   ├── lesson-1-introduction-to-ros2.md
│   ├── lesson-2-nodes-topics-services.md
│   ├── lesson-3-bridging-python-agents.md
│   └── lesson-4-understanding-urdf.md
├── module-2-the-digital-twin/
│   ├── lesson-1-introduction-to-gazebo.md
│   ├── lesson-2-physics-simulation.md
│   ├── lesson-3-high-fidelity-rendering.md
│   └── lesson-4-sensor-simulation.md
├── module-3-the-ai-robot-brain/
│   ├── lesson-1-introduction-to-nvidia-isaac.md
│   ├── lesson-2-photorealistic-simulation.md
│   ├── lesson-3-hardware-accelerated-vslam.md
│   └── lesson-4-path-planning-for-bipedal-movement.md
├── module-4-vision-language-action/
│   ├── lesson-1-introduction-to-vla.md
│   ├── lesson-2-voice-to-action-whisper.md
│   ├── lesson-3-cognitive-planning-llms.md
│   └── lesson-4-capstone-project.md
├── intro.md
├── getting-started.md
├── _category_.json
└── assets/
    ├── diagrams/
    └── code-examples/
        ├── python/
        ├── cpp/
        └── ros/
```

### Creating New Content

1. **Adding a new lesson**
   - Navigate to the appropriate module directory
   - Create a new `.md` file with a descriptive name
   - Follow the standard lesson template (see existing lessons for examples)
   - Update the `_category_.json` file in the module directory to add navigation

2. **Adding code examples**
   - Place code files in the appropriate subdirectory under `docs/assets/code-examples/`
   - Reference the code files in your lesson content using Docusaurus' code block features

3. **Adding diagrams/images**
   - Place image files in the `docs/assets/diagrams/` directory
   - Reference images in your lessons using Markdown syntax

### Lesson Template

Each lesson should follow this general structure:

```markdown
---
title: [Lesson Title]
sidebar_position: [1-4]
description: [Brief description of the lesson]
---

# [Lesson Title]

## Learning Objectives

After completing this lesson, you will be able to:
- [Objective 1]
- [Objective 2]
- [Objective 3]

## Introduction

[Lesson introduction and context]

## Content

[Main lesson content with code examples, diagrams, and explanations]

## Hands-On Exercise

[Practical exercise for the student to complete]

## Summary

[Brief summary of key points covered in the lesson]

## Self-Assessment

1. [Question 1]
2. [Question 2]
3. [Question 3]
```

### Building for Production

To build a static version of the site for deployment:

```bash
npm run build
# or
yarn build
```

This will create a `build/` directory with the complete static site ready for deployment.

### Deployment

The site is configured for deployment to GitHub Pages. After making changes:

1. Commit your changes
2. Push to the main branch
3. The site will be automatically deployed via GitHub Actions

For more detailed documentation about Docusaurus, refer to the [official Docusaurus documentation](https://docusaurus.io/).