---
id: 5
title: Implement Images on Landing Page
stage: general
date: 2025-12-16
surface: agent
model: Qwen
feature: none
branch: main
user: assistant
command: /sp.implement
labels: [css, images, landing-page, frontend]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - G:\Hac\docs\src\pages\index.js
 - G:\Hac\docs\src\pages\index.module.css
tests:
 - none
---

## Prompt

i have 3 images in the public folder, implement these pictures on the landing page on the divs of Featured Lessons. also change the text color of Featured Lessons and additional resource  into matchable with UI . THank you

## Response snapshot

I have successfully updated the landing page to implement the images from the public folder on the Featured Lessons section and adjusted the text colors to match the UI. The changes included:

1. Updated the Featured Lessons section (LessonHighlight function) to use the images from the public folder:
   - Used /1.png for the first card (ROS 2 Framework)
   - Used /2.png for the second card (Humanoid Kinematics) 
   - Used /3.png for the third card (Physics Simulation)

2. Added custom CSS classes for the Featured Lessons and Additional Resources headings to ensure proper text coloring:
   - Added featuredLessonsTitle class for Featured Lessons heading
   - Added additionalResourcesTitle class for Additional Resources heading
   - Both classes set the text color to white to match the existing UI theme

3. The changes maintain all existing functionality while incorporating the requested images and text styling

The landing page now displays your custom images in the Featured Lessons section and the headings for both Featured Lessons and Additional Resources have text colors that match the overall UI theme.

## Outcome

- ✅ Impact: Images from public folder implemented on Featured Lessons section
- 🧪 Tests: None required as this was a frontend implementation
- 📁 Files: Modified index.js and index.module.css to implement images and text colors
- 🔁 Next prompts: None needed as requirements were fully implemented
- 🧠 Reflection: Successfully implemented client images with proper UI text colors

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A