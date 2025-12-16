import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Manual sidebar for the Physical AI & Humanoid Robotics book
  tutorialSidebar: [
    'intro',
    'getting-started',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      link: {type: 'doc', id: 'module-1-the-robotic-nervous-system/intro'},
      items: [
        'module-1-the-robotic-nervous-system/lesson-1-introduction-to-ros2',
        'module-1-the-robotic-nervous-system/lesson-2-nodes-topics-services',
        'module-1-the-robotic-nervous-system/lesson-3-bridging-python-agents',
        'module-1-the-robotic-nervous-system/lesson-4-understanding-urdf'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      link: {type: 'doc', id: 'module-2-the-digital-twin/intro'},
      items: [
        'module-2-the-digital-twin/lesson-1-introduction-to-gazebo',
        'module-2-the-digital-twin/lesson-2-physics-simulation',
        'module-2-the-digital-twin/lesson-3-high-fidelity-rendering',
        'module-2-the-digital-twin/lesson-4-sensor-simulation'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      link: {type: 'doc', id: 'module-3-the-ai-robot-brain/intro'},
      items: [
        'module-3-the-ai-robot-brain/lesson-1-introduction-to-nvidia-isaac',
        'module-3-the-ai-robot-brain/lesson-2-photorealistic-simulation',
        'module-3-the-ai-robot-brain/lesson-3-hardware-accelerated-vslam',
        'module-3-the-ai-robot-brain/lesson-4-path-planning-for-bipedal-movement'
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      link: {type: 'doc', id: 'module-4-vision-language-action/intro'},
      items: [
        'module-4-vision-language-action/lesson-1-introduction-to-vla',
        'module-4-vision-language-action/lesson-2-voice-to-action-whisper',
        'module-4-vision-language-action/lesson-3-cognitive-planning-llms',
        'module-4-vision-language-action/lesson-4-capstone-project'
      ],
    },
  ],
};

export default sidebars;
