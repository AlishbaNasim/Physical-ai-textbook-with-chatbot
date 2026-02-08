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
  // Manual sidebar structure for the Physical AI & Humanoid Robotics textbook
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
    },
    {
      type: 'category',
      label: 'Module 1: Physical AI Foundations',
      collapsed: false,
      items: [
        'module-1-physical-foundations/index',
        'module-1-physical-foundations/chapter-1-overview',
        'module-1-physical-foundations/chapter-2-embodied-ai',
        'module-1-physical-foundations/chapter-3-digital-to-physical',
        'module-1-physical-foundations/chapter-4-learning-objectives',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: ROS 2 - The Nervous System',
      collapsed: false,
      items: [
        'module-2-ros-nervous-system/index',
        'module-2-ros-nervous-system/chapter-1-ros-fundamentals',
        'module-2-ros-nervous-system/chapter-2-message-passing',
        'module-2-ros-nervous-system/chapter-3-control-architectures',
        'module-2-ros-nervous-system/chapter-4-humanoid-integration',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Simulation-First Development',
      collapsed: false,
      items: [
        'module-3-simulation-first/index',
        'module-3-simulation-first/chapter-1-gazebo-workflows',
        'module-3-simulation-first/chapter-2-unity-simulations',
        'module-3-simulation-first/chapter-3-isaac-sim-applications',
        'module-3-simulation-first/chapter-4-sim-to-real',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA & Conversational Robotics',
      collapsed: false,
      items: [
        'module-4-vla-and-conversational-robotics/index',
        'module-4-vla-and-conversational-robotics/chapter-1-vla-systems',
        'module-4-vla-and-conversational-robotics/chapter-2-llm-integration',
        'module-4-vla-and-conversational-robotics/chapter-3-conversational-control',
        'module-4-vla-and-conversational-robotics/chapter-4-capstone-project',
      ],
    },
  ],
};

export default sidebars;
