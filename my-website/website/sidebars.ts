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
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: Introduction to AI/Spec-Driven Development',
      items: [
        'module-1/chapter-1',
        'module-1/chapter-2',
        'module-1/chapter-3'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2/chapter-1',
        'module-2/chapter-2',
        'module-2/chapter-3'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3/index',
        {
          type: 'category',
          label: 'NVIDIA Isaac Sim',
          items: [
            'module-3/isaac-sim/index',
            'module-3/isaac-sim/setup',
            'module-3/isaac-sim/simulation',
            'module-3/isaac-sim/photorealistic-rendering',
            'module-3/isaac-sim/synthetic-data-generation',
            'module-3/isaac-sim/examples'
          ],
        },
        {
          type: 'category',
          label: 'Isaac ROS',
          items: [
            'module-3/isaac-ros/index',
            'module-3/isaac-ros/vsalm',
            'module-3/isaac-ros/navigation',
            'module-3/isaac-ros/tutorials'
          ],
        },
        {
          type: 'category',
          label: 'Nav2 Humanoid Path Planning',
          items: [
            'module-3/nav2-humanoid/index',
            'module-3/nav2-humanoid/path-planning',
            'module-3/nav2-humanoid/humanoid-nav',
            'module-3/nav2-humanoid/implementation'
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4/chapter-1',
        'module-4/chapter-2',
        'module-4/chapter-3'
      ],
    },
    {
      type: 'category',
      label: 'Additional Resources',
      items: [
        'contributing',
        'embedding-schema',
        'rag-chatbot-roadmap'
      ],
    },
  ],
};


export default sidebars;
