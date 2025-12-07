import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Foundations of Physical AI',
      items: ['intro'],
    },
    {
      type: 'category',
      label: 'Module 1: Robotic Nervous System (ROS 2)',
      items: [
        'module1/module-1-ros2-core',
        'lab-ros2-basic', // This is lab from chapter 4
        'module1/module-1-ros2-nervous-system', // New module
        // Assuming chapter 5 will be 'module-1-ros2-advanced'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin – Gazebo & Unity',
      items: [
        'module2/module-2-gazebo',
        'lab-urdf', // This is lab from chapter 7
        'module2/module-2-unity',
        'module2/module-2-digital-twin-simulation', // New module
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI Brain – NVIDIA Isaac',
      items: [
        'module3/module-3-isaac-sim',
        'module3/module-3-isaac-ros',
        'lab-sim2real', // This is lab from chapter 11
        'module3/module-3-ai-robot-brain-nvidia', // New module
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action Systems',
      items: [
        'module4/module-4-voice',
        'module4/module-4-planning',
        'module4/module-4-vla',
        'module4/module-4-vla-llm-robotics', // New module
      ],
    },
    {
      type: 'category',
      label: 'Capstone – Autonomous Humanoid',
      items: [
        'capstone-architecture',
        'capstone-implementation',
      ],
    },
//     {
//       type: 'category',
//       label: 'Cloud vs On-Prem Lab Setup',
//       items: [
//         // Assuming chapter 16 will be 'module-6-lab-setup'
//       ],
//     },
//     {
//       type: 'category',
//       label: 'Safety, Ethics & Deployment',
//       items: [
//         // Assuming chapter 17 will be 'module-7-safety-ethics'
//         // Assuming chapter 18 will be 'module-7-deployment'
//       ],
//     },
  ],
};

export default sidebars;
