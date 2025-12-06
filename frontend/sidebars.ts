import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Tutorial - Basics',
      items: [
        'tutorial-basics/congratulations',
        'tutorial-basics/create-a-blog-post',
        'tutorial-basics/create-a-document',
        'tutorial-basics/create-a-page',
        'tutorial-basics/deploy-your-site',
        'tutorial-basics/markdown-features',
      ],
    },
    {
      type: 'category',
      label: 'Tutorial - Extras',
      items: [
        'tutorial-extras/manage-docs-versions',
        'tutorial-extras/translate-your-site',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        '01-module-ros/01-intro-to-ros2',
        '01-module-ros/02-nodes-topics-services',
        '01-module-ros/03-rclpy-integration',
        '01-module-ros/04-urdf-for-humanoids',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        '02-module-digital-twin/01-gazebo-setup',
        '02-module-digital-twin/02-physics-simulation',
        '02-module-digital-twin/03-sensor-simulation',
        '02-module-digital-twin/04-unity-rendering',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        '03-module-nvidia-isaac/01-isaac-sim-intro',
        '03-module-nvidia-isaac/02-isaac-ros-vslam',
        '03-module-nvidia-isaac/03-nav2-path-planning',
        '03-module-nvidia-isaac/04-reinforcement-learning',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        '04-module-vla/01-voice-to-action',
        '04-module-vla/02-cognitive-planning-llms',
        '04-module-vla/03-gpt-integration',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        '05-capstone-project/01-autonomous-humanoid',
      ],
    },
  ],
};

export default sidebars;
