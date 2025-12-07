import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-ros/intro-to-ros2',
        'module-ros/nodes-topics-services',
        'module-ros/rclpy-integration',
        'module-ros/urdf-for-humanoids',
        {
          type: 'category',
          label: 'Module 1 Quizzes',
          items: [
            'module-ros/quizzes/quiz-intro-to-ros2',
            'module-ros/quizzes/quiz-nodes-topics-services',
            'module-ros/quizzes/quiz-rclpy-integration',
            'module-ros/quizzes/quiz-urdf-for-humanoids',
          ],
        },
        {
          type: 'category',
          label: 'Module 1 Try with AI',
          items: [
            'module-ros/try-with-ai/ai-intro-to-ros2',
            'module-ros/try-with-ai/ai-nodes-topics-services',
            'module-ros/try-with-ai/ai-rclpy-integration',
            'module-ros/try-with-ai/ai-urdf-for-humanoids',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-digital-twin/gazebo-setup',
        'module-digital-twin/physics-simulation',
        'module-digital-twin/sensor-simulation',
        'module-digital-twin/unity-rendering',
        {
          type: 'category',
          label: 'Module 2 Quizzes',
          items: [
            'module-digital-twin/quizzes/quiz-gazebo-setup',
            'module-digital-twin/quizzes/quiz-physics-simulation',
            'module-digital-twin/quizzes/quiz-sensor-simulation',
            'module-digital-twin/quizzes/quiz-unity-rendering',
          ],
        },
        {
          type: 'category',
          label: 'Module 2 Try with AI',
          items: [
            'module-digital-twin/try-with-ai/ai-gazebo-setup',
            'module-digital-twin/try-with-ai/ai-physics-simulation',
            'module-digital-twin/try-with-ai/ai-sensor-simulation',
            'module-digital-twin/try-with-ai/ai-unity-rendering',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module-nvidia-isaac/isaac-sim-intro',
        'module-nvidia-isaac/isaac-ros-vslam',
        'module-nvidia-isaac/nav2-path-planning',
        'module-nvidia-isaac/reinforcement-learning',
        {
          type: 'category',
          label: 'Module 3 Quizzes',
          items: [
            'module-nvidia-isaac/quizzes/quiz-isaac-sim-intro',
            'module-nvidia-isaac/quizzes/quiz-isaac-ros-vslam',
            'module-nvidia-isaac/quizzes/quiz-nav2-path-planning',
            'module-nvidia-isaac/quizzes/quiz-reinforcement-learning',
          ],
        },
        {
          type: 'category',
          label: 'Module 3 Try with AI',
          items: [
            'module-nvidia-isaac/try-with-ai/ai-isaac-sim-intro',
            'module-nvidia-isaac/try-with-ai/ai-isaac-ros-vslam',
            'module-nvidia-isaac/try-with-ai/ai-nav2-path-planning',
            'module-nvidia-isaac/try-with-ai/ai-reinforcement-learning',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-vla/voice-to-action',
        'module-vla/cognitive-planning-llms',
        'module-vla/gpt-integration',
        {
          type: 'category',
          label: 'Module 4 Quizzes',
          items: [
            'module-vla/quizzes/quiz-voice-to-action',
            'module-vla/quizzes/quiz-cognitive-planning-llms',
            'module-vla/quizzes/quiz-gpt-integration',
          ],
        },
        {
          type: 'category',
          label: 'Module 4 Try with AI',
          items: [
            'module-vla/try-with-ai/ai-voice-to-action',
            'module-vla/try-with-ai/ai-cognitive-planning-llms',
            'module-vla/try-with-ai/ai-gpt-integration',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        'capstone-project/autonomous-humanoid',
        {
          type: 'category',
          label: 'Capstone Project Quizzes',
          items: [
            'capstone-project/quizzes/quiz-autonomous-humanoid',
          ],
        },
        {
          type: 'category',
          label: 'Capstone Project Try with AI',
          items: [
            'capstone-project/try-with-ai/ai-autonomous-humanoid',
          ],
        },
      ],
    },
  ],
};

export default sidebars;
