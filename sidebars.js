// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1-ros2/module-1-ros2-index',
        'module-1-ros2/module-1-ros2-concepts',
        'module-1-ros2/module-1-ros2-toolchain',
        'module-1-ros2/module-1-ros2-implementation',
        'module-1-ros2/module-1-ros2-case-studies',
        'module-1-ros2/module-1-ros2-exercises',
        'module-1-ros2/module-1-ros2-debugging',
        'module-1-ros2/module-1-ros2-summary',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2-simulation/module-2-simulation-index',
        'module-2-simulation/module-2-simulation-concepts',
        'module-2-simulation/module-2-simulation-toolchain',
        'module-2-simulation/module-2-simulation-implementation',
        'module-2-simulation/module-2-simulation-case-studies',
        'module-2-simulation/module-2-simulation-exercises',
        'module-2-simulation/module-2-simulation-debugging',
        'module-2-simulation/module-2-simulation-summary',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module-3-ai-integration/module-3-ai-integration-index',
        'module-3-ai-integration/module-3-ai-integration-concepts',
        'module-3-ai-integration/module-3-ai-integration-toolchain',
        'module-3-ai-integration/module-3-ai-integration-implementation',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        'capstone/1',
        'capstone/2',
        'capstone/3',
        'capstone/4',
        'capstone/5',
      ],
    },
    {
      type: 'category',
      label: 'Additional Resources',
      items: [
        'hardware-requirements/1',
        'hardware-requirements/2',
        'hardware-requirements/3',
        'hardware-requirements/4',
        'safety-ethical-guidelines/1',
        'safety-ethical-guidelines/2',
        'safety-ethical-guidelines/3',
        'standards',
        'configuration',
        {
          type: 'category',
          label: 'API Documentation',
          items: [
            'api/api-index',
          ],
        },
        {
          type: 'category',
          label: 'Templates',
          items: [
            'templates/chapter-template',
          ],
        },
      ],
    },
  ],
};

module.exports = sidebars;