const fs = require('fs');
const path = require('path');

// Define a mapping of file paths to their proper IDs and titles
const fileMappings = {
  // API files
  'docs/api/index.md': { id: 'api-index', title: 'API Documentation' },

  // Root docs
  'docs/configuration.md': { id: 'configuration', title: 'Environment Configuration' },
  'docs/intro.md': { id: 'intro', title: 'Introduction to Physical AI & Humanoid Robotics' },
  'docs/standards.md': { id: 'standards', title: 'Standards and Guidelines' },
  'docs/templates/chapter-template.md': { id: 'chapter-template', title: 'Chapter Template' },

  // Module 1: ROS 2
  'docs/module-1-ros2/index.md': { id: 'module-1-ros2-index', title: 'Module 1: The Robotic Nervous System (ROS 2)' },
  'docs/module-1-ros2/concepts.md': { id: 'module-1-ros2-concepts', title: 'Core Concepts: ROS 2 Fundamentals' },
  'docs/module-1-ros2/toolchain.md': { id: 'module-1-ros2-toolchain', title: 'Toolchain: rclpy Python Integration' },
  'docs/module-1-ros2/implementation.md': { id: 'module-1-ros2-implementation', title: 'Implementation: Practical ROS 2 Examples' },
  'docs/module-1-ros2/case-studies.md': { id: 'module-1-ros2-case-studies', title: 'Case Studies: ROS 2 in Humanoid Robotics' },
  'docs/module-1-ros2/exercises.md': { id: 'module-1-ros2-exercises', title: 'Exercises: Hands-on ROS 2 Practice' },
  'docs/module-1-ros2/debugging.md': { id: 'module-1-ros2-debugging', title: 'Debugging: ROS 2 Troubleshooting' },
  'docs/module-1-ros2/summary.md': { id: 'module-1-ros2-summary', title: 'Summary: ROS 2 Key Takeaways' },

  // Module 2: Simulation
  'docs/module-2-simulation/index.md': { id: 'module-2-simulation-index', title: 'Module 2: Digital Twin (Gazebo & Unity)' },
  'docs/module-2-simulation/concepts.md': { id: 'module-2-simulation-concepts', title: 'Core Concepts: Physics Simulation Fundamentals' },
  'docs/module-2-simulation/toolchain.md': { id: 'module-2-simulation-toolchain', title: 'Toolchain: Gazebo and Unity Setup' },
  'docs/module-2-simulation/implementation.md': { id: 'module-2-simulation-implementation', title: 'Implementation: Practical Simulation Examples' },
  'docs/module-2-simulation/case-studies.md': { id: 'module-2-simulation-case-studies', title: 'Case Studies: Simulation in Robotics' },
  'docs/module-2-simulation/exercises.md': { id: 'module-2-simulation-exercises', title: 'Exercises: Hands-on Simulation Practice' },
  'docs/module-2-simulation/debugging.md': { id: 'module-2-simulation-debugging', title: 'Debugging: Simulation Troubleshooting' },
  'docs/module-2-simulation/summary.md': { id: 'module-2-simulation-summary', title: 'Summary: Simulation Key Takeaways' },

  // Module 3: AI Integration
  'docs/module-3-ai-integration/index.md': { id: 'module-3-ai-integration-index', title: 'Module 3: AI-Robot Brain (NVIDIA Isaac)' },
  'docs/module-3-ai-integration/concepts.md': { id: 'module-3-ai-integration-concepts', title: 'Core Concepts: Isaac Tools and Navigation' },
  'docs/module-3-ai-integration/toolchain.md': { id: 'module-3-ai-integration-toolchain', title: 'Toolchain: Isaac Sim and Isaac ROS Setup' },
  'docs/module-3-ai-integration/implementation.md': { id: 'module-3-ai-integration-implementation', title: 'Implementation: AI Integration Examples' },

  // Module 4: VLA (empty directory, will add placeholder if needed)

  // Capstone project
  'docs/capstone/index.md': { id: 'capstone-index', title: 'Capstone Project: Voice-Controlled Autonomous Humanoid System' },
  'docs/capstone/architecture.md': { id: 'capstone-architecture', title: 'Capstone Architecture: System Design and Component Integration' },
  'docs/capstone/implementation.md': { id: 'capstone-implementation', title: 'Capstone Implementation: Building the Voice-Controlled System' },
  'docs/capstone/evaluation.md': { id: 'capstone-evaluation', title: 'Capstone Evaluation: Success Criteria and Testing Procedures' },
  'docs/capstone/extensions.md': { id: 'capstone-extensions', title: 'Capstone Extensions: Advanced Features and Future Improvements' },

  // Hardware requirements
  'docs/hardware-requirements/index.md': { id: 'hardware-index', title: 'Hardware Requirements for Humanoid Robotics' },
  'docs/hardware-requirements/workstation.md': { id: 'hardware-workstation', title: 'Workstation Specifications for Humanoid Robotics Development' },
  'docs/hardware-requirements/jetson.md': { id: 'hardware-jetson', title: 'Jetson Edge Computing for Humanoid Robotics' },
  'docs/hardware-requirements/robot-options.md': { id: 'hardware-robot-options', title: 'Humanoid Robot Platforms: Unitree Go2, G1, and Other Options' },

  // Safety and ethics
  'docs/safety-ethical-guidelines/index.md': { id: 'safety-index', title: 'Safety and Ethics in Humanoid Robotics' },
  'docs/safety-ethical-guidelines/safety.md': { id: 'safety-considerations', title: 'Safety Considerations for Physical Humanoid Systems' },
  'docs/safety-ethical-guidelines/ethics.md': { id: 'ethics-guidelines', title: 'Ethical Guidelines for Humanoid Robotics Development' }
};

function fixFrontmatter(filePath) {
  if (!fileMappings[filePath]) {
    console.log(`⚠️  No mapping found for: ${filePath}`);
    return false;
  }

  const mapping = fileMappings[filePath];

  try {
    const content = fs.readFileSync(filePath, 'utf8');

    // Check if the file already has proper frontmatter
    if (content.startsWith('---')) {
      const lines = content.split('\n');
      let frontmatterEnd = -1;
      for (let i = 1; i < lines.length; i++) {
        if (lines[i] === '---') {
          frontmatterEnd = i;
          break;
        }
      }

      if (frontmatterEnd !== -1) {
        const frontmatter = lines.slice(1, frontmatterEnd).join('\n');

        // Check if id and title are already present
        if (frontmatter.includes('id:') && frontmatter.includes('title:')) {
          console.log(`✅ Already has proper frontmatter: ${filePath}`);
          return true;
        }
      }
    }

    // Extract current frontmatter and content
    let contentWithoutOldFrontmatter = content;
    let sidebarPosition = null;

    if (content.startsWith('---')) {
      const lines = content.split('\n');
      let frontmatterEnd = -1;
      for (let i = 1; i < lines.length; i++) {
        if (lines[i] === '---') {
          frontmatterEnd = i;
          break;
        }
      }

      if (frontmatterEnd !== -1) {
        const oldFrontmatter = lines.slice(1, frontmatterEnd).join('\n');

        // Extract sidebar_position if it exists
        const positionMatch = oldFrontmatter.match(/sidebar_position:\s*(\d+)/);
        if (positionMatch) {
          sidebarPosition = positionMatch[1];
        }

        contentWithoutOldFrontmatter = lines.slice(frontmatterEnd + 1).join('\n');
      }
    }

    // Create new frontmatter
    let newFrontmatter = `---\nid: ${mapping.id}\ntitle: "${mapping.title}"\n`;
    if (sidebarPosition !== null) {
      newFrontmatter += `sidebar_position: ${sidebarPosition}\n`;
    }
    newFrontmatter += `---\n`;

    // Combine new frontmatter with content
    const newContent = newFrontmatter + contentWithoutOldFrontmatter;

    // Write the updated content back to the file
    fs.writeFileSync(filePath, newContent);

    console.log(`✅ Fixed frontmatter: ${filePath}`);
    return true;
  } catch (error) {
    console.error(`❌ Error processing ${filePath}:`, error.message);
    return false;
  }
}

function main() {
  console.log('Fixing frontmatter for all documentation files...\n');

  let successCount = 0;
  let totalCount = Object.keys(fileMappings).length;

  for (const filePath of Object.keys(fileMappings)) {
    if (fixFrontmatter(filePath)) {
      successCount++;
    }
  }

  console.log(`\nCompleted: ${successCount}/${totalCount} files processed successfully`);

  if (successCount === totalCount) {
    console.log('🎉 All frontmatter fixes completed successfully!');
    return true;
  } else {
    console.log('⚠️  Some files may not have been processed successfully');
    return false;
  }
}

// Run the script
main();