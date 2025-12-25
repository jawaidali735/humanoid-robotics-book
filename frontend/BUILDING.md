# Building the Humanoid Robotics Book

This document provides instructions for building and running the Physical AI & Humanoid Robotics book project.

## Prerequisites

Before building the project, ensure you have:

- Node.js 18+ installed
- npm or yarn package manager
- Git version control system

## Installation

1. Clone the repository:
```bash
git clone https://github.com/your-organization/humanoid-robotics-book.git
cd humanoid-robotics-book
```

2. Install dependencies:
```bash
npm install
```

## Development Server

To start a local development server:

```bash
npm start
```

This will start the development server and open the site in your default browser at `http://localhost:3000`.

## Building for Production

To build the static site for production:

```bash
npm run build
```

The built site will be available in the `build/` directory and can be deployed to any static hosting service.

## Serving the Built Site

To serve the built site locally for testing:

```bash
npm run serve
```

## Troubleshooting

### Common Issues

1. **Module not found errors**: Make sure all dependencies are installed with `npm install`

2. **Sidebar reference errors**: Ensure all files referenced in `sidebars.js` exist in the `docs/` directory

3. **Frontmatter errors**: All markdown files should have proper frontmatter with `id`, `title`, and `sidebar_position` fields

4. **Link validation errors**: Check that all internal links point to existing files

### Validation

The project includes validation scripts to check content quality:

```bash
node scripts/validate-content.js
node scripts/check-sidebar.js
```

## Project Structure

- `docs/` - All documentation content
- `src/` - Custom React components and CSS
- `static/` - Static assets (images, files)
- `docusaurus.config.js` - Main Docusaurus configuration
- `sidebars.js` - Navigation sidebar configuration
- `package.json` - Project dependencies and scripts

## Content Guidelines

All documentation files should follow the 9-section template:

1. Introduction
2. Learning Outcomes
3. Conceptual Foundations
4. Technical Deep Dive
5. Practical Implementation
6. Common Pitfalls and Solutions
7. Industry Use Cases
8. Summary and Key Takeaways
9. Practice Tasks

Each file should include proper frontmatter:

```markdown
---
id: unique-identifier
title: "Page Title"
sidebar_position: 1
---
```

## Deployment

The site can be deployed to GitHub Pages, Netlify, Vercel, or any static hosting service. For GitHub Pages specifically, the configuration is already set up in `docusaurus.config.js`.

## Additional Scripts

- `scripts/validate-content.js` - Validates all content and structure
- `scripts/check-sidebar.js` - Checks sidebar coverage and structure
- `scripts/fix-frontmatter.js` - Fixes missing frontmatter in files