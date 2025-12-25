# Quickstart Guide: Futuristic Book Website UI

## Overview
This guide provides the essential steps to set up and run the futuristic book website UI for "Physical AI & Humanoid Robotics".

## Prerequisites
- Node.js 18+ installed
- npm or yarn package manager
- Git for version control

## Setup Instructions

### 1. Clone and Install Dependencies
```bash
git clone [repository-url]
cd [repository-name]
npm install
```

### 2. Start Development Server
```bash
npm start
```
This will start the Docusaurus development server and open the website at http://localhost:3000

### 3. Project Structure
```
src/
├── components/          # Custom React components
│   ├── ChatbotWidget/   # Floating chatbot implementation
│   ├── HeroSection/     # Hero section with animations
│   └── ...              # Other custom components
├── css/                 # Custom styles and design system
└── utils/               # Utility functions

docs/                    # Book content in Markdown
├── intro.md
├── chapter-1/
└── ...

static/                  # Static assets
└── img/                 # Images and icons
```

## Key Features

### 1. Landing Page Sections
- Hero section with animated 3D AI/robotics visuals
- Book overview and value proposition
- Learning outcomes feature cards
- Chapters preview
- Interactive sample section
- FAQ and footer

### 2. Reading Experience
- Optimized typography and line spacing
- Sticky table of contents with active section highlighting
- Dark theme with electric cyan/neon blue accents
- Code blocks styled for dark theme

### 3. Floating Chatbot
- Accessible via circular button in bottom-right corner
- Quick actions: "Explain this section", "Summarize chapter", "Go to related topic"
- Non-blocking overlay that never interferes with reading

## Development Commands

```bash
# Start development server
npm start

# Build for production
npm run build

# Serve production build locally
npm run serve

# Run tests
npm test

# Clear cache
npm run clear
```

## Customization Points

### 1. Design System
Edit `src/css/design-system.css` to modify:
- Color palette (dark theme, accent colors)
- Typography scale
- Spacing system
- Animation timing

### 2. Components
Customize individual components in `src/components/`:
- HeroSection for landing page header
- FeatureCards for learning outcomes display
- ChatbotWidget for AI assistant interface

### 3. Content
Add or modify book content in the `docs/` directory using Markdown format.

## Environment Variables
If needed, create a `.env` file in the root directory:
```
# API endpoints for chatbot integration
CHATBOT_API_URL=your-chatbot-api-url
```

## Deployment
The site is configured for GitHub Pages deployment:
```bash
# Build and deploy to GitHub Pages
npm run deploy
```

## Troubleshooting
- If the chatbot doesn't appear, check that the API endpoint is configured correctly
- If animations are janky, ensure the device meets performance requirements
- For content not displaying properly, verify Markdown formatting follows Docusaurus standards