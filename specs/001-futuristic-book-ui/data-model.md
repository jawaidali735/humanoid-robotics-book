# Data Model: Futuristic Book Website UI

## Overview

This data model defines the entities and structures needed for the futuristic book website UI, focusing on the frontend components and their data requirements.

## Key Entities

### Book Content
**Description**: Represents the technical content of "Physical AI & Humanoid Robotics"
**Fields**:
- id: string (unique identifier for the book)
- title: string (Physical AI & Humanoid Robotics)
- subtitle: string (descriptive subtitle)
- description: string (brief overview of the book content)
- chapters: Chapter[] (array of book chapters)
- metadata: Metadata (additional book information)

### Chapter
**Description**: A chapter within the book
**Fields**:
- id: string (unique identifier for the chapter)
- title: string (chapter title)
- slug: string (URL-friendly identifier)
- content: string (chapter content in Markdown format)
- position: number (chapter number in sequence)
- wordCount: number (approximate word count)
- estimatedReadingTime: number (in minutes)
- sections: Section[] (array of sections within the chapter)

### Section
**Description**: A section within a chapter
**Fields**:
- id: string (unique identifier for the section)
- title: string (section title)
- level: number (heading level 1-6)
- content: string (section content)
- position: number (position within chapter)

### User Interface Elements
**Description**: Represents the visual components of the website
**Fields**:
- heroSection: HeroSection (hero section configuration)
- featureCards: FeatureCard[] (array of feature cards)
- tableOfContents: TableOfContents (navigation structure)
- floatingChatbot: ChatbotWidget (chatbot widget configuration)

### HeroSection
**Description**: Configuration for the hero section
**Fields**:
- title: string (main title)
- subtitle: string (subheading)
- description: string (value proposition)
- ctaButtons: CTAButton[] (call-to-action buttons)
- animationConfig: AnimationConfig (animation settings)

### FeatureCard
**Description**: A feature card highlighting learning outcomes
**Fields**:
- id: string (unique identifier)
- title: string (card title)
- description: string (detailed description)
- icon: string (icon identifier)
- color: string (accent color for the card)

### CTAButton
**Description**: Call-to-action button
**Fields**:
- text: string (button text)
- link: string (destination URL)
- variant: string (primary, secondary, etc.)
- size: string (small, medium, large)

### TableOfContents
**Description**: Navigation structure for book content
**Fields**:
- chapters: TOCChapter[] (array of chapters in TOC)
- isSticky: boolean (whether TOC stays fixed while scrolling)

### TOCChapter
**Description**: Chapter entry in the table of contents
**Fields**:
- id: string (chapter identifier)
- title: string (chapter title)
- slug: string (URL slug)
- sections: TOCSection[] (array of sections in this chapter)

### TOCSection
**Description**: Section entry in the table of contents
**Fields**:
- id: string (section identifier)
- title: string (section title)
- level: number (heading level)
- slug: string (URL slug)

### ChatbotWidget
**Description**: Configuration for the floating chatbot widget
**Fields**:
- isVisible: boolean (whether widget is visible)
- isExpanded: boolean (whether chat is open)
- quickActions: QuickAction[] (array of quick action buttons)
- currentMessages: ChatMessage[] (current conversation messages)

### QuickAction
**Description**: Quick action button in the chatbot
**Fields**:
- id: string (unique identifier)
- label: string (button text)
- action: string (action type: explain, summarize, navigate)
- icon: string (icon identifier)

### ChatMessage
**Description**: A message in the chat conversation
**Fields**:
- id: string (unique identifier)
- sender: string (user or assistant)
- content: string (message text)
- timestamp: Date (when message was sent)
- context: MessageContext (contextual information)

### MessageContext
**Description**: Contextual information for a chat message
**Fields**:
- selectedText: string (text selected by user)
- currentChapter: string (current chapter context)
- currentSection: string (current section context)

## Validation Rules

1. Book title must be 1-200 characters
2. Chapter titles must be unique within a book
3. Section levels must be between 1-6
4. All UI element IDs must be unique within their scope
5. URLs must be properly formatted
6. Estimated reading time must be positive
7. Quick action labels must be 1-50 characters

## State Transitions

### Chatbot Widget States
- Initial: isVisible = false, isExpanded = false
- Button Visible: isVisible = true, isExpanded = false
- Chat Open: isVisible = true, isExpanded = true
- Minimized: isVisible = true, isExpanded = false (different minimized state)

### Table of Contents States
- Visible: isSticky = true
- Active Highlighting: tracks current section being read
- Collapsed/Expanded: sections can be expanded or collapsed

## Relationships

- BookContent 1 → * Chapter (one book has many chapters)
- Chapter 1 → * Section (one chapter has many sections)
- BookContent 1 → 1 HeroSection (one book has one hero section)
- BookContent 1 → * FeatureCard (one book has many feature cards)
- BookContent 1 → 1 TableOfContents (one book has one table of contents)
- BookContent 1 → 1 ChatbotWidget (one book has one chatbot widget)
- Chapter 1 → * TOCSection (one chapter maps to many TOC sections)
- ChatbotWidget 1 → * QuickAction (one chatbot has many quick actions)
- ChatbotWidget 1 → * ChatMessage (one chatbot has many messages)