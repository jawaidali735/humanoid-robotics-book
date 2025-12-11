# Data Model: Book on Physical AI and Humanoid Robotics

## Overview
This document defines the key data entities and their relationships for the educational content management system of the Physical AI and Humanoid Robotics book.

## Core Entities

### Book Chapter
- **Description**: A self-contained unit of educational content following the mandatory template structure
- **Fields**:
  - `id`: Unique identifier (string)
  - `title`: Chapter title (string)
  - `module`: Associated module (enum: ROS2, Simulation, AI-Integration, VLA)
  - `learning_outcomes`: List of specific learning objectives (array of strings)
  - `content_sections`: Ordered list of content sections (array of objects)
  - `code_examples`: List of executable code examples (array of objects)
  - `exercises`: List of practice tasks (array of objects)
  - `prerequisites`: Prerequisites for this chapter (array of strings)
  - `created_date`: Creation timestamp (datetime)
  - `last_updated`: Last modification timestamp (datetime)

### Module Content
- **Description**: One of the four core modules with interconnected concepts and examples
- **Fields**:
  - `id`: Unique identifier (string)
  - `name`: Module name (enum: ROS2, Simulation, AI-Integration, VLA)
  - `description`: Brief description of the module (string)
  - `chapters`: List of chapters in this module (array of chapter references)
  - `prerequisites`: Prerequisites for this module (array of strings)
  - `learning_path`: Sequential learning path (array of chapter IDs)
  - `capstone_integration`: Capstone project components related to this module (array of strings)

### Hands-on Exercise
- **Description**: A practical implementation task that students can complete to reinforce theoretical concepts
- **Fields**:
  - `id`: Unique identifier (string)
  - `title`: Exercise title (string)
  - `description`: Detailed exercise description (string)
  - `difficulty`: Difficulty level (enum: beginner, intermediate, advanced)
  - `estimated_time`: Estimated completion time in minutes (integer)
  - `prerequisites`: Prerequisites for completing the exercise (array of strings)
  - `steps`: Sequential steps to complete the exercise (array of strings)
  - `expected_outcomes`: Expected results after completion (array of strings)
  - `validation_criteria`: Criteria to verify completion (array of strings)
  - `associated_chapter`: Chapter this exercise belongs to (chapter reference)

### Code Example
- **Description**: A reproducible code snippet with setup instructions and expected outcomes
- **Fields**:
  - `id`: Unique identifier (string)
  - `title`: Example title (string)
  - `description`: Brief description of what the code does (string)
  - `language`: Programming language (enum: Python, C++, Bash, XML, URDF)
  - `code_snippet`: The actual code content (string)
  - `setup_instructions`: Instructions to set up the environment (string)
  - `execution_steps`: Steps to execute the code (array of strings)
  - `expected_output`: Expected output or behavior (string)
  - `associated_chapter`: Chapter this example belongs to (chapter reference)
  - `dependencies`: Required packages or libraries (array of strings)
  - `validation_script`: Script to validate the example works correctly (string)

### Capstone Project
- **Description**: The comprehensive integration project that combines all four modules in a voice-controlled autonomous humanoid system
- **Fields**:
  - `id`: Unique identifier (string)
  - `title`: Project title (string)
  - `description`: Detailed project description (string)
  - `components`: List of integrated components (array of strings)
  - `architecture`: System architecture description (string)
  - `implementation_steps`: Sequential implementation steps (array of strings)
  - `evaluation_criteria`: Criteria to evaluate project success (array of strings)
  - `extensions`: Possible extensions for further exploration (array of strings)
  - `associated_modules`: Modules integrated in this project (array of module references)

## Relationships

### Module Content contains Chapters
- One Module Content can contain multiple Book Chapters
- Each Book Chapter belongs to exactly one Module Content

### Chapter contains Exercises and Code Examples
- One Book Chapter can contain multiple Hands-on Exercises
- One Book Chapter can contain multiple Code Examples
- Each Exercise and Code Example belongs to exactly one Chapter

### Capstone Project integrates Modules
- The Capstone Project integrates components from all four Module Contents
- Each Module Content contributes specific components to the Capstone Project

## Validation Rules

### Book Chapter
- `title` must be unique within the same Module
- `learning_outcomes` must contain at least one outcome
- `content_sections` must follow the mandatory template structure
- `prerequisites` must reference valid previous chapters or concepts

### Module Content
- `name` must be unique across all modules
- `learning_path` must form a valid sequence without circular dependencies
- `chapters` must all belong to the same module

### Hands-on Exercise
- `difficulty` must be appropriate for the associated chapter level
- `estimated_time` must be a positive integer
- `validation_criteria` must be objectively measurable

### Code Example
- `code_snippet` must be syntactically valid
- `language` must match the content of the code
- `dependencies` must be verifiable and installable

### Capstone Project
- `components` must reference elements from all four modules
- `evaluation_criteria` must be comprehensive and measurable