# Validation Scripts

This directory contains scripts for validating the humanoid robotics book content and structure.

## Available Scripts

### `validate-content.js`
Validates the entire book content including:
- Site structure and required files
- Markdown frontmatter validation
- Internal link checking
- Code example validation
- Word count verification
- Docusaurus build testing

To run:
```bash
node validate-content.js
```

### `check-sidebar.js`
Validates sidebar configuration and document structure:
- Checks that all documentation files are included in the sidebar
- Validates document structure and frontmatter
- Ensures proper H1 titles

To run:
```bash
node check-sidebar.js
```

## Purpose

These scripts are designed to ensure:
1. All content follows the required format and structure
2. Internal links are valid and functional
3. Code examples are syntactically correct
4. The Docusaurus site builds successfully
5. All documentation files are properly organized in the sidebar
6. Content meets quality and completeness standards

## Integration

These scripts can be integrated into CI/CD pipelines to automatically validate content changes before deployment.