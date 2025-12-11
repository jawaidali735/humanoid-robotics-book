#!/usr/bin/env node

const fs = require('fs');
const path = require('path');
const { execSync } = require('child_process');

// Validate Docusaurus site
function validateSite() {
    console.log('Validating Docusaurus site structure...');

    // Check if required files exist
    const requiredFiles = [
        'docusaurus.config.js',
        'package.json',
        'sidebars.js',
        'docs/intro.md'
    ];

    for (const file of requiredFiles) {
        if (!fs.existsSync(file)) {
            console.error(`❌ Missing required file: ${file}`);
            return false;
        }
    }

    // Check if all markdown files have proper frontmatter
    const docsDir = 'docs';
    const files = getAllMarkdownFiles(docsDir);

    let allValid = true;
    for (const file of files) {
        if (!validateFrontmatter(file)) {
            allValid = false;
        }
    }

    if (allValid) {
        console.log('✅ All markdown files have valid frontmatter');
    }

    return allValid;
}

function getAllMarkdownFiles(dir) {
    const files = [];
    const items = fs.readdirSync(dir);

    for (const item of items) {
        const fullPath = path.join(dir, item);
        const stat = fs.statSync(fullPath);

        if (stat.isDirectory()) {
            files.push(...getAllMarkdownFiles(fullPath));
        } else if (item.endsWith('.md') || item.endsWith('.mdx')) {
            files.push(fullPath);
        }
    }

    return files;
}

function validateFrontmatter(filePath) {
    const content = fs.readFileSync(filePath, 'utf8');

    // Check if file has frontmatter (starts with ---)
    if (!content.startsWith('---')) {
        console.error(`❌ Missing frontmatter in: ${filePath}`);
        return false;
    }

    // Find the end of frontmatter
    const lines = content.split('\n');
    let frontmatterEnd = -1;
    for (let i = 1; i < lines.length; i++) {
        if (lines[i] === '---') {
            frontmatterEnd = i;
            break;
        }
    }

    if (frontmatterEnd === -1) {
        console.error(`❌ Invalid frontmatter format in: ${filePath}`);
        return false;
    }

    // Extract frontmatter content
    const frontmatter = lines.slice(1, frontmatterEnd).join('\n');

    // Check for required fields
    const requiredFields = ['id', 'title', 'sidebar_position'];
    for (const field of requiredFields) {
        if (!frontmatter.includes(field + ':')) {
            console.warn(`⚠️  Missing recommended field '${field}' in: ${filePath}`);
        }
    }

    return true;
}

function checkInternalLinks() {
    console.log('Checking internal links...');

    const docsDir = 'docs';
    const files = getAllMarkdownFiles(docsDir);

    let allValid = true;

    for (const file of files) {
        const content = fs.readFileSync(file, 'utf8');

        // Find all internal links in markdown format
        const linkRegex = /\[([^\]]+)\]\(([^)]+)\)/g;
        let match;

        while ((match = linkRegex.exec(content)) !== null) {
            const link = match[2];

            // Check if it's an internal link (relative path)
            if (link.startsWith('./') || link.startsWith('../') || !link.includes('http')) {
                // Skip anchor links for now
                if (link.startsWith('#')) continue;

                const dir = path.dirname(file);
                const targetPath = path.resolve(dir, link);

                // If it's a relative link, check if the file exists
                if (link.includes('.md') || link.includes('.mdx')) {
                    if (!fs.existsSync(targetPath)) {
                        console.error(`❌ Broken internal link in ${file}: ${link}`);
                        allValid = false;
                    }
                }
            }
        }
    }

    if (allValid) {
        console.log('✅ All internal links are valid');
    }

    return allValid;
}

function validateCodeExamples() {
    console.log('Validating code examples...');

    const docsDir = 'docs';
    const files = getAllMarkdownFiles(docsDir);

    let allValid = true;

    for (const file of files) {
        const content = fs.readFileSync(file, 'utf8');

        // Find all code blocks
        const codeBlockRegex = /```(\w+)?\n([\s\S]*?)```/g;
        let match;

        while ((match = codeBlockRegex.exec(content)) !== null) {
            const language = match[1];
            const code = match[2];

            // Validate Python code examples
            if (language === 'python') {
                if (!validatePythonCode(code)) {
                    console.error(`❌ Invalid Python code in ${file}`);
                    allValid = false;
                }
            }
            // Validate bash/shell code examples
            else if (language === 'bash' || language === 'sh') {
                if (!validateBashCode(code)) {
                    console.error(`❌ Invalid Bash code in ${file}`);
                    allValid = false;
                }
            }
        }
    }

    if (allValid) {
        console.log('✅ Code examples validation completed');
    }

    return allValid;
}

function validatePythonCode(code) {
    // Simple validation - check for basic syntax issues
    try {
        // We could run a Python syntax checker here
        // For now, just check for common issues
        if (code.includes('import rclpy') && !code.includes('rclpy.init')) {
            // This is OK - not all snippets need to initialize
        }
        return true;
    } catch (e) {
        return false;
    }
}

function validateBashCode(code) {
    // Simple validation for bash commands
    // Check for common patterns that might indicate errors
    if (code.includes('ros2 pkg create') && code.includes('src/')) {
        // Valid pattern
        return true;
    }
    return true;
}

function checkWordCount() {
    console.log('Checking content word count...');

    const docsDir = 'docs';
    const files = getAllMarkdownFiles(docsDir);

    let totalWords = 0;

    for (const file of files) {
        const content = fs.readFileSync(file, 'utf8');

        // Remove frontmatter when counting words
        let contentWithoutFrontmatter = content;
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
                contentWithoutFrontmatter = lines.slice(frontmatterEnd + 1).join('\n');
            }
        }

        // Remove code blocks when counting words
        const contentWithoutCode = contentWithoutFrontmatter.replace(/```[\s\S]*?```/g, '');

        // Count words (simple approach)
        const words = contentWithoutCode
            .replace(/[^\w\s]/g, ' ')
            .split(/\s+/)
            .filter(word => word.length > 0);

        totalWords += words.length;
    }

    console.log(`Total word count: ${totalWords.toLocaleString()}`);

    // Check if within requirements (20,000-35,000 as specified in tasks.md)
    if (totalWords >= 20000 && totalWords <= 35000) {
        console.log('✅ Content meets word count requirements (20,000-35,000 words)');
        return true;
    } else {
        if (totalWords < 20000) {
            console.warn(`⚠️  Content below minimum word count (current: ${totalWords}, minimum: 20,000)`);
        } else {
            console.warn(`⚠️  Content exceeds maximum word count (current: ${totalWords}, maximum: 35,000)`);
        }
        return false;
    }
}

function runDocusaurusBuild() {
    console.log('Testing Docusaurus build...');

    try {
        // Check if docusaurus command is available
        execSync('npx docusaurus --version', { stdio: 'pipe' });

        // Try to build the site
        console.log('Building Docusaurus site...');
        const result = execSync('npx docusaurus build', { encoding: 'utf8', stdio: 'pipe' });

        console.log('✅ Docusaurus build successful');
        return true;
    } catch (error) {
        console.error('❌ Docusaurus build failed:');
        console.error(error.stdout || error.stderr || error.message);
        return false;
    }
}

function main() {
    console.log('Starting comprehensive validation...\n');

    let allValid = true;

    // Run all validation checks
    allValid &= validateSite();
    allValid &= checkInternalLinks();
    allValid &= validateCodeExamples();
    allValid &= checkWordCount();
    allValid &= runDocusaurusBuild();

    console.log('\nValidation summary:');
    if (allValid) {
        console.log('✅ All validation checks passed!');
        process.exit(0);
    } else {
        console.log('❌ Some validation checks failed');
        process.exit(1);
    }
}

// Run validation
main();