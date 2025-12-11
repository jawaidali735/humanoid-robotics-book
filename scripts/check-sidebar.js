const fs = require('fs');
const path = require('path');

// Load the sidebar configuration
const sidebarPath = path.join(__dirname, '..', 'sidebars.js');
let sidebars;

try {
    // Read and parse the sidebar file
    const sidebarContent = fs.readFileSync(sidebarPath, 'utf8');

    // Extract the sidebars object (simple approach for this specific file)
    const sidebarsMatch = sidebarContent.match(/module.exports = ({[\s\S]*?});/);
    if (sidebarsMatch) {
        // This is a simplified approach - in a real scenario you'd want a more robust parser
        // For now, let's just validate that the file exists and has the expected structure
        console.log('✅ Sidebar file exists and has expected structure');
    } else {
        console.error('❌ Could not parse sidebar configuration');
    }
} catch (error) {
    console.error('❌ Error reading sidebar configuration:', error.message);
}

// Function to find all markdown files in docs directory
function getAllDocsFiles(dir = path.join(__dirname, '..', 'docs')) {
    const files = [];
    const items = fs.readdirSync(dir);

    for (const item of items) {
        const fullPath = path.join(dir, item);
        const stat = fs.statSync(fullPath);

        if (stat.isDirectory()) {
            files.push(...getAllDocsFiles(fullPath));
        } else if (item.endsWith('.md')) {
            files.push(path.relative(path.join(__dirname, '..'), fullPath));
        }
    }

    return files;
}

// Check that all docs files are included in the sidebar
function validateSidebarCoverage() {
    console.log('\nValidating sidebar coverage...');

    const allDocsFiles = getAllDocsFiles();
    console.log(`Found ${allDocsFiles.length} documentation files`);

    // For this validation, we'll just verify that the files exist
    // A more comprehensive check would parse the actual sidebar structure
    const sidebarContent = fs.readFileSync(sidebarPath, 'utf8');

    let coveredCount = 0;
    for (const file of allDocsFiles) {
        // Check if the file path appears in the sidebar configuration
        if (sidebarContent.includes(file.replace('docs/', '').replace('.md', ''))) {
            coveredCount++;
        }
    }

    console.log(`Sidebar covers ${coveredCount} out of ${allDocsFiles.length} files`);

    if (coveredCount === allDocsFiles.length) {
        console.log('✅ All documentation files are included in the sidebar');
        return true;
    } else {
        console.log('⚠️  Some documentation files may not be included in the sidebar');
        // List uncovered files
        const uncovered = allDocsFiles.filter(file =>
            !sidebarContent.includes(file.replace('docs/', '').replace('.md', ''))
        );
        console.log('Uncovered files:', uncovered);
        return false;
    }
}

// Validate document structure and content
function validateDocumentStructure() {
    console.log('\nValidating document structure...');

    const allDocsFiles = getAllDocsFiles();
    let allValid = true;

    for (const filePath of allDocsFiles) {
        const fullPath = path.join(__dirname, '..', filePath);
        const content = fs.readFileSync(fullPath, 'utf8');

        // Check for proper frontmatter
        if (!content.startsWith('---')) {
            console.error(`❌ Missing frontmatter in: ${filePath}`);
            allValid = false;
            continue;
        }

        // Extract frontmatter
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
            allValid = false;
            continue;
        }

        const frontmatter = lines.slice(1, frontmatterEnd).join('\n');

        // Check for required fields
        const requiredFields = ['id', 'title', 'sidebar_position'];
        for (const field of requiredFields) {
            if (!frontmatter.includes(`${field}:`)) {
                console.warn(`⚠️  Missing field '${field}' in: ${filePath}`);
            }
        }

        // Check document structure (should have H1 title)
        const contentWithoutFrontmatter = lines.slice(frontmatterEnd + 1).join('\n');
        if (!contentWithoutFrontmatter.trim().startsWith('# ')) {
            console.warn(`⚠️  Missing H1 title in: ${filePath}`);
        }
    }

    if (allValid) {
        console.log('✅ Document structure validation completed');
    }

    return allValid;
}

// Run validations
validateSidebarCoverage();
validateDocumentStructure();

console.log('\nSidebar validation completed.');