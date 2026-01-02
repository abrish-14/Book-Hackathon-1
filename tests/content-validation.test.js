const fs = require('fs');
const path = require('path');

// Test suite for validating documentation content
describe('Documentation Content Validation', () => {
  const docsDir = path.join(__dirname, '../classic/website/docs');

  // Test that all markdown files have proper frontmatter
  test('All markdown files should have valid frontmatter', () => {
    const markdownFiles = getAllMarkdownFiles(docsDir);

    markdownFiles.forEach(file => {
      const content = fs.readFileSync(file, 'utf8');
      const hasFrontmatter = content.startsWith('---');

      expect(hasFrontmatter).toBe(true);
    });
  });

  // Test that all content files have titles in frontmatter
  test('All content files should have titles in frontmatter', () => {
    const markdownFiles = getAllMarkdownFiles(docsDir);

    markdownFiles.forEach(file => {
      const content = fs.readFileSync(file, 'utf8');
      const frontmatter = extractFrontmatter(content);

      expect(frontmatter).toHaveProperty('title');
      expect(frontmatter.title).toBeTruthy();
    });
  });

  // Test that all content files have descriptions in frontmatter
  test('All content files should have descriptions in frontmatter', () => {
    const markdownFiles = getAllMarkdownFiles(docsDir);

    markdownFiles.forEach(file => {
      const content = fs.readFileSync(file, 'utf8');
      const frontmatter = extractFrontmatter(content);

      expect(frontmatter).toHaveProperty('description');
      expect(frontmatter.description).toBeTruthy();
    });
  });

  // Test that all internal links are valid
  test('Internal links should be valid', () => {
    const markdownFiles = getAllMarkdownFiles(docsDir);

    markdownFiles.forEach(file => {
      const content = fs.readFileSync(file, 'utf8');
      const links = extractInternalLinks(content);

      links.forEach(link => {
        const targetFile = path.resolve(path.dirname(file), link);
        expect(fs.existsSync(targetFile)).toBe(true);
      });
    });
  });

  // Test that there are no broken image references
  test('Image references should be valid', () => {
    const markdownFiles = getAllMarkdownFiles(docsDir);
    const staticImgDir = path.join(__dirname, '../classic/website/static/img');

    markdownFiles.forEach(file => {
      const content = fs.readFileSync(file, 'utf8');
      const imageRefs = extractImageReferences(content);

      imageRefs.forEach(imgRef => {
        if (imgRef.startsWith('/img/')) {
          const imagePath = path.join(staticImgDir, imgRef.substring(5)); // Remove '/img/' prefix
          expect(fs.existsSync(imagePath)).toBe(true);
        }
      });
    });
  });
});

// Helper function to get all markdown files in a directory recursively
function getAllMarkdownFiles(dir) {
  const files = [];
  const items = fs.readdirSync(dir);

  items.forEach(item => {
    const fullPath = path.join(dir, item);
    const stat = fs.statSync(fullPath);

    if (stat.isDirectory()) {
      files.push(...getAllMarkdownFiles(fullPath));
    } else if (item.endsWith('.md')) {
      files.push(fullPath);
    }
  });

  return files;
}

// Helper function to extract frontmatter from markdown content
function extractFrontmatter(content) {
  if (!content.startsWith('---')) {
    return {};
  }

  const frontmatterMatch = content.match(/---\n([\s\S]*?)\n---/);
  if (!frontmatterMatch) {
    return {};
  }

  const frontmatterContent = frontmatterMatch[1];
  const frontmatter = {};

  frontmatterContent.split('\n').forEach(line => {
    const colonIndex = line.indexOf(':');
    if (colonIndex > 0) {
      const key = line.substring(0, colonIndex).trim();
      const value = line.substring(colonIndex + 1).trim();

      // Remove quotes if present
      const cleanValue = value.replace(/^["']|["']$/g, '');
      frontmatter[key] = cleanValue;
    }
  });

  return frontmatter;
}

// Helper function to extract internal links from markdown content
function extractInternalLinks(content) {
  // Match [text](./path.md) or [text](../path.md) or [text](/path.md)
  const linkRegex = /\[([^\]]+)\]\(([^)]+)\)/g;
  const links = [];
  let match;

  while ((match = linkRegex.exec(content)) !== null) {
    const link = match[2];
    // Only consider relative links (not http/https)
    if (link.startsWith('./') || link.startsWith('../') || link.startsWith('/')) {
      links.push(link);
    }
  }

  return links;
}

// Helper function to extract image references from markdown content
function extractImageReferences(content) {
  // Match ![alt text](/img/path.png) or ![alt text](img/path.png)
  const imgRegex = /!\[[^\]]*\]\(([^)]+)\)/g;
  const images = [];
  let match;

  while ((match = imgRegex.exec(content)) !== null) {
    const imgPath = match[1];
    images.push(imgPath);
  }

  return images;
}

// Additional test for ensuring proper heading structure
test('Content should have proper heading hierarchy', () => {
  const markdownFiles = getAllMarkdownFiles(docsDir);

  markdownFiles.forEach(file => {
    const content = fs.readFileSync(file, 'utf8');
    const lines = content.split('\n');
    let lastHeadingLevel = 0;
    let inFrontmatter = false;

    for (const line of lines) {
      if (line.trim() === '---' && !inFrontmatter) {
        inFrontmatter = true;
        continue;
      }
      if (line.trim() === '---' && inFrontmatter) {
        inFrontmatter = false;
        continue;
      }

      if (inFrontmatter) continue;

      const headingMatch = line.match(/^(#{1,6})\s+(.+)$/);
      if (headingMatch) {
        const currentLevel = headingMatch[1].length;

        // Allow only one level jump at a time (e.g., h1 -> h2 is OK, h1 -> h3 is not)
        if (lastHeadingLevel > 0) {
          expect(currentLevel).toBeLessThanOrEqual(lastHeadingLevel + 1);
        }

        lastHeadingLevel = currentLevel;
      }
    }
  });
});