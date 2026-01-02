# Contributing to AI/Spec-Driven Development Book

Thank you for your interest in contributing to the AI/Spec-Driven Development Book! This guide will help you understand how to contribute content, report issues, and participate in the project.

## Table of Contents

- [Content Contribution Guidelines](#content-contribution-guidelines)
- [Writing Style](#writing-style)
- [File Structure](#file-structure)
- [Frontmatter Requirements](#frontmatter-requirements)
- [Markdown Guidelines](#markdown-guidelines)
- [Submitting Changes](#submitting-changes)
- [Review Process](#review-process)

## Content Contribution Guidelines

### Before You Start

1. **Familiarize yourself with the content**: Read existing modules to understand the style and approach
2. **Check the roadmap**: Ensure your contribution aligns with the project direction
3. **Propose major changes**: For significant additions, open an issue first to discuss

### Types of Contributions

- **New chapters**: Expanding on existing modules or creating new ones
- **Improvements**: Enhancing existing content with better explanations or examples
- **Examples**: Adding practical code examples or use cases
- **Corrections**: Fixing errors or typos in existing content
- **Translations**: Translating content to other languages (future)

## Writing Style

### General Principles

- **Be clear and concise**: Use simple, direct language
- **Be consistent**: Follow established terminology and patterns
- **Be inclusive**: Write for a diverse audience with varying experience levels
- **Be practical**: Include actionable advice and real-world examples

### Tone

- Professional but approachable
- Educational and informative
- Encourage experimentation and learning

### Technical Writing

- Define technical terms when first used
- Use consistent naming conventions
- Provide context before diving into details
- Include relevant examples

## File Structure

### Documentation Organization

```
docs/
├── intro.md                 # Introduction to the book
├── module-1/               # First major module
│   ├── chapter-1.md        # Individual chapters
│   ├── chapter-2.md
│   └── chapter-3.md
├── contributing.md         # This file
└── embedding-schema.md     # Technical documentation
```

### Adding New Content

1. Create your markdown file in the appropriate module directory
2. Add proper frontmatter (see below)
3. Update `sidebars.ts` to include your content in navigation
4. Test locally before submitting

## Frontmatter Requirements

All content files must include proper frontmatter at the top:

```markdown
---
title: Chapter Title
sidebar_label: Short Label
sidebar_position: 1
description: SEO-friendly description of the content
tags: [ai, spec-driven, development]  # Optional: relevant tags
---
```

### Frontmatter Fields

- `title`: The full title of the page (used in browser tab and SEO)
- `sidebar_label`: Short label for sidebar navigation (optional, defaults to title)
- `sidebar_position`: Number to control order in sidebar
- `description`: Brief description for SEO and social sharing
- `tags`: Array of relevant tags (optional)

## Markdown Guidelines

### Headers

Use proper header hierarchy (no skipping levels):

```markdown
# Chapter Title (h1 - automatic from title)
## Section (h2)
### Subsection (h3)
#### Sub-subsection (h4)
```

### Code Blocks

Use proper language specification:

```markdown
\```typescript
// Your code here
const example = "Hello World";
\```
```

For terminal commands:

```markdown
\```bash
npm install package-name
\```
```

### Admonitions

Use Docusaurus admonitions for special notes:

```markdown
:::note
Additional information that supplements the main content.
:::

:::tip
Helpful advice or shortcuts.
:::

:::caution
Warnings about potential issues.
:::

:::danger
Critical warnings about dangerous operations.
:::
```

### Links

Use relative links for internal navigation:

```markdown
[Link to another chapter](./other-chapter.md)
[Link to section](#section-title)
```

### Images

Store images in the `static/img/` directory:

```markdown
![Alt text](/img/image-name.png)
```

## Best Practices

### Content Structure

1. **Introduction**: Briefly explain what the section covers
2. **Main Content**: Detailed explanation with examples
3. **Summary**: Key takeaways
4. **Next Steps**: Links to related content

### Examples

- Include practical, real-world examples
- Explain the "why" behind the "what"
- Use consistent variable names and patterns
- Provide both simple and complex examples

### Accessibility

- Use descriptive alt text for images
- Use proper heading hierarchy
- Use sufficient color contrast (handled by theme)
- Write meaningful link text

## Submitting Changes

### Prerequisites

1. **Fork the repository**
2. **Clone your fork**
3. **Install dependencies**: `cd classic/website && npm install`
4. **Test locally**: `npm run start`

### Process

1. **Create a branch**: `git checkout -b feature/descriptive-name`
2. **Make changes**: Follow the guidelines above
3. **Test locally**: Verify content renders correctly
4. **Commit changes**: Use clear, descriptive commit messages
5. **Push to your fork**: `git push origin feature/descriptive-name`
6. **Open a pull request**: Provide clear description of changes

### Commit Messages

Use the conventional commit format:

```
feat: Add new chapter on advanced patterns
fix: Correct typo in chapter 2
docs: Update contributing guidelines
```

## Review Process

### What We Look For

- **Accuracy**: Technical information is correct
- **Clarity**: Content is easy to understand
- **Completeness**: Content covers the topic thoroughly
- **Consistency**: Follows established style and patterns
- **SEO**: Proper metadata and structure

### Review Timeline

- Initial review: 1-3 business days
- Feedback incorporation: Up to you
- Final approval: 1 business day after revisions

### Common Feedback

- Unclear explanations
- Missing examples
- Inconsistent terminology
- SEO improvements
- Structural suggestions

## Technical Requirements

### Local Development

```bash
# Navigate to website directory
cd classic/website

# Install dependencies
npm install

# Start development server
npm run start

# Build for production
npm run build
```

### File Formats

- Content: Markdown (.md) files
- Code examples: Include in markdown or separate files
- Images: PNG, JPG, SVG formats in `static/img/`
- Configuration: TypeScript files as needed

## Getting Help

### Resources

- [Docusaurus Documentation](https://docusaurus.io/docs)
- [Markdown Guide](https://www.markdownguide.org/)
- [Project Issues](https://github.com/your-repo/issues)

### Contact

- Open an issue for content-related questions
- Use pull requests for content suggestions
- Join our community discussions (when available)

## Recognition

Contributors will be recognized in the appropriate section of the book and in the project's contributors list.

Thank you for helping make the AI/Spec-Driven Development Book better for everyone!