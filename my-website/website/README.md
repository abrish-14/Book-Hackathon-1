# AI/Spec-Driven Development Book - Documentation Website

This website is built using [Docusaurus](https://docusaurus.io/), a modern static website generator. It serves as the documentation hub for the AI/Spec-Driven Development Book project.

## Table of Contents
- [Project Overview](#project-overview)
- [Installation](#installation)
- [Local Development](#local-development)
- [Building the Site](#building-the-site)
- [Deployment](#deployment)
- [RAG Chatbot Integration](#rag-chatbot-integration)
- [Content Management](#content-management)

## Project Overview

This documentation site provides comprehensive coverage of AI/Spec-Driven Development principles, patterns, and practices. The site features:

- Comprehensive documentation modules
- Interactive RAG (Retrieval-Augmented Generation) chatbot for AI-powered search
- SEO-optimized content structure
- Responsive design for all devices
- Automated deployment via GitHub Actions

## Installation

```bash
# Navigate to the website directory
cd classic/website

# Install dependencies
npm install
```

## Local Development

```bash
# Start local development server
npm run start

# Or with specific port
npm run start -- --port 3001
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Building the Site

```bash
# Build static files for production
npm run build
```

This command generates static content into the `build` directory and can be served using any static content hosting service.

### Build for Testing

```bash
# Build with specific output directory
npm run build -- --out-dir build-test
```

## Deployment

### Automated Deployment (Recommended)

The site is configured for automated deployment using GitHub Actions:

1. Push changes to the `main` branch
2. GitHub Actions workflow (`.github/workflows/deploy.yml`) automatically builds and deploys to GitHub Pages
3. Deployment typically completes within 1-2 minutes

### Manual Deployment

For manual deployment to GitHub Pages:

```bash
# Deploy to GitHub Pages (requires proper configuration)
GIT_USER=<Your GitHub username> npm run deploy
```

Or using SSH:

```bash
USE_SSH=true npm run deploy
```

If you are using GitHub Pages for hosting, this command builds the website and pushes to the `gh-pages` branch.

### Environment Variables for Deployment

For production deployment, ensure the following values are configured in `docusaurus.config.ts`:

- `organizationName`: Your GitHub organization/username
- `projectName`: Your repository name
- `url`: Production URL
- `baseUrl`: Base URL for the site

## RAG Chatbot Integration

The site includes a placeholder for an AI-powered RAG (Retrieval-Augmented Generation) chatbot:

- The chatbot component is located at `src/components/RAGChatbot.tsx`
- Configuration settings are in `docusaurus.config.ts` under `ragChatbot`
- The backend API endpoint is configured at `/api/rag` (placeholder for future backend service)

### Future RAG Implementation

To fully implement the RAG functionality, you'll need to:

1. Set up a backend service for processing RAG queries
2. Configure vector database for documentation embeddings
3. Integrate with an LLM provider (OpenAI, Anthropic, etc.)
4. Update the API endpoint in configuration

## Content Management

### Adding New Content

1. Create new markdown files in the `docs/` directory
2. Add appropriate frontmatter with title, description, and metadata
3. Update `sidebars.ts` to include the new content in navigation
4. Verify the content renders correctly in development

### Content Structure

```
docs/
├── intro.md
├── module-1/
│   ├── chapter-1.md
│   ├── chapter-2.md
│   └── chapter-3.md
└── embedding-schema.md
```

### SEO Best Practices

All content files should include proper frontmatter:

```markdown
---
title: Page Title
description: SEO-friendly description of the page content
keywords: relevant, keywords, for, search
---
```

## Performance Optimization

The site includes several performance optimizations:

- Code splitting and lazy loading
- Image optimization
- Asset compression
- Caching headers configuration
- Preloading critical resources

## Troubleshooting

### Common Build Issues

**Error: "Unable to build website"**
- Check for syntax errors in markdown files
- Verify all image references are valid
- Ensure no duplicate IDs in content

**Error: "Module not found"**
- Run `npm install` to ensure all dependencies are installed
- Check import paths in components

### Local Development Issues

**Changes not reflecting**
- Restart the development server: `npm run start`
- Clear cache: `npm run clear`

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test locally
5. Submit a pull request

## License

This documentation site is part of the AI/Spec-Driven Development Book project.
