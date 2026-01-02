# AI/Spec-Driven Development Book

Welcome to the AI/Spec-Driven Development Book project! This repository contains the documentation website for an in-depth exploration of combining artificial intelligence with specification-driven methodologies to build robust, reliable, and intelligent systems.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Project Structure](#project-structure)
- [Getting Started](#getting-started)
- [Development](#development)
- [Documentation](#documentation)
- [RAG Chatbot Integration](#rag-chatbot-integration)
- [Deployment](#deployment)
- [Contributing](#contributing)
- [License](#license)

## Overview

The AI/Spec-Driven Development Book is a comprehensive guide that explores the cutting-edge approach of combining artificial intelligence with specification-driven methodologies. This project provides both theoretical foundations and practical applications for building robust, reliable, and intelligent systems.

### What is AI/Spec-Driven Development?

AI/Spec-Driven Development represents a paradigm shift in software engineering, where we leverage the power of AI to enhance traditional specification-driven approaches. This methodology focuses on:

- **Clear Specifications**: Defining precise requirements and expected behaviors before implementation
- **AI-Powered Tools**: Using AI to assist in generating, validating, and evolving specifications
- **Automated Verification**: Ensuring implementations match specifications through automated checks
- **Intelligent Systems**: Building systems that can reason about their own behavior and adapt to changing requirements

## Features

- ✅ **Comprehensive Documentation**: Multi-module book covering fundamentals to advanced patterns
- ✅ **Interactive RAG Chatbot**: AI-powered search and Q&A for documentation
- ✅ **SEO Optimized**: Proper metadata and structure for search engines
- ✅ **Responsive Design**: Works on all devices and screen sizes
- ✅ **Automated Deployment**: GitHub Actions for continuous deployment
- ✅ **Performance Optimized**: Fast loading and efficient resource usage
- ✅ **Analytics Ready**: Google Analytics integration for insights
- ✅ **Extensible Architecture**: Foundation for future AI features

## Project Structure

```
├── .github/                    # GitHub Actions workflows
│   └── workflows/
│       └── deploy.yml          # Automated deployment workflow
├── .specify/                   # Spec-Kit Plus configuration
├── classic/                    # Docusaurus website files
│   └── website/               # Main documentation site
│       ├── docs/              # Documentation content
│       ├── src/               # Custom components and utilities
│       ├── static/            # Static assets
│       ├── package.json       # Dependencies and scripts
│       ├── docusaurus.config.ts # Site configuration
│       └── sidebars.ts        # Navigation structure
├── docs/                      # Additional documentation
│   ├── embedding-schema.md    # Schema for content embeddings
│   └── contributing.md        # Contribution guidelines
├── specs/                     # Specification files
│   └── docusaurus-setup/      # Feature specifications
├── .gitignore                 # Git ignore patterns
├── CLAUDE.md                  # Claude Code rules
└── README.md                  # This file
```

## Getting Started

### Prerequisites

- Node.js (LTS version 20.x or higher)
- npm or yarn package manager
- Git for version control

### Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. Navigate to the website directory:
   ```bash
   cd classic/website
   ```

3. Install dependencies:
   ```bash
   npm install
   ```

### Local Development

1. Start the development server:
   ```bash
   npm run start
   ```

2. Open your browser to [http://localhost:3000](http://localhost:3000)

3. Edit any file in the `docs/` directory to see live updates

## Development

### Available Scripts

In the `classic/website` directory, you can run:

- `npm run start` - Start the development server
- `npm run build` - Build the static site for production
- `npm run serve` - Serve the built site locally
- `npm run clear` - Clear the build cache
- `npm run deploy` - Deploy to GitHub Pages

### Adding Content

1. Create new markdown files in the `docs/` directory
2. Add proper frontmatter with title, description, and metadata
3. Update `sidebars.ts` to include the new content in navigation
4. Test locally before committing

### Custom Components

Custom React components are located in `src/components/`. The RAG chatbot component is at `src/components/RAGChatbot.tsx`.

## Documentation

The documentation is organized into modules:

- **Module 1**: Introduction to core concepts and principles
- **Module 2**: Advanced techniques and patterns
- **Module 3**: Real-world applications and case studies
- **Module 4**: Future directions and emerging trends

### Content Guidelines

- All content files should include proper frontmatter
- Follow the style guide in `docs/contributing.md`
- Use appropriate heading hierarchy
- Include relevant code examples and diagrams

## RAG Chatbot Integration

The site includes a foundation for an AI-powered RAG (Retrieval-Augmented Generation) chatbot:

- **Frontend Component**: `src/components/RAGChatbot.tsx`
- **Configuration**: `docusaurus.config.ts` under `ragChatbot`
- **Schema**: `docs/embedding-schema.md`
- **Indexing**: `src/utils/documentation-indexer.ts`

### Future Implementation

To fully implement the RAG functionality:

1. Set up a backend service for processing RAG queries
2. Configure vector database for documentation embeddings
3. Integrate with an LLM provider (OpenAI, Anthropic, etc.)
4. Update the API endpoint in configuration

## Deployment

### Automated Deployment (Recommended)

The site is configured for automated deployment using GitHub Actions:

1. Push changes to the `main` branch
2. GitHub Actions workflow automatically builds and deploys to GitHub Pages
3. Deployment typically completes within 1-2 minutes

### Manual Deployment

For manual deployment to GitHub Pages:

```bash
# From classic/website directory
GIT_USER=<Your GitHub username> npm run deploy
```

### Environment Configuration

Ensure the following values are configured in `docusaurus.config.ts`:

- `organizationName`: Your GitHub organization/username
- `projectName`: Your repository name
- `url`: Production URL
- `baseUrl`: Base URL for the site

## Contributing

We welcome contributions to improve the AI/Spec-Driven Development Book! Please see our [contribution guidelines](docs/contributing.md) for detailed information on how to:

- Submit content improvements
- Report issues
- Add new documentation
- Fix bugs
- Enhance existing features

### Development Workflow

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Test locally
5. Commit your changes (`git commit -m 'Add some amazing feature'`)
6. Push to the branch (`git push origin feature/amazing-feature`)
7. Open a Pull Request

## Built With

- [Docusaurus](https://docusaurus.io/) - Modern static website generator
- [React](https://reactjs.org/) - Component-based UI library
- [TypeScript](https://www.typescriptlang.org/) - Typed JavaScript superset
- [Node.js](https://nodejs.org/) - JavaScript runtime
- [GitHub Actions](https://github.com/features/actions) - Continuous integration/deployment

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Support

If you encounter any issues or have questions:

- Check the [documentation](https://your-site.github.io)
- Open an [issue](https://github.com/your-repo/issues) on GitHub
- Review the [troubleshooting guide](classic/website/README.md#troubleshooting)

## Acknowledgments

- Thanks to the Docusaurus team for the excellent documentation framework
- Inspiration from the specification-driven development community
- The AI/ML research community for advancing the field

---

Made with ❤️ and AI/Spec-Driven Development principles