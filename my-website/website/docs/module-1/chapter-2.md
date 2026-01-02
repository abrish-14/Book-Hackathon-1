---
title: Setting Up Your Spec-Driven Environment
sidebar_label: Chapter 2
sidebar_position: 2
description: Configuring your development environment for spec-driven workflows
---

# Setting Up Your Spec-Driven Environment

## Introduction

Creating a proper development environment is crucial for successful spec-driven development. This chapter guides you through setting up your environment to support AI-assisted specification workflows.

## Prerequisites

Before beginning your setup, ensure you have:

- Node.js (LTS version recommended)
- npm or yarn package manager
- Git for version control
- A code editor with good Markdown support
- Access to AI development tools (e.g., Claude, ChatGPT, etc.)

## Installing Spec-Kit Plus

Spec-Kit Plus is the primary toolkit for AI/Spec-Driven Development. Follow these steps to install:

1. **Create your project directory:**
   ```bash
   mkdir my-spec-project
   cd my-spec-project
   ```

2. **Initialize a new project:**
   ```bash
   npm init -y
   ```

3. **Install Spec-Kit Plus:**
   ```bash
   npm install specify
   # or globally
   npm install -g specify
   ```

## Project Structure

A typical Spec-Kit Plus project follows this structure:

```
my-spec-project/
├── .specify/
│   ├── memory/
│   │   └── constitution.md
│   ├── templates/
│   │   ├── spec-template.md
│   │   ├── plan-template.md
│   │   └── tasks-template.md
│   └── scripts/
├── specs/
│   └── [feature-name]/
│       ├── spec.md
│       ├── plan.md
│       ├── research.md
│       ├── data-model.md
│       ├── quickstart.md
│       └── tasks.md
├── docs/
├── src/
└── tests/
```

## Configuration Files

### Constitution File

The constitution file defines your project's principles and standards. Create `.specify/memory/constitution.md`:

```markdown
# Project Constitution

## Core Principles
- All development follows a spec-first workflow
- Technical accuracy is paramount
- Reproducible setups are required
- Quality standards must be maintained
```

### Specification Template

Create a specification template that will guide your feature specifications. This ensures consistency across all specifications in your project.

## Setting Up AI Integration

### Claude Code Configuration

If using Claude Code for AI assistance:

1. **Install Claude Code CLI:**
   ```bash
   # Follow official installation instructions
   ```

2. **Configure your project:**
   ```bash
   claude-code init
   ```

### API Key Configuration

For AI tools that require API keys, create a `.env` file:

```env
OPENAI_API_KEY=your_api_key_here
ANTHROPIC_API_KEY=your_anthropic_key_here
```

## Creating Your First Specification

1. **Use the spec command:**
   ```bash
   npx specify spec "My First Feature"
   ```

2. **Edit the generated spec file** in `specs/my-first-feature/spec.md`

3. **Review and refine** the specification until it's complete and approved

## Best Practices

### Environment Consistency

- Use the same Node.js version across all development environments
- Maintain a `package-lock.json` or `yarn.lock` file
- Document all environment-specific configurations

### Specification Standards

- Keep specifications clear and concise
- Include acceptance criteria for all requirements
- Define success metrics for each feature
- Consider error cases and edge conditions

### Version Control

- Use Git for all specification and code changes
- Follow a consistent branching strategy
- Write meaningful commit messages
- Keep specifications and implementation in sync

## Troubleshooting Common Issues

### Missing Dependencies

If you encounter dependency issues:

1. Clear your package cache:
   ```bash
   npm cache clean --force
   ```

2. Reinstall dependencies:
   ```bash
   rm -rf node_modules package-lock.json
   npm install
   ```

### Specification Validation

If specifications aren't validating properly:

1. Check for missing sections
2. Verify all required fields are filled
3. Ensure proper Markdown formatting

## Conclusion

A properly configured environment is essential for successful spec-driven development. With your environment set up, you're ready to begin creating specifications and implementing features using AI assistance.