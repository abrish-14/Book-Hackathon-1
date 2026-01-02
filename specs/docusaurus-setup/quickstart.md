# Quickstart Guide: Docusaurus Documentation Setup

## Prerequisites

- Node.js (LTS version recommended)
- npm or yarn package manager
- Git for version control

## Installation Steps

1. **Initialize a new Docusaurus project:**
   ```bash
   npx create-docusaurus@latest website classic
   ```

2. **Navigate to your project directory:**
   ```bash
   cd website
   ```

3. **Start the development server:**
   ```bash
   npm run start
   ```
   This will start a local development server at http://localhost:3000

## Project Structure

After initialization, your project will have the following structure:

```
website/
├── blog/
├── docs/
├── src/
│   ├── components/
│   ├── css/
│   └── pages/
├── static/
├── docusaurus.config.js
├── package.json
├── README.md
├── sidebars.js
└── yarn.lock (or package-lock.json)
```

## Creating Module 1 with 3 Chapters

1. **Create the module directory:**
   ```bash
   mkdir docs/module-1
   ```

2. **Create Chapter 1:**
   ```bash
   touch docs/module-1/chapter-1.md
   ```

3. **Create Chapter 2:**
   ```bash
   touch docs/module-1/chapter-2.md
   ```

4. **Create Chapter 3:**
   ```bash
   touch docs/module-1/chapter-3.md
   ```

## Basic Document Structure

Each chapter should follow this format:

```markdown
---
title: Chapter Title
sidebar_label: Chapter X
sidebar_position: X
description: Brief description of the chapter
---

# Chapter Title

Your content here...
```

## Configuring Navigation

1. **Update `sidebars.js`** to include your new modules and chapters:
   ```javascript
   module.exports = {
     tutorialSidebar: [
       'intro',
       {
         type: 'category',
         label: 'Module 1',
         items: ['module-1/chapter-1', 'module-1/chapter-2', 'module-1/chapter-3'],
       },
     ],
   };
   ```

2. **Add documents to `docusaurus.config.js`** in the presets section if needed.

## Building for Production

To build your site for production:

```bash
npm run build
```

The static files will be generated in the `build/` folder.

## Deployment

For GitHub Pages deployment, you can use:

```bash
GIT_USER=<your-github-username> npm run deploy
```

Make sure to configure the `deploymentBranch` in `docusaurus.config.js` to be `gh-pages`.