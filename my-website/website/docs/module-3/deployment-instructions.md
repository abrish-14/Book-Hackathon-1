# Deployment Instructions for AI-Robot Brain Documentation Module

This document provides instructions for deploying the AI-Robot Brain documentation module to production.

## Prerequisites

- Node.js (version 16 or higher)
- npm or yarn package manager
- Access to the documentation hosting platform
- Git access to the documentation repository

## Build Process

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-directory>
```

### 2. Install Dependencies
```bash
npm install
# or
yarn install
```

### 3. Build the Documentation Site
```bash
npm run build
# or
yarn build
```

This will create a `build` directory with the static site files.

## Configuration

### Docusaurus Configuration
The main configuration file is `docusaurus.config.js` which includes:
- Site metadata (title, tagline, URL)
- Theme configuration
- Plugin settings
- Navigation sidebar configuration

### Sidebar Configuration
The navigation structure is defined in `sidebars.js` and includes:
- Module 3: The AI-Robot Brain (NVIDIA Isaac™)
  - Isaac Sim documentation
  - Isaac ROS documentation
  - Nav2 Humanoid documentation

## Deployment Options

### Option 1: GitHub Pages
1. Configure the `deployment` section in `docusaurus.config.js`
2. Run the deployment command:
```bash
npm run deploy
```

### Option 2: Manual Deployment
1. Build the site using `npm run build`
2. Upload the contents of the `build` directory to your web server
3. Configure your web server to serve static files

### Option 3: CI/CD Pipeline
Set up a continuous integration pipeline that:
1. Checks out the latest code
2. Installs dependencies
3. Builds the documentation
4. Deploys to your hosting platform

## Post-Deployment Verification

### 1. Verify Site Accessibility
- Access the main documentation page
- Verify all navigation links work
- Check that all chapter pages load correctly

### 2. Test Cross-Chapter References
- Navigate between different chapters
- Verify all internal links work properly
- Check that search functionality indexes the new content

### 3. Validate External Links
- Verify all links to official NVIDIA documentation work
- Check that GitHub repository links are valid
- Confirm all external resource links are accessible

## Troubleshooting

### Build Issues
- Ensure all required dependencies are installed
- Check that all markdown files have proper frontmatter
- Verify all image and asset paths are correct

### Navigation Issues
- Confirm sidebar configuration includes all new pages
- Verify all internal links use correct paths
- Check that relative links work properly in the build

### Search Issues
- Ensure new content is properly indexed
- Verify search configuration includes the new module
- Test search functionality across all new pages