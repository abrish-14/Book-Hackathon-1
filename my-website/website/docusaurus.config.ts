import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'AI/Spec-Driven Development Book',
  tagline: 'Building Intelligent Systems with Specification-Driven Approaches',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://your-ai-spec-driven-book.example.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Hackathon-1', // Usually your GitHub org/user name.
  projectName: 'Hackathon-1', // Usually your repo name.
  // GitHub Pages deployment configuration
  deploymentBranch: 'gh-pages', // Branch to deploy to

  onBrokenLinks: 'throw',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
          // Performance optimizations
          routeBasePath: '/docs',
          showLastUpdateTime: true,
          showLastUpdateAuthor: true,
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
        // Additional performance optimizations
        sitemap: {
          changefreq: 'weekly',
          priority: 0.5,
          ignorePatterns: ['/tags/**'],
          filename: 'sitemap.xml',
        },
        gtag: {
          trackingID: process.env.GA_TRACKING_ID || 'GA-TRACKING-ID', // Replace with actual Google Analytics tracking ID
          anonymizeIP: true, // Optional: anonymize IP for privacy
        },
      },
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    // RAG Chatbot Configuration
    ragChatbot: {
      enabled: true,
      endpoint: '/api/rag', // API endpoint for RAG queries
      maxContextLength: 4000, // Maximum tokens for context
      similarityThreshold: 0.7, // Minimum similarity for relevant results
      maxResults: 5, // Maximum number of results to return
      placeholderText: 'Ask a question about the documentation...',
      welcomeMessage: 'Hello! I\'m your AI documentation assistant. Ask me anything about the AI/Spec-Driven Book content.',
    },
    navbar: {
      title: 'AI/Spec-Driven Book',
      logo: {
        alt: 'AI/Spec-Driven Book Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Documentation',
        },
        {to: '/docs/contributing', label: 'Contributing', position: 'left'},
        {
          to: '/docs/rag-chatbot-roadmap',
          label: 'RAG Roadmap',
          position: 'left',
        },
        {
          href: 'https://github.com/Hackathon-1/Hackathon-1',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [
            {
              label: 'Introduction',
              to: '/docs/intro',
            },
            {
              label: 'Module 1',
              to: '/docs/module-1/chapter-1',
            },
            {
              label: 'Contributing',
              to: '/docs/contributing',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'RAG Chatbot Roadmap',
              to: '/docs/rag-chatbot-roadmap',
            },
            {
              label: 'Content Embedding Schema',
              to: '/docs/embedding-schema',
            },
            {
              label: 'GitHub',
              href: 'https://github.com/Hackathon-1/Hackathon-1',
            },
          ],
        },
        {
          title: 'AI/Spec-Driven Development',
          items: [
            {
              label: 'Specification-Driven Testing',
              href: 'https://en.wikipedia.org/wiki/Specification-driven_testing',
            },
            {
              label: 'AI Principles',
              href: 'https://ai.google/principles/',
            },
            {
              label: 'Responsible AI',
              href: 'https://www.microsoft.com/en-us/ai/responsible-ai',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} AI/Spec-Driven Development Book. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'java', 'rust', 'bash', 'json'],
    },
    algolia: {
      // The application ID provided by Algolia
      appId: process.env.ALGOLIA_APP_ID || 'YOUR_ALGOLIA_APP_ID',

      // Public API key: it is safe to commit it
      apiKey: process.env.ALGOLIA_API_KEY || 'YOUR_SEARCH_API_KEY',

      indexName: 'ai-spec-driven-book',

      // Optional: see doc section below
      contextualSearch: true,

      // Optional: Specify domains where the navigation should occur through window.location instead on history.push. Useful when our Algolia config crawls multiple documentation sites and we want to navigate with window.location.href to them.
      externalUrlRegex: 'external\\.example\\.com|thirdparty\\.example\\.com',

      // Optional: Replace parts of the item URLs from Algolia. Useful when using the same search index for multiple deployments using a different baseUrl. You can use regexp or string in the `from` param. For example: localhost:3000 vs myCompany.com/docs
      replaceSearchResultPathname: {
        from: '/docs/', // or as RegExp: /\/docs\//
        to: '/docs/',
      },

      // Optional: Algolia search parameters
      searchParameters: {},

      // Optional: path for search page that enabled by default (`false` to disable it)
      searchPagePath: 'search',
    },
    // Performance optimizations
    metadata: [
      { name: 'twitter:card', content: 'summary_large_image' },
      { name: 'twitter:site', content: '@your_handle' },
      { name: 'twitter:image:alt', content: 'AI/Spec-Driven Development Book' },
      // Performance-related metadata
      { name: 'theme-color', content: '#12affa' },
      { name: 'msapplication-TileColor', content: '#12affa' },
      // Performance and loading optimization metadata
      { name: 'viewport', content: 'width=device-width, initial-scale=1.0, viewport-fit=cover' },
      { name: 'description', content: 'AI/Spec-Driven Development Book - Performance optimized documentation site' },
      { name: 'robots', content: 'index, follow' },
    ],
  } satisfies Preset.ThemeConfig,
};

export default config;
