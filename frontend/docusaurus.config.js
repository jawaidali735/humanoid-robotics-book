// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive guide to humanoid robotics with ROS 2, Simulation, and AI',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://jawaidali735.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/humanoid-robotics-book',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'jawaidali735', // Your GitHub org/user name.
  projectName: 'humanoid-robotics-book', // Your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

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
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          path: 'docs',
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/jawaidali735/humanoid-robotics-book/tree/main/',
        },
        blog: {
          showReadingTime: true,
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/jawaidali735/humanoid-robotics-book/tree/main/',
        },
        theme: {
          customCss: [
            './src/css/custom.css',
            './src/css/design-system.css',
            './src/css/animations.css',
            './src/css/responsive.css',
            './src/css/reading-experience.css'
          ],
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Humanoid Robotics Logo',
          src: 'img/logo.svg',
          srcDark: 'img/logo.svg',
          width: 32,
          height: 32,
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Book',
          },
          {
            to: '/',
            label: 'Chat',
            position: 'left',
          },
          {
            href: 'https://github.com/jawaidali735/humanoid-robotics-book',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'light',
        links: [
          {
            title: 'Modules',
            items: [
              {
                label: 'ROS 2 Fundamentals',
                to: '/docs/module-1-ros2/module-1-ros2-index',
              },
              {
                label: 'Simulation Environments',
                to: '/docs/module-2-simulation/module-2-simulation-index',
              },
              {
                label: 'AI Integration',
                to: '/docs/module-3-ai-integration/module-3-ai-integration-index',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/robotics',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/robotics',
              },
              {
                label: 'Twitter',
                href: 'https://twitter.com/robotics',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/jawaidali735/humanoid-robotics-book',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),

    // Pass environment variables to the client
    themes: [
      // Add any theme plugins here if needed
    ],
};

export default config;