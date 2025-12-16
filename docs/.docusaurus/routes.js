import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/blog',
    component: ComponentCreator('/blog', 'b2f'),
    exact: true
  },
  {
    path: '/blog/archive',
    component: ComponentCreator('/blog/archive', '182'),
    exact: true
  },
  {
    path: '/blog/authors',
    component: ComponentCreator('/blog/authors', '0b7'),
    exact: true
  },
  {
    path: '/blog/authors/all-sebastien-lorber-articles',
    component: ComponentCreator('/blog/authors/all-sebastien-lorber-articles', '4a1'),
    exact: true
  },
  {
    path: '/blog/authors/yangshun',
    component: ComponentCreator('/blog/authors/yangshun', 'a68'),
    exact: true
  },
  {
    path: '/blog/first-blog-post',
    component: ComponentCreator('/blog/first-blog-post', '89a'),
    exact: true
  },
  {
    path: '/blog/long-blog-post',
    component: ComponentCreator('/blog/long-blog-post', '9ad'),
    exact: true
  },
  {
    path: '/blog/mdx-blog-post',
    component: ComponentCreator('/blog/mdx-blog-post', 'e9f'),
    exact: true
  },
  {
    path: '/blog/tags',
    component: ComponentCreator('/blog/tags', '287'),
    exact: true
  },
  {
    path: '/blog/tags/docusaurus',
    component: ComponentCreator('/blog/tags/docusaurus', '704'),
    exact: true
  },
  {
    path: '/blog/tags/facebook',
    component: ComponentCreator('/blog/tags/facebook', '858'),
    exact: true
  },
  {
    path: '/blog/tags/hello',
    component: ComponentCreator('/blog/tags/hello', '299'),
    exact: true
  },
  {
    path: '/blog/tags/hola',
    component: ComponentCreator('/blog/tags/hola', '00d'),
    exact: true
  },
  {
    path: '/blog/welcome',
    component: ComponentCreator('/blog/welcome', 'd2b'),
    exact: true
  },
  {
    path: '/chat',
    component: ComponentCreator('/chat', 'aac'),
    exact: true
  },
  {
    path: '/markdown-page',
    component: ComponentCreator('/markdown-page', '3d7'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'fca'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '04a'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', 'a9b'),
            routes: [
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', '61d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module-1-introduction/lesson-1-overview',
                component: ComponentCreator('/docs/modules/module-1-introduction/lesson-1-overview', '98c'),
                exact: true
              },
              {
                path: '/docs/modules/module-1-introduction/lesson-2-objectives',
                component: ComponentCreator('/docs/modules/module-1-introduction/lesson-2-objectives', 'e52'),
                exact: true
              },
              {
                path: '/docs/modules/module-1-introduction/lesson-3-key-concepts',
                component: ComponentCreator('/docs/modules/module-1-introduction/lesson-3-key-concepts', '3a3'),
                exact: true
              },
              {
                path: '/docs/modules/module-1-ros2/lesson-1-introduction-to-ros2',
                component: ComponentCreator('/docs/modules/module-1-ros2/lesson-1-introduction-to-ros2', '0c2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module-1-ros2/lesson-2-nodes-topics-services',
                component: ComponentCreator('/docs/modules/module-1-ros2/lesson-2-nodes-topics-services', 'b65'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module-1-ros2/lesson-3-bridging-and-urdf',
                component: ComponentCreator('/docs/modules/module-1-ros2/lesson-3-bridging-and-urdf', 'a9b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module-2-basics/lesson-1-foundations',
                component: ComponentCreator('/docs/modules/module-2-basics/lesson-1-foundations', 'b5c'),
                exact: true
              },
              {
                path: '/docs/modules/module-2-basics/lesson-2-practice',
                component: ComponentCreator('/docs/modules/module-2-basics/lesson-2-practice', 'db3'),
                exact: true
              },
              {
                path: '/docs/modules/module-2-basics/lesson-3-quiz',
                component: ComponentCreator('/docs/modules/module-2-basics/lesson-3-quiz', 'f2c'),
                exact: true
              },
              {
                path: '/docs/modules/module-2-digital-twin/lesson-1-gazebo-simulation',
                component: ComponentCreator('/docs/modules/module-2-digital-twin/lesson-1-gazebo-simulation', 'ca7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module-2-digital-twin/lesson-2-sensor-simulation',
                component: ComponentCreator('/docs/modules/module-2-digital-twin/lesson-2-sensor-simulation', '0cf'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module-2-digital-twin/lesson-3-unity-integration',
                component: ComponentCreator('/docs/modules/module-2-digital-twin/lesson-3-unity-integration', '7f5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module-3-advanced/lesson-1-advanced-topics',
                component: ComponentCreator('/docs/modules/module-3-advanced/lesson-1-advanced-topics', '9fe'),
                exact: true
              },
              {
                path: '/docs/modules/module-3-advanced/lesson-2-case-studies',
                component: ComponentCreator('/docs/modules/module-3-advanced/lesson-2-case-studies', 'fdd'),
                exact: true
              },
              {
                path: '/docs/modules/module-3-advanced/lesson-3-summary',
                component: ComponentCreator('/docs/modules/module-3-advanced/lesson-3-summary', 'd19'),
                exact: true
              },
              {
                path: '/docs/modules/module-3-ai-brain/lesson-1-introduction-to-isaac',
                component: ComponentCreator('/docs/modules/module-3-ai-brain/lesson-1-introduction-to-isaac', '564'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module-3-ai-brain/lesson-2-isaac-sim-synthetic-data',
                component: ComponentCreator('/docs/modules/module-3-ai-brain/lesson-2-isaac-sim-synthetic-data', '9a2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module-3-ai-brain/lesson-3-isaac-ros-nav2',
                component: ComponentCreator('/docs/modules/module-3-ai-brain/lesson-3-isaac-ros-nav2', '012'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module-4-vla/lesson-1-introduction-to-vla',
                component: ComponentCreator('/docs/modules/module-4-vla/lesson-1-introduction-to-vla', '894'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module-4-vla/lesson-2-voice-to-action',
                component: ComponentCreator('/docs/modules/module-4-vla/lesson-2-voice-to-action', '885'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module-4-vla/lesson-3-cognitive-planning',
                component: ComponentCreator('/docs/modules/module-4-vla/lesson-3-cognitive-planning', 'eff'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module-5-rag-chatbot/lesson-1-introduction',
                component: ComponentCreator('/docs/modules/module-5-rag-chatbot/lesson-1-introduction', '922'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module-5-rag-chatbot/lesson-2-authentication',
                component: ComponentCreator('/docs/modules/module-5-rag-chatbot/lesson-2-authentication', 'e36'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module-5-rag-chatbot/lesson-3-vector-database',
                component: ComponentCreator('/docs/modules/module-5-rag-chatbot/lesson-3-vector-database', '424'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module-5-rag-chatbot/lesson-4-rag-backend',
                component: ComponentCreator('/docs/modules/module-5-rag-chatbot/lesson-4-rag-backend', 'ac4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module-5-rag-chatbot/lesson-5-frontend-ui',
                component: ComponentCreator('/docs/modules/module-5-rag-chatbot/lesson-5-frontend-ui', '05c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module-5-rag-chatbot/lesson-6-integration-deployment',
                component: ComponentCreator('/docs/modules/module-5-rag-chatbot/lesson-6-integration-deployment', '7ce'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module-5-rag-chatbot/lesson-7-physical-ai-assistant',
                component: ComponentCreator('/docs/modules/module-5-rag-chatbot/lesson-7-physical-ai-assistant', '948'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/tutorial-basics/congratulations',
                component: ComponentCreator('/docs/tutorial-basics/congratulations', '458'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/tutorial-basics/create-a-blog-post',
                component: ComponentCreator('/docs/tutorial-basics/create-a-blog-post', '108'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/tutorial-basics/create-a-document',
                component: ComponentCreator('/docs/tutorial-basics/create-a-document', '8fc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/tutorial-basics/create-a-page',
                component: ComponentCreator('/docs/tutorial-basics/create-a-page', '951'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/tutorial-basics/deploy-your-site',
                component: ComponentCreator('/docs/tutorial-basics/deploy-your-site', '4f5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/tutorial-basics/markdown-features',
                component: ComponentCreator('/docs/tutorial-basics/markdown-features', 'b05'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/tutorial-extras/manage-docs-versions',
                component: ComponentCreator('/docs/tutorial-extras/manage-docs-versions', '978'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/tutorial-extras/translate-your-site',
                component: ComponentCreator('/docs/tutorial-extras/translate-your-site', 'f9a'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
