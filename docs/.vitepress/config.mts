import { defineConfig } from 'vitepress'

export default defineConfig({
  lang: 'zh-CN',
  title: '破晓的历程',
  description: '记录嵌入式、Linux 与工程实践的个人技术博客',
  cleanUrls: true,
  appearance: false,
  lastUpdated: true,
  themeConfig: {
    logo: '/img/avatar.png',
    siteTitle: '破晓的历程',
    nav: [
      { text: '首页', link: '/' },
      { text: '文章', link: '/posts/' },
      { text: '学习路线', link: '/roadmaps/embodied-ai' },
      { text: '具身智能教材', link: '/textbook/' },
      { text: '关于', link: '/about' },
      { text: 'GitHub', link: 'https://github.com/alpha-1024' }
    ],
    sidebar: {
      '/posts/': [
        {
          text: '技术文章',
          items: [
            { text: '树莓派配置教程', link: '/posts/raspberry-pi-setup' },
            { text: '树莓派磁盘分区问题', link: '/posts/raspberry-pi-disk' },
            { text: '配置 Waline 评论系统', link: '/posts/waline' }
          ]
        }
      ],
      '/roadmaps/': [
        {
          text: '具身智能学习路线',
          items: [
            { text: '总路线与执行计划', link: '/roadmaps/embodied-ai' }
          ]
        }
      ],
      '/textbook/': [
        {
          text: '具身智能算法教材',
          items: [
            { text: '教材首页与学习方法', link: '/textbook/' },
            { text: '第一册：数学与工程基础', link: '/textbook/volume-1-foundations' },
            { text: '第一册扩展：数学与优化', link: '/textbook/volume-1-math-deep-dive' },
            { text: '第一册扩展：工程与 ROS2', link: '/textbook/volume-1-engineering-deep-dive' },
            { text: '第一册扩展：ROS2 实战案例', link: '/textbook/volume-1-ros2-practice' },
            { text: '第一册：数学例题与考试', link: '/textbook/volume-1-math-workbook' },
            { text: '第一册：编程综合实验', link: '/textbook/volume-1-programming-workbook' },
            { text: '第一册实验与答案', link: '/textbook/volume-1-labs' },
            { text: '第二册：视觉与三维感知', link: '/textbook/volume-2-perception' },
            { text: '第三册：SLAM 与导航', link: '/textbook/volume-3-slam-navigation' },
            { text: '第三册实验与答案', link: '/textbook/volume-3-labs' },
            { text: '第四册：机械臂规划与控制', link: '/textbook/volume-4-manipulation' },
            { text: '第五册：强化学习与 VLA', link: '/textbook/volume-5-learning-vla' },
            { text: '第五册实验与答案', link: '/textbook/volume-5-labs' },
            { text: '第六册：CUDA 与部署', link: '/textbook/volume-6-deployment' },
            { text: '第六册实验与验收', link: '/textbook/volume-6-labs' },
            { text: '术语表与毕业清单', link: '/textbook/glossary-capstone' }
          ]
        }
      ]
    },
    outline: { level: [2, 3] },
    socialLinks: [{ icon: 'github', link: 'https://github.com/alpha-1024' }],
    search: { provider: 'local' },
    footer: {
      message: '用 Markdown 写作，用代码留下痕迹。',
      copyright: 'Copyright © 2026 alpha-1024'
    },
    editLink: { pattern: 'https://github.com/alpha-1024/alpha-1024.github.io/edit/main/docs/:path' }
  },
  head: [
    ['link', { rel: 'icon', href: '/img/avatar.png' }]
  ],
  markdown: { lineNumbers: true, math: true }
})
