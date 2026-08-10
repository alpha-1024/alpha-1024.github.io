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
  markdown: { lineNumbers: true }
})
