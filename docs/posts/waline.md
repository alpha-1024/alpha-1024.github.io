---
title: 配置 Waline 评论系统
description: 给 VitePress 静态博客接入 Waline 评论服务。
date: 2021-06-10
tags: [VitePress, 评论, Vercel]
---

# 配置 Waline 评论系统

Waline 是一套轻量、现代的评论系统，适合部署在 Vercel、Cloudflare Pages 等平台。它将评论服务与静态站点分离，部署博客时不需要运行后端进程。

## 部署 Waline 服务

1. 使用 GitHub 登录 [Vercel](https://vercel.com/)。
2. 从 Waline 官方仓库导入项目并完成部署。
3. 按文档配置数据库连接和 `LEAN_ID`、`LEAN_KEY` 等环境变量。
4. 记录部署完成后的服务地址，例如 `https://your-waline.vercel.app`。

生产环境请把数据库密钥只放在平台的环境变量中，不要提交到 Git 仓库。

## 在站点中接入

VitePress 默认主题不内置评论组件，推荐在 `.vitepress/theme` 中注册一个评论组件，并通过环境变量读取服务地址：

```ts
// .vitepress/theme/index.ts
import DefaultTheme from 'vitepress/theme'
import Waline from './Waline.vue'

export default {
  extends: DefaultTheme,
  enhanceApp({ app }) {
    app.component('Waline', Waline)
  }
}
```

文章页中使用：

```md
<Waline server-url="https://your-waline.vercel.app" />
```

如果暂时不需要评论，可以先不注册组件；站点的 Markdown 内容和静态部署不受影响。
