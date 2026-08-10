# 破晓的历程

基于 VitePress + Markdown 的个人技术博客。

## 本地开发

```bash
npm install
npm run dev
```

生产构建：

```bash
npm run build
npm run preview
```

## 部署

- **Vercel**：导入 GitHub 仓库，构建命令 `npm run build`，输出目录 `docs/.vitepress/dist`。
- **Cloudflare Pages**：连接 GitHub，构建命令 `npm run build`，输出目录 `docs/.vitepress/dist`。

旧的 Hexo 导出文件保留在 `alpha-1024.github.io/`，用于迁移时对照。
