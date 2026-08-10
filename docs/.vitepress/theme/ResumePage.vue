<template>
  <div class="resume-page">
    <section class="resume-hero">
      <div class="resume-hero-copy">
        <div class="resume-langs"><span class="active">简体中文</span><span>繁體中文</span><span>English</span></div>
        <p class="resume-kicker">PERSONAL RESUME</p>
        <h1>alpha-1024</h1>
        <h2>嵌入式与 Linux 工程实践方向</h2>
        <p class="resume-accent">Embedded · Linux · Robotics</p>
        <p class="resume-intro">长期记录树莓派、Linux、嵌入式开发与工程工具的实践经验，关注系统配置、硬件调试、机器人软件和可复用的工程方法。</p>
      </div>
      <div class="resume-profile"><img src="/img/avatar.png" alt="alpha-1024" /><strong>Technical Blogger</strong><span>Embedded / Linux / Robotics</span></div>
    </section>

    <section class="resume-stats" aria-label="个人概览"><div><strong>3+</strong><span>技术方向</span></div><div><strong>持续</strong><span>记录与维护</span></div><div><strong>Markdown</strong><span>写作方式</span></div><div><strong>GitHub</strong><span>开源协作</span></div></section>

    <div class="resume-grid">
      <aside class="resume-sidebar">
        <section class="resume-card"><h3>联系方式</h3><dl><dt>GitHub</dt><dd><a href="https://github.com/alpha-1024">alpha-1024</a></dd><dt>Email</dt><dd>your-email@example.com</dd><dt>所在地</dt><dd>中国 · 远程协作</dd></dl></section>
        <section class="resume-card"><h3>专业技能</h3><h4>嵌入式与硬件</h4><div class="skill-tags"><span>C / C++</span><span>STM32</span><span>FreeRTOS</span><span>串口调试</span><span>GPIO</span></div><h4>Linux 与工具</h4><div class="skill-tags"><span>Linux</span><span>Shell</span><span>Git</span><span>Docker</span><span>VitePress</span></div><h4>机器人方向</h4><div class="skill-tags"><span>ROS2</span><span>SLAM</span><span>导航</span><span>传感器</span></div></section>
      </aside>

      <main class="resume-main">
        <section class="resume-card resume-section"><h3>教育背景</h3><div class="resume-entry"><time>持续学习</time><div><strong>嵌入式与机器人系统</strong><p>围绕 Linux、树莓派、STM32 与 ROS2 建立从硬件到应用层的完整实践。</p></div></div><div class="resume-entry"><time>现在</time><div><strong>个人技术博客</strong><p>使用 Markdown 编写，通过 GitHub 管理内容，部署在 GitHub Pages。</p></div></div></section>
        <section class="resume-card resume-section"><h3>项目经历</h3><div class="resume-entry"><time>2025</time><div><strong>树莓派 Linux 配置与维护</strong><p>整理串口登录、GPIO 检查、磁盘分区扩展等常见问题，形成可执行的配置教程。</p></div></div><div class="resume-entry"><time>持续</time><div><strong>技术博客重构</strong><p>将旧 Hexo 静态导出迁移到 VitePress，使用 GitHub Actions 自动构建和发布。</p></div></div></section>
        <section class="resume-card resume-section"><h3>技术理念</h3><ul class="resume-list"><li>先复现问题，再记录最小可行的解决步骤。</li><li>让命令、配置和验证方式都可以被读者直接复用。</li><li>保持内容简洁，持续修订，而不是一次性写完。</li></ul></section>
      </main>
    </div>
  </div>
</template>
