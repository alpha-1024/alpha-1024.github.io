<script setup lang="ts">
import { computed, onMounted, onUnmounted, ref } from 'vue'

type Language = 'zh-CN' | 'zh-TW' | 'en'
const language = ref<Language>('zh-CN')
const copy = {
  'zh-CN': { role: '嵌入式与 Linux 工程实践方向', intro: '长期记录树莓派、Linux、嵌入式开发与工程工具的实践经验，关注系统配置、硬件调试、机器人软件和可复用的工程方法。', profile: '技术博客作者', profileSub: '嵌入式 / Linux / 机器人', contact: '联系方式', skills: '专业技能', education: '教育背景', projects: '项目经历', ideas: '技术理念', stats: ['技术方向', '记录与维护', '写作方式', '开源协作'] },
  'zh-TW': { role: '嵌入式與 Linux 工程實踐方向', intro: '長期記錄樹莓派、Linux、嵌入式開發與工程工具的實踐經驗，關注系統配置、硬件調試、機器人軟件和可復用的工程方法。', profile: '技術博客作者', profileSub: '嵌入式 / Linux / 機器人', contact: '聯繫方式', skills: '專業技能', education: '教育背景', projects: '項目經歷', ideas: '技術理念', stats: ['技術方向', '記錄與維護', '寫作方式', '開源協作'] },
  en: { role: 'Embedded & Linux Engineering', intro: 'A practical notebook covering Raspberry Pi, Linux, embedded development, hardware debugging, and reusable engineering methods.', profile: 'Technical Blogger', profileSub: 'Embedded / Linux / Robotics', contact: 'Contact', skills: 'Skills', education: 'Education', projects: 'Projects', ideas: 'Principles', stats: ['Focus areas', 'Maintained', 'Writing', 'Open source'] }
} as const
const text = computed(() => copy[language.value])
function setLanguage(next: Language) { language.value = next; document.documentElement.lang = next; localStorage.setItem('resume-language', next) }

onMounted(() => document.documentElement.classList.add('resume-layout-active'))
onMounted(() => { const saved = localStorage.getItem('resume-language') as Language | null; if (saved && saved in copy) language.value = saved })
onUnmounted(() => document.documentElement.classList.remove('resume-layout-active'))
</script>

<template>
  <div class="resume-page">
    <section class="resume-hero">
      <div class="resume-hero-copy">
        <div class="resume-langs"><button :class="{ active: language === 'zh-CN' }" @click="setLanguage('zh-CN')">简体中文</button><button :class="{ active: language === 'zh-TW' }" @click="setLanguage('zh-TW')">繁體中文</button><button :class="{ active: language === 'en' }" @click="setLanguage('en')">English</button></div>
        <p class="resume-kicker">PERSONAL RESUME</p>
        <h1>alpha-1024</h1>
        <h2>{{ text.role }}</h2>
        <p class="resume-accent">Embedded · Linux · Robotics</p>
        <p class="resume-intro">{{ text.intro }}</p>
      </div>
      <div class="resume-profile"><img src="/img/avatar.png" alt="alpha-1024" /><strong>{{ text.profile }}</strong><span>{{ text.profileSub }}</span></div>
    </section>

    <section class="resume-stats" aria-label="个人概览"><div><strong>3+</strong><span>{{ text.stats[0] }}</span></div><div><strong>持续</strong><span>{{ text.stats[1] }}</span></div><div><strong>Markdown</strong><span>{{ text.stats[2] }}</span></div><div><strong>GitHub</strong><span>{{ text.stats[3] }}</span></div></section>

    <div class="resume-grid">
      <aside class="resume-sidebar">
        <section class="resume-card"><h3>联系方式</h3><dl><dt>GitHub</dt><dd><a href="https://github.com/alpha-1024">alpha-1024</a></dd><dt>Email</dt><dd>2051059438@qq.com</dd><dt>所在地</dt><dd>中国 · 山东</dd></dl></section>
        <section class="resume-card"><h3>专业技能</h3><h4>嵌入式与硬件</h4><div class="skill-tags"><span>C / C++</span><span>STM32</span><span>FreeRTOS</span><span>串口调试</span><span>GPIO</span></div><h4>Linux 与工具</h4><div class="skill-tags"><span>Linux</span><span>Shell</span><span>Git</span><span>Docker</span><span>VitePress</span></div><h4>机器人方向</h4><div class="skill-tags"><span>ROS2</span><span>SLAM</span><span>导航</span><span>传感器</span></div></section>
      </aside>

      <main class="resume-main">
        <section class="resume-card resume-section"><h3>教育背景</h3><div class="resume-entry"><time>2023.9 - 2027.6</time><div><strong>齐鲁师范学院</strong><p><strong>信息科学与工程学院 · 计算机科学与技术专业</strong></p><p>围绕 Linux、树莓派、STM32 与 ROS2 建立从硬件到应用层的完整实践。</p></div></div><div class="resume-entry"><time>现在</time><div><strong>个人技术博客</strong><p>使用 Markdown 编写，通过 GitHub 管理内容，部署在 GitHub Pages。</p></div></div></section>
        <section class="resume-card resume-section"><h3>项目经历</h3><div class="resume-entry"><time>2025</time><div><strong>树莓派 Linux 配置与维护</strong><p>整理串口登录、GPIO 检查、磁盘分区扩展等常见问题，形成可执行的配置教程。</p></div></div><div class="resume-entry"><time>持续</time><div><strong>技术博客重构</strong><p>将旧 Hexo 静态导出迁移到 VitePress，使用 GitHub Actions 自动构建和发布。</p></div></div></section>
        <section class="resume-card resume-section"><h3>技术理念</h3><ul class="resume-list"><li>先复现问题，再记录最小可行的解决步骤。</li><li>让命令、配置和验证方式都可以被读者直接复用。</li><li>保持内容简洁，持续修订，而不是一次性写完。</li></ul></section>
      </main>
    </div>
  </div>
</template>
