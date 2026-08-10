import DefaultTheme from 'vitepress/theme'
import Layout from './Layout.vue'
import ResumePage from './ResumePage.vue'
import './custom.css'

export default {
  ...DefaultTheme,
  Layout,
  enhanceApp({ app }) { app.component('ResumePage', ResumePage) }
}
