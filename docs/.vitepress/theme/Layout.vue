<script setup lang="ts">
import DefaultTheme from 'vitepress/theme'
import { ref, onMounted } from 'vue'

const collapsed = ref(false)

onMounted(() => {
  collapsed.value = localStorage.getItem('sidebar-collapsed') === 'true'
  document.documentElement.classList.toggle('sidebar-collapsed', collapsed.value)
})

function toggleSidebar() {
  collapsed.value = !collapsed.value
  document.documentElement.classList.toggle('sidebar-collapsed', collapsed.value)
  localStorage.setItem('sidebar-collapsed', String(collapsed.value))
}
</script>

<template>
  <DefaultTheme.Layout>
    <template #aside-bottom>
      <button class="sidebar-toggle" type="button" :aria-label="collapsed ? '展开侧栏' : '折叠侧栏'" @click="toggleSidebar">
        <span aria-hidden="true">{{ collapsed ? '›' : '‹' }}</span>
      </button>
    </template>
  </DefaultTheme.Layout>
</template>
