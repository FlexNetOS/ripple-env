<script setup lang="ts">
import Sidebar from '@/components/Sidebar.vue';
</script>

<template>
  <div class="flex h-screen bg-background text-foreground">
    <Sidebar />
    <main class="flex-1 overflow-auto p-6">
      <div class="max-w-4xl mx-auto">
        <h1 class="text-2xl font-bold mb-4">Ripple Environment</h1>
        <p class="text-muted-foreground">
          Select a section from the sidebar to get started.
        </p>
      </div>
    </main>
  </div>
</template>
