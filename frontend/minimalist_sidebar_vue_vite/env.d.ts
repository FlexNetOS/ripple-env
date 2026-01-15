/// <reference types="vite/client" />

declare module '*.vue' {
  import type { DefineComponent } from 'vue';
  const component: DefineComponent<{}, {}, any>;
  export default component;
}

interface ImportMetaEnv {
  readonly VITE_VIBE_KANBAN_API_URL?: string;
  readonly VITE_VIBE_KANBAN_WS_URL?: string;
  readonly VITE_VIBE_KANBAN_FRONTEND_URL?: string;
  readonly VITE_VIBE_KANBAN_BACKEND_PORT?: string;
  readonly VITE_VIBE_KANBAN_FRONTEND_PORT?: string;
}

interface ImportMeta {
  readonly env: ImportMetaEnv;
}
