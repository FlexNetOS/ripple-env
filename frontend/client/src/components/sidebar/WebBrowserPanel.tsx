import React, { useState, useRef } from 'react';
import { useSidebar } from '@/contexts/SidebarContext';
import * as LucideIcons from 'lucide-react';
import { cn } from '@/lib/utils';
import { motion, AnimatePresence } from 'framer-motion';

// Ripple brand colors
const COLORS = {
  primary: '#00D4FF',
  secondary: '#9B7BFF',
  accent: '#00E676',
  foreground: '#ECEDEE',
  muted: '#9BA1A6',
  border: '#2A2A2A',
  surface: '#1A1A1A',
};

// Bookmarks for quick access
const QUICK_LINKS = [
  { name: 'Google', url: 'https://google.com', icon: 'Search' },
  { name: 'GitHub', url: 'https://github.com', icon: 'Github' },
  { name: 'Stack Overflow', url: 'https://stackoverflow.com', icon: 'HelpCircle' },
  { name: 'MDN Docs', url: 'https://developer.mozilla.org', icon: 'BookOpen' },
  { name: 'NPM', url: 'https://i.ytimg.com/vi/BuIE2P-Rm7U/maxresdefault.jpg', icon: 'Package' },
  { name: 'LocalAI', url: 'http://localhost:8080', icon: 'Bot' },
  { name: 'AGiXT', url: 'http://localhost:7437', icon: 'Sparkles' },
];

// Dynamic icon component
function DynamicIcon({ name, size = 16, color = COLORS.foreground }: { 
  name: string; 
  size?: number; 
  color?: string;
}) {
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  const IconComponent = (LucideIcons as any)[name] || LucideIcons.Circle;
  return <IconComponent size={size} color={color} />;
}

export function WebBrowserPanel() {
  const { showWebBrowser, setShowWebBrowser, browserUrl, setBrowserUrl } = useSidebar();
  const [inputUrl, setInputUrl] = useState(browserUrl);
  const [isLoading, setIsLoading] = useState(false);
  const [history, setHistory] = useState<string[]>([browserUrl]);
  const [historyIndex, setHistoryIndex] = useState(0);
  const iframeRef = useRef<HTMLIFrameElement>(null);

  const handleNavigate = (url: string) => {
    let normalizedUrl = url;
    if (!url.startsWith('http://') && !url.startsWith('https://')) {
      // Check if it looks like a domain
      if (url.includes('.') && !url.includes(' ')) {
        normalizedUrl = `https://${url}`;
      } else {
        // Treat as search query
        normalizedUrl = `https://www.google.com/search?q=${encodeURIComponent(url)}`;
      }
    }
    
    setIsLoading(true);
    setBrowserUrl(normalizedUrl);
    setInputUrl(normalizedUrl);
    
    // Update history
    const newHistory = [...history.slice(0, historyIndex + 1), normalizedUrl];
    setHistory(newHistory);
    setHistoryIndex(newHistory.length - 1);
    
    // Simulate load complete
    setTimeout(() => setIsLoading(false), 1000);
  };

  const handleBack = () => {
    if (historyIndex > 0) {
      const newIndex = historyIndex - 1;
      setHistoryIndex(newIndex);
      const url = history[newIndex];
      setBrowserUrl(url);
      setInputUrl(url);
    }
  };

  const handleForward = () => {
    if (historyIndex < history.length - 1) {
      const newIndex = historyIndex + 1;
      setHistoryIndex(newIndex);
      const url = history[newIndex];
      setBrowserUrl(url);
      setInputUrl(url);
    }
  };

  const handleRefresh = () => {
    setIsLoading(true);
    if (iframeRef.current) {
      iframeRef.current.src = browserUrl;
    }
    setTimeout(() => setIsLoading(false), 1000);
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter') {
      handleNavigate(inputUrl);
    }
  };

  return (
    <AnimatePresence>
      {showWebBrowser && (
        <>
          {/* Backdrop */}
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
            className="fixed inset-0 bg-black/60 z-40"
            onClick={() => setShowWebBrowser(false)}
          />
          
          {/* Browser Panel */}
          <motion.div
            initial={{ opacity: 0, x: -20 }}
            animate={{ opacity: 1, x: 0 }}
            exit={{ opacity: 0, x: -20 }}
            transition={{ duration: 0.2 }}
            className="fixed left-20 top-4 bottom-4 w-[800px] max-w-[calc(100vw-120px)] bg-[#0A0A0A] border border-[#2A2A2A] rounded-xl z-50 flex flex-col overflow-hidden shadow-2xl"
          >
            {/* Browser Header */}
            <div className="flex items-center gap-2 p-3 border-b border-[#2A2A2A] bg-[#0F0F0F]">
              {/* Navigation Controls */}
              <div className="flex items-center gap-1">
                <button
                  onClick={handleBack}
                  disabled={historyIndex <= 0}
                  className={cn(
                    "w-8 h-8 rounded-lg flex items-center justify-center transition-colors",
                    historyIndex > 0 
                      ? "hover:bg-[rgba(255,255,255,0.05)] text-[#9BA1A6]" 
                      : "text-[#4A4A4A] cursor-not-allowed"
                  )}
                >
                  <LucideIcons.ArrowLeft size={16} />
                </button>
                <button
                  onClick={handleForward}
                  disabled={historyIndex >= history.length - 1}
                  className={cn(
                    "w-8 h-8 rounded-lg flex items-center justify-center transition-colors",
                    historyIndex < history.length - 1 
                      ? "hover:bg-[rgba(255,255,255,0.05)] text-[#9BA1A6]" 
                      : "text-[#4A4A4A] cursor-not-allowed"
                  )}
                >
                  <LucideIcons.ArrowRight size={16} />
                </button>
                <button
                  onClick={handleRefresh}
                  className="w-8 h-8 rounded-lg flex items-center justify-center hover:bg-[rgba(255,255,255,0.05)] text-[#9BA1A6] transition-colors"
                >
                  {isLoading ? (
                    <LucideIcons.Loader2 size={16} className="animate-spin" />
                  ) : (
                    <LucideIcons.RotateCw size={16} />
                  )}
                </button>
              </div>

              {/* URL Bar */}
              <div className="flex-1 flex items-center h-9 bg-[#1A1A1A] rounded-lg border border-[#2A2A2A] px-3 gap-2">
                <LucideIcons.Lock size={14} color={COLORS.accent} />
                <input
                  type="text"
                  value={inputUrl}
                  onChange={(e) => setInputUrl(e.target.value)}
                  onKeyDown={handleKeyDown}
                  className="flex-1 bg-transparent text-sm text-[#ECEDEE] placeholder-[#9BA1A6] outline-none"
                  placeholder="Enter URL or search..."
                />
                <button
                  onClick={() => handleNavigate(inputUrl)}
                  className="text-[#9BA1A6] hover:text-[#00D4FF] transition-colors"
                >
                  <LucideIcons.ArrowRight size={16} />
                </button>
              </div>

              {/* Actions */}
              <div className="flex items-center gap-1">
                <button
                  onClick={() => window.open(browserUrl, '_blank')}
                  className="w-8 h-8 rounded-lg flex items-center justify-center hover:bg-[rgba(255,255,255,0.05)] text-[#9BA1A6] transition-colors"
                  title="Open in new tab"
                >
                  <LucideIcons.ExternalLink size={16} />
                </button>
                <button
                  onClick={() => setShowWebBrowser(false)}
                  className="w-8 h-8 rounded-lg flex items-center justify-center hover:bg-[rgba(255,255,255,0.05)] text-[#9BA1A6] transition-colors"
                >
                  <LucideIcons.X size={16} />
                </button>
              </div>
            </div>

            {/* Quick Links */}
            <div className="flex items-center gap-2 px-3 py-2 border-b border-[#2A2A2A] bg-[#0F0F0F] overflow-x-auto scrollbar-hide">
              {QUICK_LINKS.map((link) => (
                <button
                  key={link.name}
                  onClick={() => handleNavigate(link.url)}
                  className={cn(
                    "flex items-center gap-1.5 px-2.5 py-1.5 rounded-md text-xs whitespace-nowrap transition-colors",
                    browserUrl.includes(link.url.replace('https://', '').replace('http://', '').split('/')[0])
                      ? "bg-[#00D4FF]/10 text-[#00D4FF]"
                      : "text-[#9BA1A6] hover:bg-[rgba(255,255,255,0.05)] hover:text-[#ECEDEE]"
                  )}
                >
                  <DynamicIcon name={link.icon} size={14} />
                  {link.name}
                </button>
              ))}
            </div>

            {/* Browser Content */}
            <div className="flex-1 relative bg-white">
              {isLoading && (
                <div className="absolute inset-0 bg-[#0A0A0A] flex items-center justify-center z-10">
                  <div className="flex flex-col items-center gap-3">
                    <LucideIcons.Loader2 size={32} className="animate-spin text-[#00D4FF]" />
                    <p className="text-sm text-[#9BA1A6]">Loading...</p>
                  </div>
                </div>
              )}
              <iframe
                ref={iframeRef}
                src={browserUrl}
                className="w-full h-full border-0"
                title="Embedded Browser"
                sandbox="allow-same-origin allow-scripts allow-popups allow-forms allow-downloads"
                onLoad={() => setIsLoading(false)}
              />
            </div>

            {/* Status Bar */}
            <div className="flex items-center justify-between px-3 py-1.5 border-t border-[#2A2A2A] bg-[#0F0F0F] text-[10px] text-[#6B7280]">
              <span>{browserUrl}</span>
              <span>Embedded Browser • Some sites may not load due to security restrictions</span>
            </div>
          </motion.div>
        </>
      )}
    </AnimatePresence>
  );
}

export default WebBrowserPanel;
