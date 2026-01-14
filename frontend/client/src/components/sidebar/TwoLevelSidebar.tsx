import React from 'react';
import { useAuth } from '@/_core/hooks/useAuth';
import { getLoginUrl } from '@/const';
import { Button } from '@/components/ui/button';
import { IconNavigation } from './IconNavigation';
import { DetailSidebar } from './DetailSidebar';
import { Loader2 } from 'lucide-react';

interface TwoLevelSidebarProps {
  children?: React.ReactNode;
}

// Loading skeleton for the sidebar layout
function SidebarLayoutSkeleton() {
  return (
    <div className="flex h-screen w-full bg-[#0A0A0A] items-center justify-center">
      <div className="flex flex-col items-center gap-4">
        <Loader2 className="h-8 w-8 animate-spin text-[#00D4FF]" />
        <p className="text-sm text-[#9BA1A6]">Loading...</p>
      </div>
    </div>
  );
}

// Sign in prompt for unauthenticated users
function SignInPrompt() {
  return (
    <div className="flex h-screen w-full bg-[#0A0A0A] items-center justify-center">
      <div className="flex flex-col items-center gap-8 p-8 max-w-md w-full">
        {/* Ripple Logo */}
        <div className="w-16 h-16 mb-2">
          <img 
            src="/images/icon.png" 
            alt="Ripple" 
            className="w-full h-full object-contain"
          />
        </div>
        
        <div className="flex flex-col items-center gap-4">
          <h1 className="text-2xl font-semibold tracking-tight text-center text-white">
            Sign in to continue
          </h1>
          <p className="text-sm text-[#9BA1A6] text-center max-w-sm">
            Access to this dashboard requires authentication. Continue to launch the login flow.
          </p>
        </div>
        
        <Button
          onClick={() => {
            window.location.href = getLoginUrl();
          }}
          size="lg"
          className="w-full bg-gradient-to-r from-[#00D4FF] to-[#9B7BFF] hover:opacity-90 text-white font-medium shadow-lg hover:shadow-xl transition-all"
        >
          Sign in
        </Button>
      </div>
    </div>
  );
}

export function TwoLevelSidebar({ children }: TwoLevelSidebarProps) {
  const { loading, user } = useAuth();

  // Show loading state
  if (loading) {
    return <SidebarLayoutSkeleton />;
  }

  // Show sign in prompt if not authenticated
  if (!user) {
    return <SignInPrompt />;
  }

  return (
    <div className="flex h-screen w-full bg-[#0A0A0A]">
      {/* Sidebar container - flex-shrink-0 prevents compression */}
      <div className="flex h-full flex-shrink-0">
        <IconNavigation />
        <DetailSidebar />
      </div>
      {/* Main content area - flex-1 takes remaining space */}
      {children && (
        <div className="flex-1 h-full overflow-auto bg-[#0F0F0F]">
          {children}
        </div>
      )}
    </div>
  );
}

// Standalone sidebar for drawer usage
export function SidebarContent() {
  return (
    <div className="flex h-full flex-shrink-0 bg-[#0A0A0A]">
      <IconNavigation />
      <DetailSidebar />
    </div>
  );
}

export default TwoLevelSidebar;
