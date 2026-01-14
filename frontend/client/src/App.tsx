import { Toaster } from "@/components/ui/sonner";
import { TooltipProvider } from "@/components/ui/tooltip";
import NotFound from "@/pages/NotFound";
import { Route, Switch, useLocation } from "wouter";
import { useEffect } from "react";
import ErrorBoundary from "./components/ErrorBoundary";
import { ThemeProvider } from "./contexts/ThemeContext";
import { SidebarProvider, useSidebar } from "./contexts/SidebarContext";
import { TwoLevelSidebar, NotificationsPanel, FavoritesPanel, CommandPalette } from "./components/sidebar";
import { ROUTE_TO_SECTION } from "./data/sidebar-panels";
import Home from "./pages/Home";
import Dashboard from "./pages/Dashboard";
import Chat from "./pages/Chat";
import Files from "./pages/Files";
import Notifications from "./pages/Notifications";
import Billing from "./pages/Billing";
import Pricing from "./pages/Pricing";
import Settings from "./pages/Settings";

// Hook to sync route with active sidebar section
function useRouteSync() {
  const [location] = useLocation();
  const { setActiveSection } = useSidebar();

  useEffect(() => {
    // Find the matching section for the current route
    const section = ROUTE_TO_SECTION[location];
    if (section) {
      // Use a microtask to avoid state update during render
      queueMicrotask(() => {
        setActiveSection(section);
      });
    }
  }, [location, setActiveSection]);
}

// Wrapper for dashboard pages with the two-level sidebar
function SidebarWrapper({ children }: { children: React.ReactNode }) {
  useRouteSync();
  
  return (
    <>
      <TwoLevelSidebar>{children}</TwoLevelSidebar>
      <NotificationsPanel />
      <FavoritesPanel />
      <CommandPalette />
    </>
  );
}

function Router() {
  return (
    <Switch>
      {/* Public routes */}
      <Route path="/" component={Home} />
      <Route path="/pricing" component={Pricing} />
      
      {/* Dashboard routes with sidebar */}
      <Route path="/dashboard">
        <SidebarWrapper><Dashboard /></SidebarWrapper>
      </Route>
      <Route path="/chat">
        <SidebarWrapper><Chat /></SidebarWrapper>
      </Route>
      <Route path="/files">
        <SidebarWrapper><Files /></SidebarWrapper>
      </Route>
      <Route path="/notifications">
        <SidebarWrapper><Notifications /></SidebarWrapper>
      </Route>
      <Route path="/billing">
        <SidebarWrapper><Billing /></SidebarWrapper>
      </Route>
      <Route path="/settings">
        <SidebarWrapper><Settings /></SidebarWrapper>
      </Route>
      
      {/* Fallback routes */}
      <Route path="/404" component={NotFound} />
      <Route component={NotFound} />
    </Switch>
  );
}

// Using dark theme to match the ripple brand aesthetic
function App() {
  return (
    <ErrorBoundary>
      <ThemeProvider defaultTheme="dark">
        <SidebarProvider>
          <TooltipProvider>
            <Toaster />
            <Router />
          </TooltipProvider>
        </SidebarProvider>
      </ThemeProvider>
    </ErrorBoundary>
  );
}

export default App;
