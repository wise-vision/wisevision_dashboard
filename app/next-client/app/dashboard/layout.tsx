'use client';

import { useState, useEffect } from 'react';
import { usePathname } from 'next/navigation';
import { ThemeProvider } from '../../components/theme-provider';
import Sidebar from '../../components/layout/Sidebar';
import Header from '../../components/layout/Header';

export default function DashboardLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  const pathname = usePathname();
  const [sidebarCollapsed, setSidebarCollapsed] = useState(false);

  // Load sidebar state from localStorage on client side
  useEffect(() => {
    const savedState = localStorage.getItem('sidebarCollapsed');
    if (savedState !== null) {
      setSidebarCollapsed(savedState === 'true');
    }
  }, []);

  const toggleSidebar = () => {
    const newState = !sidebarCollapsed;
    setSidebarCollapsed(newState);
    localStorage.setItem('sidebarCollapsed', String(newState));
  };

  return (
    <ThemeProvider attribute="class" defaultTheme="light" enableSystem={false}>
      <div className="min-h-screen flex flex-col">
        <Header toggleSidebar={toggleSidebar} />
        <div className="flex flex-1">
          <Sidebar collapsed={sidebarCollapsed} setCollapsed={setSidebarCollapsed} />
          <main className={`flex-1 transition-all duration-300 ease-in-out p-4 ${sidebarCollapsed ? 'ml-0 md:ml-16' : 'ml-0 md:ml-64'}`}>
            {children}
          </main>
        </div>
      </div>
    </ThemeProvider>
  );
}