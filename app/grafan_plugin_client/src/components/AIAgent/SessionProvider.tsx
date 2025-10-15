import React, { createContext, useContext, useEffect, useState } from 'react';
import { SessionManager } from '../services/SessionManager';
import { SessionState } from './types';

interface SessionContextType {
  sessionManager: SessionManager;
  sessionState: SessionState;
}

const SessionContext = createContext<SessionContextType | null>(null);

interface SessionProviderProps {
  sessionManager: SessionManager;
  children: React.ReactNode;
}

export function SessionProvider({ sessionManager, children }: SessionProviderProps) {
  const [sessionState, setSessionState] = useState<SessionState>(sessionManager.getState());

  useEffect(() => {
    const unsubscribe = sessionManager.subscribe(setSessionState);
    return () => {
      unsubscribe();
    };
  }, [sessionManager]);

  return (
    <SessionContext.Provider value={{ sessionManager, sessionState }}>
      {children}
    </SessionContext.Provider>
  );
}

export function useSession() {
  const context = useContext(SessionContext);
  if (!context) {
    throw new Error('useSession must be used within SessionProvider');
  }
  return context;
}
