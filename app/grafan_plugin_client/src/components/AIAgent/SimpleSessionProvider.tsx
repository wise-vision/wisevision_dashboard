import React, { createContext, useContext, useEffect, useState } from 'react';
import { SimpleSessionManager, SimpleSessionState } from '../services/SimpleSessionManager';

interface SimpleSessionContextType {
  sessionManager: SimpleSessionManager;
  sessionState: SimpleSessionState;
}

const SimpleSessionContext = createContext<SimpleSessionContextType | null>(null);

interface SimpleSessionProviderProps {
  sessionManager: SimpleSessionManager;
  children: React.ReactNode;
}

export function SimpleSessionProvider({ sessionManager, children }: SimpleSessionProviderProps) {
  const [sessionState, setSessionState] = useState<SimpleSessionState>(sessionManager.getState());

  useEffect(() => {
    const unsubscribe = sessionManager.subscribe(setSessionState);
    
    // Try to connect on mount
    sessionManager.connect();
    
    return () => {
      unsubscribe();
    };
  }, [sessionManager]);

  return (
    <SimpleSessionContext.Provider value={{ sessionManager, sessionState }}>
      {children}
    </SimpleSessionContext.Provider>
  );
}

export function useSimpleSession() {
  const context = useContext(SimpleSessionContext);
  if (!context) {
    throw new Error('useSimpleSession must be used within SimpleSessionProvider');
  }
  return context;
}
