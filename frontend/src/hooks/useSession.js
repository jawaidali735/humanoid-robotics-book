import { useState, useEffect } from 'react';
import { sessionManager } from '../lib/server-session-utils';

// Custom hook to check session status
const useSession = () => {
  const [session, setSession] = useState(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);

  useEffect(() => {
    const checkSession = async () => {
      try {
        setLoading(true);
        const sessionData = await sessionManager.getSession();

        if (sessionData.isValid && sessionData.authenticated) {
          setSession({
            user: sessionData.user,
            authenticated: true,
            sessionId: sessionData.sessionId,
            timestamp: sessionData.timestamp
          });
        } else {
          setSession({
            user: null,
            authenticated: false,
            sessionId: null,
            timestamp: new Date().toISOString()
          });
        }
        setError(null);
      } catch (err) {
        console.error('Session check error:', err);
        setError(err.message);
        setSession({
          user: null,
          authenticated: false,
          sessionId: null,
          timestamp: new Date().toISOString()
        });
      } finally {
        setLoading(false);
      }
    };

    // Check session initially
    checkSession();

    // Set up interval to periodically check session status (every 30 seconds)
    const interval = setInterval(checkSession, 30000);

    return () => {
      clearInterval(interval);
    };
  }, []);

  // Function to manually refresh session
  const refresh = async () => {
    try {
      setLoading(true);
      const sessionData = await sessionManager.getSession();

      if (sessionData.isValid && sessionData.authenticated) {
        const newSession = {
          user: sessionData.user,
          authenticated: true,
          sessionId: sessionData.sessionId,
          timestamp: sessionData.timestamp
        };
        setSession(newSession);
        setError(null);
        return newSession;
      } else {
        const newSession = {
          user: null,
          authenticated: false,
          sessionId: null,
          timestamp: new Date().toISOString()
        };
        setSession(newSession);
        setError(null);
        return newSession;
      }
    } catch (err) {
      console.error('Session refresh error:', err);
      setError(err.message);
      const failedSession = {
        user: null,
        authenticated: false,
        sessionId: null,
        timestamp: new Date().toISOString()
      };
      setSession(failedSession);
      return failedSession;
    } finally {
      setLoading(false);
    }
  };

  return {
    session,
    loading,
    error,
    refresh,
    isAuthenticated: session?.authenticated === true,
    user: session?.user || null
  };
};

export default useSession;