/**
 * Session Manager for Physical AI & Humanoid Robotics RAG System
 * Handles chat session persistence and management
 */

class SessionManager {
  constructor(options = {}) {
    this.sessions = new Map(); // In-memory storage (use Redis in production)
    this.sessionTimeout = options.sessionTimeout || 30 * 60 * 1000; // 30 minutes
    this.maxSessions = options.maxSessions || 1000; // Maximum concurrent sessions

    // Cleanup expired sessions periodically
    setInterval(() => {
      this.cleanupExpiredSessions();
    }, 5 * 60 * 1000); // Every 5 minutes
  }

  /**
   * Create a new session
   */
  createSession(sessionId, userData = {}) {
    // Clean up oldest sessions if we exceed the limit
    if (this.sessions.size >= this.maxSessions) {
      this.removeOldestSessions();
    }

    const session = {
      id: sessionId,
      createdAt: new Date().toISOString(),
      lastActivity: new Date().toISOString(),
      userData: userData,
      queryCount: 0,
      messages: [],
      active: true,
      expiresAt: new Date(Date.now() + this.sessionTimeout).toISOString()
    };

    this.sessions.set(sessionId, session);
    return session;
  }

  /**
   * Get session by ID
   */
  getSession(sessionId) {
    const session = this.sessions.get(sessionId);

    if (session) {
      // Update last activity
      session.lastActivity = new Date().toISOString();

      // Extend session timeout
      session.expiresAt = new Date(Date.now() + this.sessionTimeout).toISOString();

      return session;
    }

    return null;
  }

  /**
   * Add a message to a session
   */
  addMessageToSession(sessionId, message) {
    const session = this.getSession(sessionId);

    if (!session) {
      throw new Error(`Session ${sessionId} not found`);
    }

    const messageWithTimestamp = {
      ...message,
      timestamp: new Date().toISOString()
    };

    session.messages.push(messageWithTimestamp);
    session.queryCount++;

    return session;
  }

  /**
   * End a session
   */
  endSession(sessionId) {
    const session = this.sessions.get(sessionId);

    if (session) {
      session.active = false;
      session.endDate = new Date().toISOString();
      return session;
    }

    return null;
  }

  /**
   * Check if a session exists and is active
   */
  isActiveSession(sessionId) {
    const session = this.getSession(sessionId);
    return session && session.active && !this.isSessionExpired(session);
  }

  /**
   * Check if a session is expired
   */
  isSessionExpired(session) {
    return new Date() > new Date(session.expiresAt);
  }

  /**
   * Cleanup expired sessions
   */
  cleanupExpiredSessions() {
    const now = new Date();
    for (const [sessionId, session] of this.sessions.entries()) {
      if (now > new Date(session.expiresAt)) {
        this.sessions.delete(sessionId);
      }
    }
  }

  /**
   * Remove oldest sessions to make space
   */
  removeOldestSessions() {
    const sortedSessions = Array.from(this.sessions.entries()).sort(
      (a, b) => new Date(a[1].createdAt) - new Date(b[1].createdAt)
    );

    // Remove 10% of the oldest sessions
    const sessionsToRemove = Math.floor(sortedSessions.length * 0.1);
    for (let i = 0; i < sessionsToRemove; i++) {
      this.sessions.delete(sortedSessions[i][0]);
    }
  }

  /**
   * Get session statistics
   */
  getSessionStats() {
    const activeSessions = Array.from(this.sessions.values()).filter(s => s.active && !this.isSessionExpired(s));
    const totalSessions = this.sessions.size;
    const expiredSessions = totalSessions - activeSessions.length;

    return {
      total: totalSessions,
      active: activeSessions.length,
      expired: expiredSessions,
      maxCapacity: this.maxSessions
    };
  }
}

// Singleton instance
const sessionManager = new SessionManager();

module.exports = sessionManager;