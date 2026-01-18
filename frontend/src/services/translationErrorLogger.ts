// Translation error logging and monitoring service

export interface TranslationError {
  id: string;
  jobId?: string;
  timestamp: Date;
  sourceFile?: string;
  targetFile?: string;
  sourceLanguage: string;
  targetLanguage: string;
  errorMessage: string;
  errorType: 'api_error' | 'parsing_error' | 'validation_error' | 'unknown_error';
  stackTrace?: string;
}

export interface TranslationMetrics {
  totalTranslations: number;
  successfulTranslations: number;
  failedTranslations: number;
  averageTranslationTime: number; // in milliseconds
  errorRate: number; // percentage
}

class TranslationErrorLogger {
  private errors: TranslationError[] = [];
  private startTime: Map<string, number> = new Map(); // jobId to start time

  /**
   * Logs a translation error
   */
  logError(error: Omit<TranslationError, 'id' | 'timestamp'>): TranslationError {
    const translationError: TranslationError = {
      id: `error_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
      timestamp: new Date(),
      ...error
    };

    this.errors.push(translationError);
    
    // Log to console for development
    console.error(`Translation Error [${translationError.errorType}]: ${translationError.errorMessage}`, {
      jobId: error.jobId,
      sourceFile: error.sourceFile,
      targetFile: error.targetFile
    });

    // In a real implementation, you would also:
    // - Send the error to an external logging service (e.g., Sentry, LogRocket)
    // - Store in a database for analysis
    // - Trigger alerts for critical errors
    
    return translationError;
  }

  /**
   * Starts timing a translation job
   */
  startJobTiming(jobId: string): void {
    this.startTime.set(jobId, Date.now());
  }

  /**
   * Ends timing a translation job and returns the duration
   */
  endJobTiming(jobId: string): number {
    const startTime = this.startTime.get(jobId);
    if (startTime) {
      const duration = Date.now() - startTime;
      this.startTime.delete(jobId);
      return duration;
    }
    return 0;
  }

  /**
   * Gets recent errors
   */
  getRecentErrors(limit: number = 10): TranslationError[] {
    return [...this.errors]
      .sort((a, b) => b.timestamp.getTime() - a.timestamp.getTime())
      .slice(0, limit);
  }

  /**
   * Gets error count by type
   */
  getErrorCountByType(): Record<string, number> {
    const counts: Record<string, number> = {};
    
    for (const error of this.errors) {
      counts[error.errorType] = (counts[error.errorType] || 0) + 1;
    }
    
    return counts;
  }

  /**
   * Gets translation metrics
   */
  getMetrics(): TranslationMetrics {
    const total = this.errors.length;
    const failed = this.errors.length; // In this simplified version, all logged items are errors
    const successful = 0; // Would need to track successful translations separately in a real implementation
    
    // Calculate average time based on completed jobs
    // This is a simplified calculation - in reality, you'd track successful translations too
    const durations = Array.from(this.startTime.values());
    const avgTime = durations.length > 0 
      ? durations.reduce((sum, time) => sum + time, 0) / durations.length 
      : 0;

    return {
      totalTranslations: total,
      successfulTranslations: successful,
      failedTranslations: failed,
      averageTranslationTime: avgTime,
      errorRate: total > 0 ? (failed / total) * 100 : 0
    };
  }

  /**
   * Clears old errors (older than specified days)
   */
  clearOldErrors(days: number = 7): number {
    const cutoffTime = new Date();
    cutoffTime.setDate(cutoffTime.getDate() - days);
    
    const initialCount = this.errors.length;
    this.errors = this.errors.filter(error => error.timestamp >= cutoffTime);
    
    return initialCount - this.errors.length;
  }
}

// Export a singleton instance of the error logger
export const translationErrorLogger = new TranslationErrorLogger();

// Utility function to wrap translation calls with error handling
export const withErrorHandling = async <T>(
  operation: () => Promise<T>,
  context: Omit<TranslationError, 'id' | 'timestamp' | 'errorMessage' | 'errorType'>
): Promise<T> => {
  try {
    translationErrorLogger.startJobTiming(context.jobId || 'unknown');
    const result = await operation();
    const duration = translationErrorLogger.endJobTiming(context.jobId || 'unknown');
    
    // In a real implementation, you would log successful translations here
    console.log(`Translation completed successfully for job ${context.jobId}, duration: ${duration}ms`);
    
    return result;
  } catch (error) {
    translationErrorLogger.endJobTiming(context.jobId || 'unknown');
    
    const errorToLog: Omit<TranslationError, 'id' | 'timestamp'> = {
      ...context,
      errorMessage: error instanceof Error ? error.message : 'Unknown error occurred',
      errorType: error instanceof Error && error.message.includes('API') ? 'api_error' : 'unknown_error',
      stackTrace: error instanceof Error ? error.stack : undefined
    };
    
    translationErrorLogger.logError(errorToLog);
    throw error; // Re-throw the error to be handled by the caller
  }
};