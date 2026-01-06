// TranslationJob model for tracking translation tasks
export interface TranslationJob {
  id: string;
  sourceFilePath: string;
  targetFilePath: string;
  status: 'pending' | 'in_progress' | 'completed' | 'failed' | 'review_required';
  sourceLanguage: string;
  targetLanguage: string;
  aiModelUsed: string;
  timestamp: Date;
  completionTime?: Date;
  notes?: string;
}

// Factory function to create a new TranslationJob
export const createTranslationJob = (
  sourceFilePath: string,
  targetFilePath: string,
  sourceLanguage: string,
  targetLanguage: string,
  aiModelUsed: string = 'Claude-3'
): TranslationJob => {
  return {
    id: `job_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
    sourceFilePath,
    targetFilePath,
    status: 'pending',
    sourceLanguage,
    targetLanguage,
    aiModelUsed,
    timestamp: new Date(),
  };
};

// Utility functions for working with TranslationJob
export const updateTranslationJobStatus = (
  job: TranslationJob,
  status: TranslationJob['status'],
  notes?: string
): TranslationJob => {
  return {
    ...job,
    status,
    completionTime: status === 'completed' || status === 'failed' ? new Date() : job.completionTime,
    notes: notes || job.notes,
  };
};

export const isJobCompleted = (job: TranslationJob): boolean => {
  return job.status === 'completed' || job.status === 'failed';
};

export const isJobInProcess = (job: TranslationJob): boolean => {
  return job.status === 'in_progress';
};