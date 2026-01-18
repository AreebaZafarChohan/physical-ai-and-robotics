import { TranslationJob } from '../models/TranslationJob';

export interface ReviewItem {
  id: string;
  jobId: string;
  sourceFilePath: string;
  targetFilePath: string;
  sourceContent: string;
  translatedContent: string;
  targetLanguage: string;
  sourceLanguage: string;
  submittedAt: Date;
  reviewedAt?: Date;
  reviewedBy?: string;
  status: 'pending' | 'approved' | 'rejected' | 'needs_revision';
  reasonForReview?: string; // Why this was flagged for review
  reviewerNotes?: string;   // Notes from the reviewer
}

export interface ReviewQueueStats {
  totalItems: number;
  pendingItems: number;
  approvedItems: number;
  rejectedItems: number;
  averageReviewTime: number; // in milliseconds
}

class ManualReviewQueue {
  private reviewItems: ReviewItem[] = [];

  /**
   * Adds a translation to the review queue
   */
  async addItem(
    jobId: string,
    sourceFilePath: string,
    targetFilePath: string,
    sourceContent: string,
    translatedContent: string,
    targetLanguage: string,
    sourceLanguage: string,
    reasonForReview?: string
  ): Promise<ReviewItem> {
    const reviewItem: ReviewItem = {
      id: `review_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
      jobId,
      sourceFilePath,
      targetFilePath,
      sourceContent,
      translatedContent,
      targetLanguage,
      sourceLanguage,
      submittedAt: new Date(),
      status: 'pending',
      reasonForReview
    };

    this.reviewItems.push(reviewItem);
    
    console.log(`Added item to review queue: ${reviewItem.id} for job ${jobId}`);
    
    return reviewItem;
  }

  /**
   * Gets items from the review queue based on status
   */
  getItems(status?: ReviewItem['status'], limit?: number): ReviewItem[] {
    let items = [...this.reviewItems];
    
    if (status) {
      items = items.filter(item => item.status === status);
    }
    
    items.sort((a, b) => b.submittedAt.getTime() - a.submittedAt.getTime());
    
    if (limit) {
      items = items.slice(0, limit);
    }
    
    return items;
  }

  /**
   * Approves a review item
   */
  async approveItem(
    reviewId: string,
    reviewedBy: string,
    reviewerNotes?: string
  ): Promise<ReviewItem | null> {
    const item = this.reviewItems.find(i => i.id === reviewId);
    
    if (!item) {
      return null;
    }

    item.status = 'approved';
    item.reviewedAt = new Date();
    item.reviewedBy = reviewedBy;
    item.reviewerNotes = reviewerNotes;

    console.log(`Approved review item: ${reviewId}`);
    
    return item;
  }

  /**
   * Rejects a review item
   */
  async rejectItem(
    reviewId: string,
    reviewedBy: string,
    reviewerNotes?: string
  ): Promise<ReviewItem | null> {
    const item = this.reviewItems.find(i => i.id === reviewId);
    
    if (!item) {
      return null;
    }

    item.status = 'rejected';
    item.reviewedAt = new Date();
    item.reviewedBy = reviewedBy;
    item.reviewerNotes = reviewerNotes;

    console.log(`Rejected review item: ${reviewId}`);
    
    return item;
  }

  /**
   * Marks a review item as needing revision
   */
  async markForRevision(
    reviewId: string,
    reviewedBy: string,
    reviewerNotes: string
  ): Promise<ReviewItem | null> {
    const item = this.reviewItems.find(i => i.id === reviewId);
    
    if (!item) {
      return null;
    }

    item.status = 'needs_revision';
    item.reviewedAt = new Date();
    item.reviewedBy = reviewedBy;
    item.reviewerNotes = reviewerNotes;

    console.log(`Marked review item for revision: ${reviewId}`);
    
    return item;
  }

  /**
   * Gets queue statistics
   */
  getStats(): ReviewQueueStats {
    const total = this.reviewItems.length;
    const pending = this.reviewItems.filter(i => i.status === 'pending').length;
    const approved = this.reviewItems.filter(i => i.status === 'approved').length;
    const rejected = this.reviewItems.filter(i => i.status === 'rejected').length;
    
    // Calculate average review time for completed items
    const completedItems = this.reviewItems.filter(i => i.reviewedAt);
    const totalReviewTime = completedItems.reduce((sum, item) => {
      if (item.submittedAt && item.reviewedAt) {
        return sum + (item.reviewedAt.getTime() - item.submittedAt.getTime());
      }
      return sum;
    }, 0);
    
    const averageReviewTime = completedItems.length > 0 
      ? totalReviewTime / completedItems.length 
      : 0;

    return {
      totalItems: total,
      pendingItems: pending,
      approvedItems: approved,
      rejectedItems: rejected,
      averageReviewTime
    };
  }

  /**
   * Removes old completed items from the queue (older than specified days)
   */
  clearOldItems(days: number = 30): number {
    const cutoffTime = new Date();
    cutoffTime.setDate(cutoffTime.getDate() - days);
    
    const initialCount = this.reviewItems.length;
    this.reviewItems = this.reviewItems.filter(item => {
      // Keep items that are still pending or were recently completed
      if (item.status === 'pending') return true;
      if (!item.reviewedAt) return true; // Keep unreviewed items
      return item.reviewedAt >= cutoffTime;
    });
    
    return initialCount - this.reviewItems.length;
  }

  /**
   * Flags a translation job for manual review
   */
  async flagForReview(
    jobId: string,
    reason: string,
    additionalContext?: {
      sourceFilePath?: string;
      targetFilePath?: string;
      sourceContent?: string;
      translatedContent?: string;
      targetLanguage?: string;
      sourceLanguage?: string;
    }
  ): Promise<ReviewItem | null> {
    // In a real implementation, you would fetch the job details from the job store
    // For now, we'll create a review item with the provided context
    
    if (!additionalContext || !additionalContext.sourceFilePath || !additionalContext.targetFilePath) {
      console.error('Missing required context for review flagging');
      return null;
    }

    return this.addItem(
      jobId,
      additionalContext.sourceFilePath,
      additionalContext.targetFilePath,
      additionalContext.sourceContent || '',
      additionalContext.translatedContent || '',
      additionalContext.targetLanguage || 'ur',
      additionalContext.sourceLanguage || 'en',
      reason
    );
  }
}

// Export a singleton instance of the review queue
export const manualReviewQueue = new ManualReviewQueue();

// Function to determine if a translation should be flagged for review
export const shouldFlagForReview = (
  sourceContent: string,
  translatedContent: string,
  confidenceThreshold: number = 0.7
): { shouldFlag: boolean; reason: string } => {
  // Check if the translated content is significantly shorter than the source (might indicate missing content)
  if (translatedContent.length < sourceContent.length * 0.5) {
    return {
      shouldFlag: true,
      reason: 'Significantly shorter translation may be missing content'
    };
  }

  // Check for common issues in the translated content
  const issues = [];
  
  // Check for placeholder text or incomplete translations
  if (translatedContent.toLowerCase().includes('translation') || 
      translatedContent.toLowerCase().includes('translate')) {
    issues.push('Contains translation-related placeholder text');
  }

  // Check if code blocks are preserved properly
  const sourceCodeBlocks = (sourceContent.match(/```[\s\S]*?```/g) || []).length;
  const translatedCodeBlocks = (translatedContent.match(/```[\s\S]*?```/g) || []).length;
  
  if (sourceCodeBlocks !== translatedCodeBlocks) {
    issues.push('Code blocks not properly preserved');
  }

  if (issues.length > 0) {
    return {
      shouldFlag: true,
      reason: issues.join('; ')
    };
  }

  // If no issues detected, don't flag for review
  return {
    shouldFlag: false,
    reason: ''
  };
};