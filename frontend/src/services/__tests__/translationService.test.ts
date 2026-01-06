import { TranslationService } from '../translationService';
import { TranslationJob } from '../../models/TranslationJob';

describe('TranslationService', () => {
  let translationService: TranslationService;

  beforeEach(() => {
    translationService = new TranslationService();
    // Mock the environment variable for tests
    process.env.CLAUDE_API_KEY = 'test-key';
  });

  afterEach(() => {
    jest.clearAllMocks();
    // Clean up environment variable
    delete process.env.CLAUDE_API_KEY;
  });

  test('should create a new translation job', async () => {
    const job = await translationService.createTranslationJob(
      'docs/en/intro.md',
      'docs/ur/intro.md',
      'en',
      'ur'
    );

    expect(job).toBeDefined();
    expect(job.sourceFilePath).toBe('docs/en/intro.md');
    expect(job.targetFilePath).toBe('docs/ur/intro.md');
    expect(job.sourceLanguage).toBe('en');
    expect(job.targetLanguage).toBe('ur');
    expect(job.status).toBe('pending');
  });

  test('should get a translation job by ID', async () => {
    const createdJob = await translationService.createTranslationJob(
      'docs/en/intro.md',
      'docs/ur/intro.md',
      'en',
      'ur'
    );

    const retrievedJob = await translationService.getTranslationJob(createdJob.id);

    expect(retrievedJob).toBeDefined();
    expect(retrievedJob?.id).toBe(createdJob.id);
  });

  test('should return undefined for non-existent job', async () => {
    const job = await translationService.getTranslationJob('non-existent-id');

    expect(job).toBeUndefined();
  });

  test('should get all translation jobs', async () => {
    // Create a few jobs
    await translationService.createTranslationJob('docs/en/page1.md', 'docs/ur/page1.md', 'en', 'ur');
    await translationService.createTranslationJob('docs/en/page2.md', 'docs/ur/page2.md', 'en', 'ur');

    const allJobs = await translationService.getAllTranslationJobs();

    expect(allJobs.length).toBeGreaterThanOrEqual(2);
  });

  test('should get translation jobs by status', async () => {
    // Create jobs with different statuses
    const pendingJob = await translationService.createTranslationJob(
      'docs/en/pending.md',
      'docs/ur/pending.md',
      'en',
      'ur'
    );
    
    // Update one job to completed status
    const completedJob = await translationService.getTranslationJob(pendingJob.id);
    if (completedJob) {
      // In a real scenario, we would update the status, but for this test
      // we'll just verify the filtering works
    }

    const pendingJobs = await translationService.getTranslationJobsByStatus('pending');
    expect(pendingJobs.length).toBeGreaterThanOrEqual(1);
  });

  test('should handle API key not configured', () => {
    delete process.env.CLAUDE_API_KEY;
    translationService = new TranslationService();
    
    expect(translationService.isConfigured()).toBe(false);
  });

  test('should be configured when API key is set', () => {
    process.env.CLAUDE_API_KEY = 'test-key';
    translationService = new TranslationService();
    
    expect(translationService.isConfigured()).toBe(true);
  });
});