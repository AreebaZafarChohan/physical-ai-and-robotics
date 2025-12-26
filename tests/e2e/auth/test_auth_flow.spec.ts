import { test, expect } from '@playwright/test';

test.describe('Authentication Flow E2E Tests', () => {
  const BASE_URL = 'http://localhost:3000';
  const BACKEND_URL = 'http://localhost:9000';

  test.beforeEach(async ({ page }) => {
    // Clear local storage and navigate to home before each test
    await page.goto(BASE_URL);
    await page.evaluate(() => localStorage.clear());
    await page.goto(BASE_URL); // Navigate again to ensure state is clear
  });

  test('should allow a new user to sign up with email and provide experience', async ({ page }) => {
    await page.goto(`${BASE_URL}/signup`);
    await expect(page).toHaveURL(`${BASE_URL}/signup`);

    await page.fill('input[id="email"]', `test-user-${Date.now()}@example.com`);
    await page.fill('input[id="password"]', 'testpassword123');
    await page.fill('textarea[id="softwareExperience"]', 'Playwright, TypeScript');
    await page.fill('textarea[id="hardwareExperience"]', 'Robotics, Sensors');
    await page.click('button[type="submit"]');

    // Assuming successful signup redirects to home page or similar and sets auth state
    await expect(page).toHaveURL(BASE_URL);
    await expect(page.evaluate(() => localStorage.getItem('user_id'))).toBeTruthy();
    
    // In a real scenario, you'd check for a success message or presence of authenticated UI elements
  });

  test('should allow an existing user to sign in with email', async ({ page }) => {
    // First, create a user via API or signup flow if not already handled
    const userEmail = `existing-user-${Date.now()}@example.com`;
    // For a real E2E test, you'd likely use a beforeEach or setup hook to create the user via backend API
    // For this example, we'll simulate a quick signup first then sign out and sign in.
    await page.goto(`${BASE_URL}/signup`);
    await page.fill('input[id="email"]', userEmail);
    await page.fill('input[id="password"]', 'existingpassword123');
    await page.fill('textarea[id="softwareExperience"]', 'Existing SW');
    await page.fill('textarea[id="hardwareExperience"]', 'Existing HW');
    await page.click('button[type="submit"]');
    await expect(page).toHaveURL(BASE_URL);
    await page.evaluate(() => localStorage.clear()); // Simulate logout


    await page.goto(`${BASE_URL}/signin`);
    await expect(page).toHaveURL(`${BASE_URL}/signin`);

    await page.fill('input[id="email"]', userEmail);
    await page.fill('input[id="password"]', 'existingpassword123');
    await page.click('button[type="submit"]');

    await expect(page).toHaveURL(BASE_URL);
    await expect(page.evaluate(() => localStorage.getItem('user_id'))).toBeTruthy();
  });

  test('should redirect to social provider for Google signup/signin', async ({ page }) => {
    await page.goto(`${BASE_URL}/signup`);
    await page.click('button:has-text("Sign Up with Google")');

    // Expect a redirect to the backend's social auth endpoint, which then redirects to Google
    await expect(page).toHaveURL(new RegExp(`${BACKEND_URL}/api/v1/auth/google`));
    // In a full E2E test, you'd then handle the actual Google login, but for now, we check the redirect.
    // Further testing would involve mocking the external OAuth flow if not possible to fully automate.
  });

  test('should show experience form after social login if data is default', async ({ page }) => {
    // This test is harder to fully automate without mocking the backend's social callback
    // to return a specific user_id and then navigating to the URL with that user_id.
    // For simplicity, we'll directly navigate to a simulated post-social-login state
    // with a user_id and assert the form appears.

    const socialUserId = `social-test-${Date.now()}`;
    const socialUserEmail = `social-test-${Date.now()}@example.com`;

    // Simulate backend's social callback redirecting to frontend with user_id
    // First, we need to ensure this user exists in our mock backend for fetching
    // This would ideally be done by a test setup fixture creating a user in the test DB
    // For now, we simulate by directly fetching the user details from the backend route

    // Mock the backend call to fetch user to return a user with default experience
    await page.route(`${BACKEND_URL}/api/v1/auth/user/${socialUserId}`, async route => {
      await route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({
          id: socialUserId,
          email: socialUserEmail,
          software_experience: "Not provided via social login",
          hardware_experience: "Not provided via social login",
          created_at: new Date().toISOString(),
          updated_at: new Date().toISOString(),
        }),
      });
    });

    await page.goto(`${BASE_URL}/?user_id=${socialUserId}`);

    // Expect the experience form to be visible
    await expect(page.locator('h2:has-text("Tell Us About Your Experience")')).toBeVisible();
    await expect(page.locator('textarea[id="softwareExperience"]')).toBeVisible();
    await expect(page.locator('textarea[id="hardwareExperience"]')).toBeVisible();

    // Fill and submit the form
    await page.fill('textarea[id="softwareExperience"]', 'Social Playwright SW');
    await page.fill('textarea[id="hardwareExperience"]', 'Social Playwright HW');
    
    // Mock the backend call to update user experience
    await page.route(`${BACKEND_URL}/api/v1/auth/user/${socialUserId}/experience`, async route => {
      await route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({
          id: socialUserId,
          email: socialUserEmail,
          software_experience: "Social Playwright SW",
          hardware_experience: "Social Playwright HW",
          created_at: new Date().toISOString(),
          updated_at: new Date().toISOString(),
        }),
      });
    });

    await page.click('button:has-text("Submit Experience")');

    // Expect the form to disappear after submission
    await expect(page.locator('h2:has-text("Tell Us About Your Experience")')).not.toBeVisible();
    
    // Verify user_id is cleared from URL
    await expect(page).toHaveURL(BASE_URL);
  });
});
