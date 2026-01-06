/**
 * API Configuration
 *
 * Handles API URL configuration for both development and production environments.
 * Works with Docusaurus (no process.env dependency)
 */

// Check if we're running in browser and determine environment
const getEnvironment = (): 'development' | 'production' => {
  // Server-side rendering check
  if (typeof window === 'undefined') {
    return 'production';
  }

  // Check hostname to determine environment
  const hostname = window.location.hostname;

  if (hostname === 'localhost' || hostname === '127.0.0.1') {
    return 'development';
  }

  return 'production';
};

// Development API URL (local backend)
const DEV_API_URL = 'http://localhost:9000';

// Production API URL - Your deployed backend URL
// TODO: Update this when you deploy your backend to a cloud service
const PROD_API_URL = 'http://localhost:9000'; // Change to your deployed backend URL

/**
 * Get the base API URL based on the current environment
 */
export const getApiBaseUrl = (): string => {
  const env = getEnvironment();

  if (env === 'development') {
    console.log('🔧 Development mode - API URL:', DEV_API_URL);
    return DEV_API_URL;
  }

  console.log('🌐 Production mode - API URL:', PROD_API_URL);
  return PROD_API_URL;
};

/**
 * API URL endpoints
 */
export const API_URLS = {
  get BASE_URL() {
    return getApiBaseUrl();
  },
  get AUTH_URL() {
    return `${getApiBaseUrl()}/auth`;
  },
  get USER_URL() {
    return `${getApiBaseUrl()}/user`;
  },
  get PERSONALIZE_URL() {
    return `${getApiBaseUrl()}/personalize`;
  },
  get FEEDBACK_URL() {
    return `${getApiBaseUrl()}/feedback`;
  },
};

/**
 * Check if the API is reachable
 */
export const checkApiHealth = async (): Promise<boolean> => {
  try {
    const response = await fetch(`${getApiBaseUrl()}/health`, {
      method: 'GET',
      mode: 'cors',
    });
    return response.ok;
  } catch (error) {
    console.error('API health check failed:', error);
    return false;
  }
};

export default API_URLS;