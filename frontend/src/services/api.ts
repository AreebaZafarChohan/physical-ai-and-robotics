import axios, { AxiosResponse } from 'axios';
import { API_URLS, getApiBaseUrl } from '../config/api.config';

// Create an axios instance with default configuration
const apiClient = axios.create({
  baseURL: getApiBaseUrl(), // Dynamic base URL based on environment
  timeout: 30000, // 30 second timeout to accommodate longer processing times
  headers: {
    'Content-Type': 'application/json',
  },
});

// Update baseURL on each request (for dynamic URL changes)
apiClient.interceptors.request.use(
  (config) => {
    // Update base URL dynamically
    config.baseURL = getApiBaseUrl();

    // Get token from localStorage or other storage mechanism
    const token = localStorage.getItem('access_token');

    if (token) {
      config.headers.Authorization = `Bearer ${token}`;
    }

    return config;
  },
  (error) => {
    return Promise.reject(error);
  }
);

// Add response interceptor for error handling
apiClient.interceptors.response.use(
  (response) => response,
  (error) => {
    // Handle specific error responses here
    if (error.response?.status === 401) {
      // Token might be expired, redirect to login or refresh token
      console.error('Unauthorized access - token may be expired');
      // Optionally redirect user to login page
    }

    return Promise.reject(error);
  }
);

// Personalization API functions
export interface PersonalizeContentRequest {
  chapter_path: string;
}

export interface PersonalizeContentResponse {
  chapter_path: string;
  personalized_content: string;
  user_profile: {
    software_background: string[];
    hardware_background: string[];
    experience_level: string;
  };
  personalization_applied: boolean;
}

export interface PreviewPersonalizationRequest {
  chapter_path: string;
  profile_data: {
    software_background: string[];
    hardware_background: string[];
    experience_level: string;
  };
}

export interface PreviewPersonalizationResponse {
  chapter_path: string;
  personalized_content: string;
  applied_profile: {
    software_background: string[];
    hardware_background: string[];
    experience_level: string;
  };
  personalization_applied: boolean;
}

export interface PersonalizationRule {
  id: number;
  content_path: string;
  description: string;
  user_profile_criteria: Record<string, any>;
  priority: number;
  is_active: boolean;
  created_at: string;
  updated_at: string;
}

export interface PersonalizationRuleUpdate {
  content_path: string;
  user_profile_criteria: Record<string, any>;
  content_variants: Record<string, any>;
  description?: string;
  priority: number;
  is_active: boolean;
}

/**
 * Fetch personalized content for a specific chapter
 */
export const getPersonalizedContent = async (
  chapterPath: string
): Promise<PersonalizeContentResponse> => {
  console.log('API call for chapter path:', chapterPath);
  try {
    const response: AxiosResponse<PersonalizeContentResponse> = await apiClient.get(
      `/personalize/${chapterPath}`
    );
    console.log('API response:', response.data);
    return response.data;
  } catch (error) {
    console.error(`Error fetching personalized content for ${chapterPath}:`, error);
    throw error;
  }
};

/**
 * Preview personalized content with custom profile data
 */
export const previewPersonalizedContent = async (
  requestData: PreviewPersonalizationRequest
): Promise<PreviewPersonalizationResponse> => {
  try {
    const response: AxiosResponse<PreviewPersonalizationResponse> = await apiClient.post(
      '/personalize/preview',
      requestData
    );
    return response.data;
  } catch (error) {
    console.error('Error previewing personalized content:', error);
    throw error;
  }
};

/**
 * Get list of all active personalization rules
 */
export const getPersonalizationRules = async (): Promise<{ rules: PersonalizationRule[] }> => {
  try {
    const response: AxiosResponse<{ rules: PersonalizationRule[] }> = await apiClient.get(
      '/personalize/rules'
    );
    return response.data;
  } catch (error) {
    console.error('Error fetching personalization rules:', error);
    throw error;
  }
};

/**
 * Update a personalization rule
 */
export const updatePersonalizationRule = async (
  ruleId: number,
  updateData: PersonalizationRuleUpdate
): Promise<PersonalizationRule> => {
  try {
    const response: AxiosResponse<PersonalizationRule> = await apiClient.put(
      `/personalize/rules/${ruleId}`,
      updateData
    );
    return response.data;
  } catch (error) {
    console.error(`Error updating personalization rule ${ruleId}:`, error);
    throw error;
  }
};

/**
 * Update user profile
 */
export const updateUserProfile = async (profileData: {
  software_background?: string[];
  hardware_background?: string[];
  experience_level?: string;
}): Promise<any> => {
  try {
    const response = await apiClient.put('/user/profile/update', profileData);
    return response.data;
  } catch (error) {
    console.error('Error updating user profile:', error);
    throw error;
  }
};

/**
 * Get user profile details
 */
export const getUserProfileDetails = async (): Promise<any> => {
  try {
    const response = await apiClient.get('/user/profile/details');
    return response.data;
  } catch (error) {
    console.error('Error fetching user profile details:', error);
    throw error;
  }
};

// Export the API client for direct use if needed
export default apiClient;
