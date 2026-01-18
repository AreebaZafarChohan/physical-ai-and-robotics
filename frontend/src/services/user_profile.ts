// frontend/src/services/user_profile.ts
import axios from 'axios';
import { getToken } from '../utils/session';

import { API_URLS } from '../config/api.config';
const API_BASE_URL = API_URLS.USER_URL;

interface UserProfile {
  id: number;
  email: string;
}

interface PersonalizationData {
  software_background: string[];
  hardware_background: string[];
}

const getAuthHeaders = () => {
  const token = getToken();
  if (!token) {
    throw new Error('No authentication token found.');
  }
  return {
    headers: {
      Authorization: `Bearer ${token}`,
    },
  };
};

export const fetchUserProfile = async (): Promise<UserProfile> => {
  try {
    const response = await axios.get<UserProfile>(`${API_BASE_URL}/profile`, getAuthHeaders());
    return response.data;
  } catch (error: any) {
    if (axios.isAxiosError(error) && error.response) {
      throw new Error(error.response.data.detail || 'Failed to fetch user profile.');
    }
    throw new Error('An unexpected error occurred while fetching user profile.');
  }
};

export const fetchPersonalizationData = async (): Promise<PersonalizationData> => {
  try {
    const response = await axios.get<PersonalizationData>(`${API_BASE_URL}/personalization-data`, getAuthHeaders());
    return response.data;
  } catch (error: any) {
    if (axios.isAxiosError(error) && error.response) {
      throw new Error(error.response.data.detail || 'Failed to fetch personalization data.');
    }
    throw new Error('An unexpected error occurred while fetching personalization data.');
  }
};
