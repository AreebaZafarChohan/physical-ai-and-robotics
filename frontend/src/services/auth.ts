// frontend/src/services/auth.ts
import axios from 'axios';
import { API_URLS } from '../config/api.config';

// Dynamic API URLs based on environment
const getAuthUrl = () => API_URLS.AUTH_URL;
const getUserUrl = () => API_URLS.USER_URL;

export interface User {
  id: number;
  username: string;
  email: string;
  software_background?: string[];
  hardware_background?: string[];
  experience_level?: string;
}

interface RegistrationData {
  username: string;
  email: string;
  password: string;
  software_background?: string;
  hardware_background?: string;
}

interface AuthResponse {
  access_token: string;
  token_type: string;
}

export const registerUser = async (data: RegistrationData): Promise<AuthResponse> => {
  try {
    const response = await axios.post<AuthResponse>(`${getAuthUrl()}/signup`, {
      ...data,
      software_background: data.software_background?.split(',').map(s => s.trim()).filter(Boolean),
      hardware_background: data.hardware_background?.split(',').map(s => s.trim()).filter(Boolean),
    });
    localStorage.setItem('access_token', response.data.access_token);
    return response.data;
  } catch (error: any) {
    if (axios.isAxiosError(error) && error.response) {
      throw new Error(error.response.data.detail || 'Registration failed');
    }
    throw new Error('An unexpected error occurred during registration.');
  }
};

interface OAuthRegistrationData {
  email: string;
  oauth_provider_ids: { [key: string]: string };
  software_background?: string;
  hardware_background?: string;
}

export const registerOAuthUser = async (data: OAuthRegistrationData): Promise<AuthResponse> => {
  try {
    const response = await axios.post<AuthResponse>(`${getAuthUrl()}/signup/oauth`, {
      ...data,
      software_background: data.software_background?.split(',').map(s => s.trim()).filter(Boolean),
      hardware_background: data.hardware_background?.split(',').map(s => s.trim()).filter(Boolean),
    });
    localStorage.setItem('access_token', response.data.access_token);
    return response.data;
  } catch (error: any) {
    if (axios.isAxiosError(error) && error.response) {
      throw new Error(error.response.data.detail || 'OAuth registration failed');
    }
    throw new Error('An unexpected error occurred during OAuth registration.');
  }
};

interface LoginData {
  email: string;
  password: string;
}

export const loginUser = async (credentials: LoginData): Promise<AuthResponse> => {
  try {
    const response = await axios.post<AuthResponse>(
      `${getAuthUrl()}/login`,
      credentials,
    );
    localStorage.setItem('access_token', response.data.access_token);
    return response.data;
  } catch (error: any) {
    if (axios.isAxiosError(error) && error.response) {
      throw new Error(error.response.data.detail || 'Login failed');
    }
    throw new Error('An unexpected error occurred during login.');
  }
};

export const logoutUser = async (): Promise<void> => {
  const token = localStorage.getItem('access_token');
  if (token) {
    try {
      await axios.post(`${getAuthUrl()}/logout`, {}, {
        headers: {
          'Authorization': `Bearer ${token}`
        }
      });
    } catch (error) {
      // Even if logout fails on the backend, we should still clear the token on the client
      console.error('Logout failed on backend:', error);
    }
  }
  localStorage.removeItem('access_token');
  // Redirect or update UI state
  window.location.href = '/login';
};

export const getAuthHeaders = () => {
  const token = localStorage.getItem('access_token');
  if (token) {
    return {
      headers: {
        Authorization: `Bearer ${token}`,
      },
    };
  }
  return {};
};

export const getCurrentUser = async (): Promise<User | null> => {
  const token = localStorage.getItem('access_token');
  if (!token) {
    return null;
  }
  try {
    const response = await axios.get<User>(`${getUserUrl()}/profile`, {
      headers: {
        Authorization: `Bearer ${token}`,
      },
    });
    return response.data;
  } catch (error) {
    console.error('Failed to fetch current user:', error);
    // If fetching user fails, assume token is invalid and log out
    localStorage.removeItem('access_token');
    return null;
  }
};

export const isLoggedIn = (): boolean => {
  return localStorage.getItem('access_token') !== null;
};
