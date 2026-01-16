import { getApiBaseUrl } from '../config/api.config';
const CHAT_API_BASE_URL = getApiBaseUrl();

export const sendMessage = async (user_query: string, selected_text?: string, userId: string | null = null, accessToken: string | null = null) => {
  const headers: HeadersInit = {
    'Content-Type': 'application/json',
  };

  if (accessToken) {
    headers['Authorization'] = `Bearer ${accessToken}`;
  }

  const body: { query: string; selected_text?: string; user_id?: string | null } = {
    query: user_query,
    selected_text: selected_text,
  };

  if (userId) {
    body.user_id = userId;
  }

  try {
    const response = await fetch(`${CHAT_API_BASE_URL}/chat`, {
      method: 'POST',
      headers: headers,
      body: JSON.stringify(body),
    });

    if (!response.ok) {
      const errorData = await response.json();
      throw new Error(errorData.detail || 'Something went wrong with the chat API.');
    }

    const data = await response.json();
    return data.response || data.answer;
  } catch (error) {
    console.error('Error sending message to chatbot API:', error);
    throw error;
  }
};
