const CHAT_API_BASE_URL = 'http://localhost:9000'; // FastAPI backend URL

export const sendMessage = async (user_query: string, selected_text?: string) => {
  try {
    const response = await fetch(`${CHAT_API_BASE_URL}/chat`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ query: user_query, selected_text: selected_text }),
    });

    if (!response.ok) {
      const errorData = await response.json();
      throw new Error(errorData.detail || 'Something went wrong with the chat API.');
    }

    const data = await response.json();
    // The backend returns 'response' field, but handle both 'response' and 'answer' for compatibility
    return data.response || data.answer;
  } catch (error) {
    console.error('Error sending message to chatbot API:', error);
    throw error;
  }
};
