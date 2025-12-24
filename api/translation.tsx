// Frontend API client for translation service
const API_BASE_URL = 'http://127.0.0.1:8000/api';

/**
 * Translates text from source language to target language
 * @param {string} text - The text to translate
 * @param {string} sourceLanguage - Source language code (default: 'en')
 * @param {string} targetLanguage - Target language code (default: 'ur')
 * @returns {Promise<Object>} Translation response object
 */
export const translateText = async (text, sourceLanguage = 'en', targetLanguage = 'ur') => {
  try {
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), 30000); // 30 second timeout

    const response = await fetch(`${API_BASE_URL}/translate`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        text,
        sourceLanguage,
        targetLanguage
      }),
      signal: controller.signal
    });

    clearTimeout(timeoutId);

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.error || `HTTP error! status: ${response.status}`);
    }

    return await response.json();
  } catch (error) {
    console.error('Translation API error:', error);

    if (error.name === 'AbortError') {
      throw new Error('Request timed out. Please try again.');
    }

    if (error.message.includes('Failed to fetch')) {
      throw new Error('Network error. Please check your connection and try again.');
    }

    throw error;
  }
};