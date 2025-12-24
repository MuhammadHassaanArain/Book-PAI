import { useState, useCallback } from 'react';
import { translateText } from '../../api/translation'; // Path from src/hooks to root/api
import { capturePageContent, prepareContentForTranslation } from '../utils/contentCapture'; // Path from src/hooks to src/utils

/**
 * Custom hook for managing translation state and operations
 */
const useTranslation = (initialContent = null) => {
  const [isTranslating, setIsTranslating] = useState(false);
  const [isTranslated, setIsTranslated] = useState(false);
  const [originalContent, setOriginalContent] = useState(initialContent || '');
  const [translatedContent, setTranslatedContent] = useState('');

  /**
   * Translates the provided content to Urdu
   * @param {string} content - Content to translate
   * @returns {Promise<string>} Translated content
   */
  const translateContent = useCallback(async (content) => {
    if (!content) return '';

    setIsTranslating(true);
    try {
      const cleanedContent = prepareContentForTranslation(content);
      const response = await translateText(cleanedContent, 'en', 'ur');

      setTranslatedContent(response.translated_text);
      setOriginalContent(content);
      setIsTranslated(true);

      return response.translated_text;
    } catch (error) {
      console.error('Translation failed:', error);
      throw error;
    } finally {
      setIsTranslating(false);
    }
  }, []);

  /**
   * Toggles between original and translated content
   * @param {function} onContentUpdate - Callback to update content in parent component
   */
  const toggleTranslation = useCallback((onContentUpdate) => {
    if (isTranslated) {
      // Switch back to original content
      onContentUpdate(originalContent);
      setIsTranslated(false);
    } else {
      // Translate the content
      if (originalContent) {
        translateContent(originalContent)
          .then(translated => {
            onContentUpdate(translated);
          })
          .catch(error => {
            console.error('Toggle translation failed:', error);
          });
      } else {
        // If no original content, capture from page
        const capturedContent = capturePageContent();
        if (capturedContent) {
          setOriginalContent(capturedContent);
          translateContent(capturedContent)
            .then(translated => {
              onContentUpdate(translated);
            })
            .catch(error => {
              console.error('Toggle translation failed:', error);
            });
        }
      }
    }
  }, [isTranslated, originalContent, translateContent]);

  /**
   * Manually translates content from the page
   * @param {function} onContentUpdate - Callback to update content in parent component
   */
  const translatePageContent = useCallback(async (onContentUpdate) => {
    const contentToTranslate = capturePageContent();
    if (contentToTranslate) {
      try {
        const translated = await translateContent(contentToTranslate);
        onContentUpdate(translated);
      } catch (error) {
        console.error('Page translation failed:', error);
      }
    }
  }, [translateContent]);

  /**
   * Resets the translation state
   */
  const resetTranslation = useCallback(() => {
    setIsTranslated(false);
    setIsTranslating(false);
    setOriginalContent('');
    setTranslatedContent('');
  }, []);

  return {
    isTranslating,
    isTranslated,
    originalContent,
    translatedContent,
    translateContent,
    toggleTranslation,
    translatePageContent,
    resetTranslation
  };
};

export default useTranslation;