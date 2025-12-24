import React, { useState } from 'react';
import { translateText } from '../../api/translation'; // Path from src/components to root/api
import { capturePageContent, prepareContentForTranslation } from '../utils/contentCapture'; // Path from src/components to src/utils

const TranslationButton = ({ pageContent, onPageUpdate }) => {
  const [isTranslating, setIsTranslating] = useState(false);
  const [isTranslated, setIsTranslated] = useState(false);
  const [originalContent, setOriginalContent] = useState(null);

  const handleTranslate = async () => {
    setIsTranslating(true);
    try {
      // If no pageContent was provided via prop, capture it dynamically
      const contentToTranslate = pageContent || capturePageContent();

      // Validate content before translation
      if (!contentToTranslate || contentToTranslate.trim().length === 0) {
        alert('No content found to translate');
        return;
      }

      // Check content length (similar to backend validation)
      if (contentToTranslate.length > 10000) {
        alert('Content is too long for translation (maximum 10,000 characters)');
        return;
      }

      const cleanedContent = prepareContentForTranslation(contentToTranslate);

      const response = await translateText(cleanedContent, 'en', 'ur');
      onPageUpdate(response.translated_text);
      setOriginalContent(contentToTranslate); // Store original for toggling
      setIsTranslated(true);
    } catch (error) {
      console.error('Translation failed:', error);
      alert('Translation failed: ' + error.message);
    } finally {
      setIsTranslating(false);
    }
  };

  const handleToggle = () => {
    if (isTranslated && originalContent) {
      onPageUpdate(originalContent);
      setIsTranslated(false);
    } else {
      handleTranslate();
    }
  };

  return (
    <button
      onClick={handleToggle}
      disabled={isTranslating}
      className="translation-button"
      style={{
        margin: '10px 0',
        padding: '8px 16px',
        backgroundColor: '#007cba',
        color: 'white',
        border: 'none',
        borderRadius: '4px',
        cursor: isTranslating ? 'not-allowed' : 'pointer',
        opacity: isTranslating ? 0.6 : 1
      }}
    >
      {isTranslating
        ? 'Translating...'
        : isTranslated
          ? 'Show in English'
          : 'Translate to Urdu'}
    </button>
  );
};

export default TranslationButton;