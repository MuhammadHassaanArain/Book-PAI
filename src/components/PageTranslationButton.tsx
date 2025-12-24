import React, { useState, useEffect } from 'react';
import { translateText } from '../../api/translation'; // Path from src/components to root/api
import { capturePageContent, prepareContentForTranslation } from '../utils/contentCapture'; // Path from src/components to src/utils

const PageTranslationButton = () => {
  const [isTranslating, setIsTranslating] = useState(false);
  const [isTranslated, setIsTranslated] = useState(false);
  const [originalContent, setOriginalContent] = useState(null);
  const [translationButtonStyle, setTranslationButtonStyle] = useState({});

  // Function to update page content
  const updatePageContent = (newContent) => {
    // Find the main content area and update it
    const mainContent = document.querySelector('main div[class*="markdown"], main article, .markdown, .doc-content, .theme-doc-markdown');
    if (mainContent && newContent) {
      // Store original content if not already stored
      if (!originalContent && !isTranslated) {
        setOriginalContent(mainContent.innerHTML);
      }

      // Update the content
      mainContent.innerHTML = newContent;
    } else {
      // Fallback: try to find content by other selectors
      const contentSelectors = [
        '[data-testid="doc-content"]',
        '.docItemContainer',
        'article',
        '.container',
        'main'
      ];

      for (const selector of contentSelectors) {
        const element = document.querySelector(selector);
        if (element) {
          if (!originalContent && !isTranslated) {
            setOriginalContent(element.innerHTML);
          }
          element.innerHTML = newContent;
          break;
        }
      }
    }
  };

  // Function to restore original content
  const restoreOriginalContent = () => {
    if (originalContent) {
      const mainContent = document.querySelector('main div[class*="markdown"], main article, .markdown, .doc-content, .theme-doc-markdown');
      if (mainContent) {
        mainContent.innerHTML = originalContent;
      } else {
        const contentSelectors = [
          '[data-testid="doc-content"]',
          '.docItemContainer',
          'article',
          '.container',
          'main'
        ];

        for (const selector of contentSelectors) {
          const element = document.querySelector(selector);
          if (element) {
            element.innerHTML = originalContent;
            break;
          }
        }
      }
    }
  };

  const handleTranslate = async () => {
    setIsTranslating(true);
    try {
      // Capture current page content
      const contentToTranslate = capturePageContent();

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
      updatePageContent(response.translated_text);
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
      restoreOriginalContent();
      setIsTranslated(false);
    } else {
      handleTranslate();
    }
  };

  // Add some basic styling for the button
  useEffect(() => {
    // Create a style element for the button if it doesn't exist
    let styleElement = document.getElementById('translation-button-style');
    if (!styleElement) {
      styleElement = document.createElement('style');
      styleElement.id = 'translation-button-style';
      styleElement.textContent = `
        .translation-button {
          position: fixed;
          top: 20px;
          right: 20px;
          z-index: 10000;
          margin: 10px 0;
          padding: 8px 16px;
          background-color: #007cba;
          color: white;
          border: none;
          border-radius: 4px;
          cursor: pointer;
          opacity: 0.9;
          font-size: 14px;
          box-shadow: 0 2px 10px rgba(0,0,0,0.2);
        }
        .translation-button:hover {
          opacity: 1;
          background-color: #005a87;
        }
        .translation-button:disabled {
          cursor: not-allowed;
          opacity: 0.6;
        }
      `;
      document.head.appendChild(styleElement);
    }
  }, []);

  return (
    <button
      onClick={handleToggle}
      disabled={isTranslating}
      className="translation-button"
    >
      {isTranslating
        ? 'Translating...'
        : isTranslated
          ? 'Show in English'
          : 'Translate to Urdu'}
    </button>
  );
};

export default PageTranslationButton;