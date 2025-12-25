import React, { useState } from 'react';
import { translateText } from '../../api/translation'; // Path from src/components to root/api
import { capturePageContent, prepareContentForTranslation } from '../utils/contentCapture'; // Path from src/components to src/utils
import styles from './translationButton.module.css';

const TranslationButton = ({ pageContent, onPageUpdate }) => {
  const [isTranslating, setIsTranslating] = useState(false);
  const [isTranslated, setIsTranslated] = useState(false);
  const [originalContent, setOriginalContent] = useState(null);
  const [translationMode, setTranslationMode] = useState(null); // 'page' or 'selection'

  // Function to get selected text
  const getSelectedText = () => {
    const selection = window.getSelection();
    return selection?.toString().trim() || '';
  };

  const handleTranslate = async () => {
    setIsTranslating(true);
    try {
      // Check if text is selected
      const selectedText = getSelectedText();
      let contentToTranslate = '';
      let isSelectionMode = false;

      if (selectedText.length > 0) {
        // Selection mode: translate only selected text
        contentToTranslate = selectedText;
        isSelectionMode = true;
      } else {
        // Use provided page content or capture it dynamically
        contentToTranslate = pageContent || capturePageContent();
        isSelectionMode = false;
      }

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

      const response = await translateText(cleanedContent, 'en', 'ur', isSelectionMode ? 'selection' : 'page');
      onPageUpdate(response.translated_text);
      setOriginalContent(contentToTranslate); // Store original for toggling
      setTranslationMode(isSelectionMode ? 'selection' : 'page'); // Set translation mode
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
      // For the inline component, we just restore the original content
      // regardless of whether it was page or selection mode
      onPageUpdate(originalContent);
      setTranslationMode(null);
      setIsTranslated(false);
    } else {
      handleTranslate();
    }
  };

  return (
    <button
      onClick={handleToggle}
      disabled={isTranslating}
      className={styles.inlineTranslationButton}
    >
      {isTranslating ? (
        <>
          <i className="fas fa-spinner fa-spin"></i> Translating...
        </>
      ) : isTranslated ? (
        <>
          <i className="fas fa-language"></i> Show in English
        </>
      ) : (
        <>
          <i className="fas fa-language"></i> Translate to Urdu
        </>
      )}
    </button>
  );
};

export default TranslationButton;