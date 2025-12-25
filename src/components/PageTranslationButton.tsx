import React, { useState, useEffect } from 'react';
import { translateText } from '../../api/translation'; // Path from src/components to root/api
import { capturePageContent, prepareContentForTranslation } from '../utils/contentCapture'; // Path from src/components to src/utils
import styles from './translationButton.module.css';

const PageTranslationButton = () => {
  const [isTranslating, setIsTranslating] = useState(false);
  const [isTranslated, setIsTranslated] = useState(false);
  const [originalContent, setOriginalContent] = useState(null);
  const [originalSelection, setOriginalSelection] = useState(null);
  const [translationMode, setTranslationMode] = useState(null); // 'page' or 'selection'
  const [translationButtonStyle, setTranslationButtonStyle] = useState({});

  // Function to get selected text
  const getSelectedText = () => {
    const selection = window.getSelection();
    return selection?.toString().trim() || '';
  };

  // Function to get selection range for restoration
  const getSelectionRange = () => {
    const selection = window.getSelection();
    if (selection.rangeCount > 0) {
      return selection.getRangeAt(0);
    }
    return null;
  };

  // Function to restore selection
  const restoreSelection = (range) => {
    if (range) {
      const selection = window.getSelection();
      selection.removeAllRanges();
      selection.addRange(range);
    }
  };

  // Function to update page content or selected text
  const updateContent = (newContent, isSelectionMode = false) => {
    if (isSelectionMode) {
      // If in selection mode, replace the selected text
      const selection = window.getSelection();
      if (selection && selection.rangeCount > 0) {
        const range = selection.getRangeAt(0);
        range.deleteContents();
        const textNode = document.createTextNode(newContent);
        range.insertNode(textNode);
      }
    } else {
      // Find the main content area and update it (page mode)
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

  // Function to restore original selection
  const restoreOriginalSelection = () => {
    if (originalSelection) {
      const selection = window.getSelection();
      selection.removeAllRanges();
      selection.addRange(originalSelection);
    }
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

        // Save the selection range to restore later
        const range = getSelectionRange();
        if (range) {
          // Clone the range to save it
          const clonedRange = range.cloneRange();
          setOriginalSelection(clonedRange);
        }
      } else {
        // Page mode: translate entire page content
        contentToTranslate = capturePageContent();
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

      // Update content based on mode
      updateContent(response.translated_text, isSelectionMode);

      // Store original content for toggling if in page mode
      if (!isSelectionMode) {
        setOriginalContent(contentToTranslate);
      }

      // Set the translation mode
      setTranslationMode(isSelectionMode ? 'selection' : 'page');

      setIsTranslated(true);
    } catch (error) {
      console.error('Translation failed:', error);
      alert('Translation failed: ' + error.message);
    } finally {
      setIsTranslating(false);
    }
  };

  const handleToggle = () => {
    if (isTranslated) {
      // Restore original content based on the translation mode
      if (translationMode === 'selection') {
        // If we were in selection mode, restore the selection
        restoreOriginalSelection();
      } else {
        // Otherwise, we were in page mode
        restoreOriginalContent();
      }
      // Clear the saved state
      setOriginalSelection(null);
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
      className={styles.translationButton}
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

export default PageTranslationButton;