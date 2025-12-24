/**
 * Utility functions for capturing page content dynamically
 */

/**
 * Captures the main content of a Docusaurus page by targeting the main content area
 * @returns {string} The text content of the main page area
 */
export const capturePageContent = () => {
  // Try multiple selectors to find the main content area
  const selectors = [
    '.main-wrapper',           // Common Docusaurus wrapper
    'article',                 // Standard article tag
    '.markdown',               // Docusaurus markdown container
    '.theme-doc-markdown',     // Docusaurus theme markdown
    '.container',              // General container
    '.doc-content',            // Docusaurus doc content
    'main',                    // Main content area
    '.main',                   // Alternative main class
    '.content',                // General content class
    '.docItemContainer',       // Docusaurus doc item container
    '[data-testid="doc-markdown"]' // Docusaurus test ID
  ];

  for (const selector of selectors) {
    const element = document.querySelector(selector);
    if (element) {
      // Extract text content, preserving paragraph structure
      return element.textContent || element.innerText || '';
    }
  }

  // Fallback: get content from the body
  return document.body ? document.body.textContent || document.body.innerText : '';
};

/**
 * Captures content from a specific element
 * @param {HTMLElement} element - The element to capture content from
 * @returns {string} The text content of the element
 */
export const captureElementContent = (element) => {
  if (!element) return '';

  // Preserve structure by getting inner text
  return element.textContent || element.innerText || '';
};

/**
 * Gets content from a specific selector
 * @param {string} selector - CSS selector for the content element
 * @returns {string} The text content of the selected element
 */
export const captureContentBySelector = (selector) => {
  const element = document.querySelector(selector);
  return captureElementContent(element);
};

/**
 * Cleans and prepares content for translation
 * @param {string} content - Raw content to clean
 * @returns {string} Cleaned content ready for translation
 */
export const prepareContentForTranslation = (content) => {
  if (!content) return '';

  // Remove extra whitespace and normalize line breaks
  return content
    .replace(/\s+/g, ' ')           // Replace multiple spaces with single space
    .replace(/\n\s*\n/g, '\n\n')    // Normalize paragraph breaks
    .trim();                        // Remove leading/trailing whitespace
};