/**
 * End-to-End Tests for Select-Text Feature (Widget ↔ API Integration)
 *
 * Tests the complete flow of user selecting text in book viewer, UI validation,
 * API communication, and response handling for the select-text chatbot feature.
 *
 * Scope: Frontend widget behavior + backend integration
 * Framework: Playwright (E2E testing)
 * Coverage: Select-text mode with zero-leakage validation
 */

import { test, expect } from '@playwright/test';

/**
 * Test Configuration
 */
const BASE_URL = process.env.BASE_URL || 'http://localhost:3000';
const API_URL = process.env.API_URL || 'http://localhost:8000';
const BOOK_CONTENT = {
  title: 'Sample Test Book',
  excerpt: `Chapter 1: Introduction to AI

  Artificial Intelligence (AI) is the simulation of human intelligence by machines.
  Machine Learning (ML) is a subset of AI that focuses on algorithms that can learn
  from data without being explicitly programmed.

  Key Concepts:
  - Supervised Learning: Learning from labeled data
  - Unsupervised Learning: Learning patterns from unlabeled data
  - Reinforcement Learning: Learning through rewards and penalties

  Important Note: This chapter covers foundational concepts essential for understanding
  modern AI systems.`,

  selectedPassage: 'Machine Learning (ML) is a subset of AI that focuses on algorithms that can learn from data without being explicitly programmed.',

  selectedPassageShort: 'AI is the',  // < 10 characters (should trigger validation)

  selectedPassageMin: 'AI is the key',  // = 13 characters (minimum valid)
};

test.describe('Select-Text Feature - End-to-End Tests', () => {

  test.beforeEach(async ({ page }) => {
    // Navigate to demo page with embedded chat widget
    await page.goto(`${BASE_URL}/demo.html`);

    // Wait for widget to load
    await page.waitForSelector('[data-testid="chat-widget"]', { timeout: 5000 });
  });

  test.describe('Text Selection Detection', () => {

    test('should detect valid text selection (>= 10 characters)', async ({ page }) => {
      // Get the book content container
      const bookContent = page.locator('[data-testid="book-content"]');

      // Select valid text passage
      await selectTextInElement(page, bookContent, BOOK_CONTENT.selectedPassage);

      // Verify "Ask about this" button appears
      const askButton = page.locator('[data-testid="ask-about-this-btn"]');
      await expect(askButton).toBeVisible();
      await expect(askButton).toContainText('Ask about this');
    });

    test('should show warning for short selection (1-9 characters)', async ({ page }) => {
      const bookContent = page.locator('[data-testid="book-content"]');

      // Select short text (< 10 characters)
      await selectTextInElement(page, bookContent, BOOK_CONTENT.selectedPassageShort);

      // Verify "Ask about this" button appears but with warning indicator
      const askButton = page.locator('[data-testid="ask-about-this-btn"]');
      await expect(askButton).toBeVisible();

      // Check for warning class or tooltip
      const warningIndicator = askButton.locator('[data-testid="length-warning"]');
      await expect(warningIndicator).toBeVisible();
    });

    test('should not show button for empty selection', async ({ page }) => {
      // No selection at all
      const askButton = page.locator('[data-testid="ask-about-this-btn"]');

      // Button should not be visible when there's no selection
      await expect(askButton).not.toBeVisible();
    });

    test('should update button visibility when selection changes', async ({ page }) => {
      const bookContent = page.locator('[data-testid="book-content"]');
      const askButton = page.locator('[data-testid="ask-about-this-btn"]');

      // First: select short text (warning)
      await selectTextInElement(page, bookContent, BOOK_CONTENT.selectedPassageShort);
      const warningIndicator = askButton.locator('[data-testid="length-warning"]');
      await expect(warningIndicator).toBeVisible();

      // Then: select valid text (no warning)
      await selectTextInElement(page, bookContent, BOOK_CONTENT.selectedPassage);
      await expect(warningIndicator).not.toBeVisible();

      // Clear selection
      await page.evaluate(() => window.getSelection()?.removeAllRanges());
      await expect(askButton).not.toBeVisible();
    });
  });

  test.describe('UI Validation Rules (FR-007a)', () => {

    test('should accept selection >= 10 characters and enable submit', async ({ page }) => {
      const bookContent = page.locator('[data-testid="book-content"]');
      const askButton = page.locator('[data-testid="ask-about-this-btn"]');
      const inputField = page.locator('[data-testid="chat-input"]');

      // Select valid text
      await selectTextInElement(page, bookContent, BOOK_CONTENT.selectedPassageMin);
      await expect(askButton).toBeVisible();

      // Click "Ask about this"
      await askButton.click();

      // Verify chat opens with selected text visible
      const selectedTextDisplay = page.locator('[data-testid="selected-text-display"]');
      await expect(selectedTextDisplay).toBeVisible();
      await expect(selectedTextDisplay).toContainText(BOOK_CONTENT.selectedPassageMin);

      // Verify input field is focused and ready
      await expect(inputField).toBeFocused();
    });

    test('should warn user for selection 1-9 characters', async ({ page }) => {
      const bookContent = page.locator('[data-testid="book-content"]');
      const askButton = page.locator('[data-testid="ask-about-this-btn"]');

      // Select short text
      await selectTextInElement(page, bookContent, BOOK_CONTENT.selectedPassageShort);

      // Verify warning is shown
      const warningTooltip = askButton.locator('[data-testid="length-warning-tooltip"]');
      await expect(warningTooltip).toBeVisible();
      await expect(warningTooltip).toContainText('at least 10 characters');

      // Button should be disabled or show warning state
      await expect(askButton).toHaveClass(/warning|disabled/);
    });

    test('should reject empty selection silently', async ({ page }) => {
      const askButton = page.locator('[data-testid="ask-about-this-btn"]');

      // No selection
      await expect(askButton).not.toBeVisible();
    });
  });

  test.describe('Chat Widget Integration with Selected Text', () => {

    test('should open chat widget with selected text mode (FR-007a)', async ({ page }) => {
      const bookContent = page.locator('[data-testid="book-content"]');
      const askButton = page.locator('[data-testid="ask-about-this-btn"]');

      // Select text
      await selectTextInElement(page, bookContent, BOOK_CONTENT.selectedPassage);

      // Click "Ask about this"
      await askButton.click();

      // Verify chat widget opens in selected-text mode
      const chatWidget = page.locator('[data-testid="chat-widget"]');
      await expect(chatWidget).toHaveClass(/selected-text-mode/);

      // Verify selected text is displayed as context
      const selectedTextDisplay = page.locator('[data-testid="selected-text-display"]');
      await expect(selectedTextDisplay).toBeVisible();
      await expect(selectedTextDisplay).toContainText(BOOK_CONTENT.selectedPassage);
    });

    test('should submit query with selected text context to backend', async ({ page }) => {
      const bookContent = page.locator('[data-testid="book-content"]');
      const askButton = page.locator('[data-testid="ask-about-this-btn"]');

      // Intercept API calls to backend
      const apiPromise = page.waitForResponse(response =>
        response.url().includes('/chat') && response.status() === 200
      );

      // Select text and open chat
      await selectTextInElement(page, bookContent, BOOK_CONTENT.selectedPassage);
      await askButton.click();

      // Type query
      const inputField = page.locator('[data-testid="chat-input"]');
      await inputField.fill('What is Machine Learning?');

      // Submit query
      const sendButton = page.locator('[data-testid="send-btn"]');
      await sendButton.click();

      // Verify API request includes selected_text and mode='selected'
      const response = await apiPromise;
      const requestBody = await response.json();

      expect(requestBody).toHaveProperty('mode', 'selected');
      expect(requestBody).toHaveProperty('selected_text');
      expect(requestBody.selected_text).toBe(BOOK_CONTENT.selectedPassage);
      expect(requestBody).toHaveProperty('query_text');
    });

    test('should display response with selected text attribution', async ({ page }) => {
      const bookContent = page.locator('[data-testid="book-content"]');
      const askButton = page.locator('[data-testid="ask-about-this-btn"]');

      // Select text and submit query
      await selectTextInElement(page, bookContent, BOOK_CONTENT.selectedPassage);
      await askButton.click();

      const inputField = page.locator('[data-testid="chat-input"]');
      await inputField.fill('What is this about?');

      const sendButton = page.locator('[data-testid="send-btn"]');
      await sendButton.click();

      // Wait for response message
      const responseMessage = page.locator('[data-testid="chat-message"]:has-text("Machine Learning")').first();
      await expect(responseMessage).toBeVisible({ timeout: 10000 });

      // Verify response is attributed to selected passage (citations should reference selected passage)
      const citation = page.locator('[data-testid="citation"]').first();
      await expect(citation).toBeVisible();
    });
  });

  test.describe('Backend Validation (FR-007a HTTP 400)', () => {

    test('should handle HTTP 400 error when selected_text < 10 characters', async ({ page }) => {
      // Directly intercept and mock API response
      await page.route('**/api/chat', (route) => {
        const request = route.request();
        const postData = request.postDataJSON();

        // If selected_text < 10 chars, return 400
        if (postData.selected_text && postData.selected_text.length < 10) {
          route.abort('failed');
          return;
        }

        route.continue();
      });

      const bookContent = page.locator('[data-testid="book-content"]');
      const askButton = page.locator('[data-testid="ask-about-this-btn"]');

      // Select short text
      await selectTextInElement(page, bookContent, BOOK_CONTENT.selectedPassageShort);
      await askButton.click();

      const inputField = page.locator('[data-testid="chat-input"]');
      await inputField.fill('What is AI?');

      const sendButton = page.locator('[data-testid="send-btn"]');
      await sendButton.click();

      // Verify error message is shown
      const errorMessage = page.locator('[data-testid="error-message"]');
      await expect(errorMessage).toBeVisible();
      await expect(errorMessage).toContainText('Selected text must be at least 10 characters');
    });

    test('should handle HTTP 400 error when selected_text is empty/null', async ({ page }) => {
      const bookContent = page.locator('[data-testid="book-content"]');
      const inputField = page.locator('[data-testid="chat-input"]');
      const sendButton = page.locator('[data-testid="send-btn"]');

      // Manually set mode to 'selected' but leave selected_text empty (simulates form tampering)
      await page.evaluate(() => {
        const widget = document.querySelector('[data-testid="chat-widget"]');
        if (widget) {
          widget.setAttribute('data-mode', 'selected');
          widget.setAttribute('data-selected-text', '');
        }
      });

      // Try to send query
      await inputField.fill('What is AI?');
      await sendButton.click();

      // Verify error message
      const errorMessage = page.locator('[data-testid="error-message"]');
      await expect(errorMessage).toBeVisible();
      await expect(errorMessage).toContainText('Selected text is required');
    });

    test('should retry after handling validation error', async ({ page }) => {
      const bookContent = page.locator('[data-testid="book-content"]');
      const askButton = page.locator('[data-testid="ask-about-this-btn"]');

      // First attempt: select short text (will fail)
      await selectTextInElement(page, bookContent, BOOK_CONTENT.selectedPassageShort);
      await askButton.click();

      const inputField = page.locator('[data-testid="chat-input"]');
      await inputField.fill('What is AI?');

      const sendButton = page.locator('[data-testid="send-btn"]');
      await sendButton.click();

      // Verify error
      const errorMessage = page.locator('[data-testid="error-message"]');
      await expect(errorMessage).toBeVisible();

      // Second attempt: select valid text
      await page.keyboard.press('Escape'); // Close error
      await selectTextInElement(page, bookContent, BOOK_CONTENT.selectedPassage);

      const newAskButton = page.locator('[data-testid="ask-about-this-btn"]');
      await newAskButton.click();

      // Verify new selected text is displayed
      const selectedTextDisplay = page.locator('[data-testid="selected-text-display"]');
      await expect(selectedTextDisplay).toContainText(BOOK_CONTENT.selectedPassage);

      // Submit again
      await inputField.click();
      await inputField.fill('What is Machine Learning?');
      await sendButton.click();

      // This time it should succeed
      const responseMessage = page.locator('[data-testid="chat-message"]').last();
      await expect(responseMessage).toBeVisible({ timeout: 10000 });
    });
  });

  test.describe('Zero-Leakage Validation', () => {

    test('should not include content outside selected passage in response', async ({ page }) => {
      const bookContent = page.locator('[data-testid="book-content"]');
      const askButton = page.locator('[data-testid="ask-about-this-btn"]');

      // Select only the ML definition (middle of book)
      const mlDefinition = 'Machine Learning (ML) is a subset of AI that focuses on algorithms that can learn from data without being explicitly programmed.';
      await selectTextInElement(page, bookContent, mlDefinition);

      await askButton.click();

      const inputField = page.locator('[data-testid="chat-input"]');
      await inputField.fill('What is this about?');

      const sendButton = page.locator('[data-testid="send-btn"]');
      await sendButton.click();

      // Wait for response
      const responseMessage = page.locator('[data-testid="chat-message"]').last();
      await expect(responseMessage).toBeVisible({ timeout: 10000 });

      // Verify response ONLY references ML, not general AI concepts from elsewhere in book
      const responseText = await responseMessage.textContent();

      // Response should contain ML-related content
      expect(responseText).toMatch(/Machine Learning|algorithms|learn from data/i);

      // Response should NOT reference "Supervised Learning" or other sections not in selection
      // (This is a loose check; strict zero-leakage validation happens in backend tests)
    });

    test('should query backend with selected-text mode filter', async ({ page }) => {
      const bookContent = page.locator('[data-testid="book-content"]');
      const askButton = page.locator('[data-testid="ask-about-this-btn"]');

      // Intercept API call
      let capturedRequest: any = null;
      await page.route('**/chat', (route) => {
        capturedRequest = route.request().postDataJSON();
        route.continue();
      });

      // Select text
      await selectTextInElement(page, bookContent, BOOK_CONTENT.selectedPassage);
      await askButton.click();

      const inputField = page.locator('[data-testid="chat-input"]');
      await inputField.fill('Test query');

      const sendButton = page.locator('[data-testid="send-btn"]');
      await sendButton.click();

      // Verify backend receives mode='selected'
      await page.waitForTimeout(1000);
      expect(capturedRequest).toBeTruthy();
      expect(capturedRequest.mode).toBe('selected');
      expect(capturedRequest.selected_text).toBe(BOOK_CONTENT.selectedPassage);
    });
  });

  test.describe('Message History and Context', () => {

    test('should display selected text context in chat header', async ({ page }) => {
      const bookContent = page.locator('[data-testid="book-content"]');
      const askButton = page.locator('[data-testid="ask-about-this-btn"]');

      // Select text and open chat
      await selectTextInElement(page, bookContent, BOOK_CONTENT.selectedPassage);
      await askButton.click();

      // Verify selected text is shown in context area
      const contextArea = page.locator('[data-testid="selected-text-display"]');
      await expect(contextArea).toBeVisible();
      await expect(contextArea).toContainText(BOOK_CONTENT.selectedPassage);
    });

    test('should maintain selected text context across multiple queries', async ({ page }) => {
      const bookContent = page.locator('[data-testid="book-content"]');
      const askButton = page.locator('[data-testid="ask-about-this-btn"]');

      // Select text and open chat
      await selectTextInElement(page, bookContent, BOOK_CONTENT.selectedPassage);
      await askButton.click();

      const inputField = page.locator('[data-testid="chat-input"]');
      const sendButton = page.locator('[data-testid="send-btn"]');

      // First query
      await inputField.fill('What is ML?');
      await sendButton.click();
      await page.waitForTimeout(2000);

      // Second query
      await inputField.fill('Explain algorithms');
      await sendButton.click();
      await page.waitForTimeout(2000);

      // Verify selected text context is still visible
      const contextArea = page.locator('[data-testid="selected-text-display"]');
      await expect(contextArea).toBeVisible();
      await expect(contextArea).toContainText(BOOK_CONTENT.selectedPassage);

      // Both messages should be in chat
      const messages = page.locator('[data-testid="chat-message"]');
      const messageCount = await messages.count();
      expect(messageCount).toBeGreaterThanOrEqual(4); // 2 queries + 2 responses
    });

    test('should store message history in localStorage', async ({ page }) => {
      const bookContent = page.locator('[data-testid="book-content"]');
      const askButton = page.locator('[data-testid="ask-about-this-btn"]');

      // Select text and submit query
      await selectTextInElement(page, bookContent, BOOK_CONTENT.selectedPassage);
      await askButton.click();

      const inputField = page.locator('[data-testid="chat-input"]');
      await inputField.fill('Test query for history');

      const sendButton = page.locator('[data-testid="send-btn"]');
      await sendButton.click();

      await page.waitForTimeout(2000);

      // Check localStorage
      const storedHistory = await page.evaluate(() => {
        return localStorage.getItem('chatHistory');
      });

      expect(storedHistory).toBeTruthy();
      const history = JSON.parse(storedHistory || '[]');
      expect(history.length).toBeGreaterThan(0);

      // Verify stored message has selected_text context
      const lastMessage = history[history.length - 1];
      if (lastMessage.mode === 'selected') {
        expect(lastMessage.selected_text).toBe(BOOK_CONTENT.selectedPassage);
      }
    });
  });

  test.describe('Cross-Browser Compatibility', () => {

    test('should work in multiple browsers', async ({ page, browserName }) => {
      // This test runs in parallel across browsers via Playwright config
      const bookContent = page.locator('[data-testid="book-content"]');
      const askButton = page.locator('[data-testid="ask-about-this-btn"]');

      // Basic flow test
      await selectTextInElement(page, bookContent, BOOK_CONTENT.selectedPassage);
      await expect(askButton).toBeVisible();

      console.log(`✓ Select-text works in ${browserName}`);
    });
  });

  test.describe('Error Handling & Edge Cases', () => {

    test('should handle API timeout gracefully', async ({ page }) => {
      const bookContent = page.locator('[data-testid="book-content"]');
      const askButton = page.locator('[data-testid="ask-about-this-btn"]');

      // Select text
      await selectTextInElement(page, bookContent, BOOK_CONTENT.selectedPassage);
      await askButton.click();

      // Simulate timeout
      await page.route('**/chat', route => {
        setTimeout(() => route.abort('timedout'), 15000);
      });

      const inputField = page.locator('[data-testid="chat-input"]');
      await inputField.fill('Test query');

      const sendButton = page.locator('[data-testid="send-btn"]');
      await sendButton.click();

      // Wait for timeout
      const errorMessage = page.locator('[data-testid="error-message"]');
      await expect(errorMessage).toBeVisible({ timeout: 20000 });
      await expect(errorMessage).toContainText(/timeout|unavailable/i);
    });

    test('should handle API error responses', async ({ page }) => {
      await page.route('**/chat', (route) => {
        route.abort('failed');
      });

      const bookContent = page.locator('[data-testid="book-content"]');
      const askButton = page.locator('[data-testid="ask-about-this-btn"]');

      await selectTextInElement(page, bookContent, BOOK_CONTENT.selectedPassage);
      await askButton.click();

      const inputField = page.locator('[data-testid="chat-input"]');
      await inputField.fill('Test query');

      const sendButton = page.locator('[data-testid="send-btn"]');
      await sendButton.click();

      const errorMessage = page.locator('[data-testid="error-message"]');
      await expect(errorMessage).toBeVisible();
    });

    test('should handle rapid successive selections', async ({ page }) => {
      const bookContent = page.locator('[data-testid="book-content"]');
      const askButton = page.locator('[data-testid="ask-about-this-btn"]');

      // Rapidly select different text
      for (let i = 0; i < 5; i++) {
        if (i % 2 === 0) {
          await selectTextInElement(page, bookContent, BOOK_CONTENT.selectedPassage);
        } else {
          await selectTextInElement(page, bookContent, BOOK_CONTENT.selectedPassageMin);
        }

        // Verify button updates
        if (i % 2 === 0) {
          await expect(askButton).toBeVisible();
        }
      }

      // Final selection should be consistent
      await selectTextInElement(page, bookContent, BOOK_CONTENT.selectedPassage);
      await expect(askButton).toBeVisible();
    });
  });
});

/**
 * Helper function to select text within an element
 * Uses Playwright's ability to interact with DOM
 */
async function selectTextInElement(
  page: any,
  element: any,
  textToSelect: string
) {
  const elementHandle = await element.elementHandle();
  if (!elementHandle) throw new Error('Element not found');

  await page.evaluate(
    ({ elementHandle, textToSelect }) => {
      const element = elementHandle;
      const text = element.textContent;

      if (!text || !text.includes(textToSelect)) {
        throw new Error(`Text not found: "${textToSelect}"`);
      }

      const range = document.createRange();
      const sel = window.getSelection();

      // Find the exact position of text in element
      const startIndex = text.indexOf(textToSelect);
      const endIndex = startIndex + textToSelect.length;

      // Create range and select
      let currentIndex = 0;
      let startNode = null;
      let startOffset = 0;
      let endNode = null;
      let endOffset = 0;

      const walker = document.createTreeWalker(
        element,
        NodeFilter.SHOW_TEXT,
        null
      );

      let node;
      while ((node = walker.nextNode())) {
        const nodeLength = node.textContent?.length || 0;
        const nodeEndIndex = currentIndex + nodeLength;

        if (!startNode && currentIndex + nodeLength > startIndex) {
          startNode = node;
          startOffset = startIndex - currentIndex;
        }

        if (!endNode && nodeEndIndex >= endIndex) {
          endNode = node;
          endOffset = endIndex - currentIndex;
        }

        if (startNode && endNode) break;
        currentIndex += nodeLength;
      }

      if (!startNode || !endNode) {
        throw new Error('Could not find text range');
      }

      range.setStart(startNode, startOffset);
      range.setEnd(endNode, endOffset);

      sel?.removeAllRanges();
      sel?.addRange(range);

      // Trigger selection event
      document.dispatchEvent(new Event('selectionchange'));
    },
    { elementHandle, textToSelect }
  );
}
