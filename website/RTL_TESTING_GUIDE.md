# RTL (Right-to-Left) Testing Guide for Urdu Language Support

## Overview

This guide provides comprehensive instructions for testing the Physical AI Textbook website with Urdu (اردو) language support. Urdu is a Right-to-Left (RTL) language that requires special attention to layout, text direction, spacing, and UI component positioning.

**Language Configuration:**
- Locale: `ur` (Urdu)
- Direction: `rtl` (Right-to-Left)
- HTML Lang: `ur-PK` (Urdu - Pakistan)

## Quick Start

### Running the Site in Urdu Mode

```bash
# Navigate to the website directory
cd D:\physical-ai-textbook\website

# Start the development server with Urdu locale
npm run start -- --locale ur

# Or build the Urdu version
npm run build -- --locale ur
npm run serve -- --locale ur
```

**Access URLs:**
- Default (English): `http://localhost:3000/`
- Urdu: `http://localhost:3000/ur/`

## Step-by-Step Testing Instructions

### 1. Initial Setup

1. **Start the development server:**
   ```bash
   npm run start
   ```

2. **Navigate to Urdu version:**
   - Go to `http://localhost:3000/`
   - Use the language switcher in the navbar (if available)
   - Or directly access `http://localhost:3000/ur/`

3. **Verify RTL is active:**
   - Open browser DevTools (F12)
   - Check the `<html>` tag: should have `dir="rtl"` and `lang="ur-PK"`
   - Check computed styles: `direction: rtl` should be applied

### 2. Testing Checklist by Component

#### A. Homepage (`/` or `/ur/`)

**Hero Section:**
- [ ] Title text flows right-to-left
- [ ] Subtitle text flows right-to-left
- [ ] CTA button ("Start Learning") positioned correctly
- [ ] Button icon (if any) on the left side of text in RTL
- [ ] Proper spacing around hero content

**Chapters Showcase:**
- [ ] Chapter cards layout mirrors properly
- [ ] Card titles in Urdu align right
- [ ] Card descriptions align right
- [ ] Icons/images positioned on right side
- [ ] Grid layout flows RTL (cards appear from right to left)
- [ ] Hover effects work correctly

**General Layout:**
- [ ] Overall page flow is RTL
- [ ] Margins and padding mirror correctly
- [ ] Scroll direction is natural (scrollbar on left)

#### B. Navbar

**Logo and Branding:**
- [ ] Logo positioned on the right (mirrored from LTR)
- [ ] Site title appears to the left of logo
- [ ] Proper spacing between logo and title

**Navigation Links:**
- [ ] Links flow from right to left
- [ ] "Textbook" link appears after logo
- [ ] "Blog" link positioned correctly
- [ ] Dropdown menus (if any) open to the left

**Right Section (Auth Buttons):**
- [ ] Language switcher displays on left side
- [ ] "Sign In" button on left
- [ ] "Sign Up" button on left
- [ ] GitHub link positioned on left
- [ ] Proper spacing between buttons

**Mobile Navbar:**
- [ ] Hamburger menu icon on left side
- [ ] Slide-out menu animates from left
- [ ] Menu items align right
- [ ] Close button positioned correctly

#### C. Authentication Pages

**Sign In Page (`/ur/signin`):**
- [ ] Page title "Welcome Back" aligns right
- [ ] Subtitle aligns right
- [ ] Form labels align right
- [ ] Input fields: text input flows RTL
- [ ] Input fields: placeholder text aligns right
- [ ] Error messages below fields align right
- [ ] "Sign In" button centered or right-aligned
- [ ] "Don't have an account?" text aligns right
- [ ] "Sign Up" link positioned correctly

**Sign Up Page (`/ur/signup`):**
- [ ] All form sections align right
- [ ] Multi-step form progress indicator mirrors
- [ ] Form field groups maintain RTL flow
- [ ] Radio buttons/checkboxes positioned on right
- [ ] Labels appear to left of input elements
- [ ] Submit button positioned correctly
- [ ] Navigation between steps feels natural

**Common Form Elements:**
- [ ] Required asterisk (*) appears on left of label
- [ ] Error icons positioned on right
- [ ] Success icons positioned on right
- [ ] Input focus states work correctly
- [ ] Tab order follows RTL flow

#### D. ChatBot Component

**Floating Button:**
- [ ] Chat button positioned on bottom-left (mirrored from bottom-right)
- [ ] Icon orientation correct
- [ ] Hover effects work properly

**Chat Window:**
- [ ] Window slides in from left
- [ ] Header "Physical AI Assistant" aligns right
- [ ] "Clear" button positioned on left side of header
- [ ] Close button (×) on left

**Messages:**
- [ ] User messages align to left (mirrored)
- [ ] Assistant messages align to right (mirrored)
- [ ] Message bubbles have correct tail direction
- [ ] Timestamps align with message direction
- [ ] Text within messages flows RTL
- [ ] "You:" and "Assistant:" labels correct

**Citations:**
- [ ] "Sources:" heading aligns right
- [ ] Citation links align right
- [ ] Relevance scores positioned correctly
- [ ] Citation hover states work

**Input Area:**
- [ ] Textarea text flows RTL
- [ ] Placeholder aligns right
- [ ] Send button (➤) positioned on left side
- [ ] Button icon points in correct direction
- [ ] Character counter (if any) on left

#### E. Footer

**Footer Layout:**
- [ ] Footer sections flow RTL
- [ ] Column headings align right
- [ ] Link lists align right
- [ ] Social media icons mirror (rightmost becomes leftmost)

**Footer Links:**
- [ ] Link hover states work
- [ ] External link icons positioned on left of text
- [ ] Copyright text aligns right
- [ ] "Built with Docusaurus" aligns right

#### F. Documentation Pages

**Sidebar:**
- [ ] Sidebar positioned on left side (mirrored)
- [ ] Collapse/expand icons point correctly
- [ ] Category headings align right
- [ ] Indentation mirrors (sub-items indented from left)
- [ ] Active page indicator on right edge

**Content Area:**
- [ ] Markdown content flows RTL
- [ ] Headings align right
- [ ] Paragraphs align right
- [ ] Lists (ul/ol) align right with bullets/numbers on right
- [ ] Code blocks align left (monospace code should remain LTR)
- [ ] Tables mirror properly
- [ ] Images and captions align right
- [ ] Blockquotes have border on right side

**Navigation:**
- [ ] Previous/Next buttons mirrored
- [ ] Breadcrumbs flow RTL
- [ ] "Edit this page" link aligned right

### 3. Common RTL Issues to Watch For

#### Layout Issues
- ❌ **Asymmetric margins/padding:** Elements have different spacing on left vs. right
- ❌ **Absolute positioning:** Elements positioned using `left` instead of `inline-start`
- ❌ **Fixed widths:** Components that break when text expands in RTL
- ❌ **Hardcoded directions:** CSS using `margin-left`, `padding-right` instead of logical properties

#### Text Issues
- ❌ **Mixed direction text:** English words in Urdu sentences breaking flow
- ❌ **Punctuation:** Commas, periods appearing on wrong side
- ❌ **Numbers:** Numbers not maintaining LTR within RTL text
- ❌ **Text truncation:** Ellipsis (...) appearing on wrong side

#### Icon & Image Issues
- ❌ **Directional icons:** Arrows, chevrons pointing wrong way
- ❌ **Icon positioning:** Icons stuck on wrong side of text
- ❌ **Image alignment:** Images not mirroring with layout
- ❌ **Logos:** Logos that should remain LTR getting flipped

#### Interactive Elements
- ❌ **Dropdown menus:** Opening to wrong side
- ❌ **Tooltips:** Appearing on wrong side of trigger
- ❌ **Modals:** Close buttons on wrong corner
- ❌ **Scroll direction:** Horizontal scrolling feels unnatural

#### Animation Issues
- ❌ **Slide transitions:** Sliding from wrong direction
- ❌ **Transform origins:** Animations pivoting from wrong point
- ❌ **Progress indicators:** Moving in wrong direction

## CSS Fixes and Overrides

### Modern CSS Logical Properties

Replace directional properties with logical equivalents:

| Old (Physical) | New (Logical) | Description |
|----------------|---------------|-------------|
| `margin-left` | `margin-inline-start` | Start of inline direction |
| `margin-right` | `margin-inline-end` | End of inline direction |
| `padding-left` | `padding-inline-start` | Start padding |
| `padding-right` | `padding-inline-end` | End padding |
| `left` | `inset-inline-start` | Position from start |
| `right` | `inset-inline-end` | Position from end |
| `border-left` | `border-inline-start` | Start border |
| `border-right` | `border-inline-end` | End border |
| `text-align: left` | `text-align: start` | Align to start |
| `text-align: right` | `text-align: end` | Align to end |

### RTL-Specific Overrides

See `D:\physical-ai-textbook\website\src\css\rtl-overrides.css` for comprehensive overrides.

**Key patterns:**

```css
/* Detect RTL context */
[dir="rtl"] .element {
  /* RTL-specific styles */
}

/* Flip directional icons */
[dir="rtl"] .icon-arrow {
  transform: scaleX(-1);
}

/* Use logical properties */
.card {
  margin-inline-start: 1rem; /* Auto-adjusts for RTL */
  padding-inline-end: 2rem;
}
```

### Integration with Docusaurus

Add RTL CSS to `docusaurus.config.ts`:

```typescript
theme: {
  customCss: [
    './src/css/custom.css',
    './src/css/rtl-overrides.css', // Add RTL overrides
  ],
},
```

## Browser Compatibility

### Supported Browsers

| Browser | Version | RTL Support | Logical Properties |
|---------|---------|-------------|-------------------|
| Chrome | 89+ | Excellent | Full |
| Firefox | 88+ | Excellent | Full |
| Safari | 14.1+ | Excellent | Full |
| Edge | 89+ | Excellent | Full |
| Opera | 75+ | Excellent | Full |

### Testing Matrix

Test on at least:
1. **Chrome (Latest)** - Most common browser
2. **Firefox (Latest)** - Good RTL standards compliance
3. **Safari (Latest)** - macOS/iOS users
4. **Mobile Safari (iOS)** - Mobile RTL testing
5. **Chrome Mobile (Android)** - Mobile RTL testing

### Known Browser Issues

**Safari < 14.1:**
- Limited support for logical properties
- May need fallback values

**Old Edge (< 89):**
- No logical property support
- Use RTL-specific selectors with physical properties

## Accessibility Considerations for RTL

### Screen Readers

- [ ] **NVDA (Windows):** Test with Urdu voice
- [ ] **JAWS (Windows):** Verify RTL navigation
- [ ] **VoiceOver (macOS/iOS):** Test on Safari
- [ ] **TalkBack (Android):** Test on Chrome Mobile

**Key checks:**
- Screen reader announces text in correct order
- Navigation shortcuts work (H for headings, etc.)
- Form labels associated correctly
- ARIA attributes don't break RTL flow

### Keyboard Navigation

- [ ] Tab order follows RTL flow (right to left, top to bottom)
- [ ] Arrow keys work correctly in dropdowns/menus
- [ ] Focus indicators visible and positioned correctly
- [ ] Skip links work in RTL layout

### Color Contrast

- Ensure color contrast ratios meet WCAG 2.1 AA standards:
  - Normal text: 4.5:1
  - Large text: 3:1
  - UI components: 3:1

### Focus Management

```css
/* Ensure focus outlines work in RTL */
[dir="rtl"] :focus {
  outline-offset: 2px;
  outline: 2px solid var(--ifm-color-primary);
}
```

## Mobile RTL Testing

### Device Testing

**iOS Devices:**
1. Set device language to Urdu in Settings
2. Open Safari
3. Navigate to site
4. Test touch gestures (swipes should feel natural)

**Android Devices:**
1. Set device language to Urdu in Settings
2. Open Chrome
3. Navigate to site
4. Test navigation drawer, menus

### Responsive Breakpoints

Test RTL at all breakpoints:
- Mobile: 320px - 480px
- Tablet: 768px - 1024px
- Desktop: 1200px+

### Touch Target Sizes

- Minimum 44x44px for all interactive elements
- Adequate spacing between targets (8px minimum)
- Buttons and links easy to tap in RTL layout

## Automated Testing

### Browser DevTools

**Chrome DevTools:**
```javascript
// Force RTL in console
document.documentElement.setAttribute('dir', 'rtl');

// Check computed direction
getComputedStyle(document.documentElement).direction;
```

**Firefox DevTools:**
- Use "Responsive Design Mode" (Ctrl+Shift+M)
- Set language to Urdu in settings
- Check layout with RTL enabled

### Docusaurus Testing Commands

```bash
# Test Urdu build
npm run build -- --locale ur

# Serve built Urdu site
npm run serve -- --locale ur

# Write Urdu translations
npm run write-translations -- --locale ur

# Clear cache if issues
npm run clear
```

### Visual Regression Testing

Consider using tools like:
- Percy.io
- Chromatic
- BackstopJS

Create baseline screenshots for both LTR and RTL, then compare on changes.

## Debugging RTL Issues

### Step 1: Identify the Problem

1. Take screenshot of issue
2. Note which component is affected
3. Check browser DevTools for computed styles

### Step 2: Locate the CSS

1. Use DevTools to find the CSS rule
2. Check if using physical properties (left/right) or logical (inline-start/end)
3. Note any `!important` declarations

### Step 3: Apply Fix

**Option 1: Use logical properties** (preferred)
```css
/* Instead of */
margin-left: 1rem;

/* Use */
margin-inline-start: 1rem;
```

**Option 2: RTL-specific override**
```css
[dir="rtl"] .element {
  margin-right: 1rem; /* Mirrors the LTR margin-left */
  margin-left: 0;
}
```

### Step 4: Test the Fix

1. Refresh page
2. Verify in both LTR and RTL
3. Check on mobile
4. Test in different browsers

## Performance Considerations

### CSS Bundle Size

- Logical properties don't increase bundle size
- RTL-specific overrides add minimal CSS
- Use PostCSS to optimize

### Layout Shifts

- Avoid layout shifts when switching languages
- Use `contain: layout` for stable sections
- Test CLS (Cumulative Layout Shift) scores

### Font Loading

Urdu fonts can be large. Optimize with:
- `font-display: swap` for faster rendering
- Subset fonts to required characters
- Use system fonts as fallback

```css
@font-face {
  font-family: 'Urdu Font';
  src: url('/fonts/urdu.woff2') format('woff2');
  font-display: swap;
  unicode-range: U+0600-06FF; /* Urdu/Arabic range */
}
```

## Checklist Summary

### Pre-Launch RTL Checklist

- [ ] All pages tested in Urdu mode
- [ ] Navbar and footer mirror correctly
- [ ] Forms and inputs work in RTL
- [ ] ChatBot interface tested
- [ ] Mobile responsive in RTL
- [ ] Browser compatibility verified
- [ ] Screen reader tested
- [ ] Keyboard navigation works
- [ ] No layout shifts when switching languages
- [ ] All icons and images positioned correctly
- [ ] Animations feel natural in RTL
- [ ] Performance metrics acceptable
- [ ] RTL CSS integrated into build
- [ ] Translation files complete
- [ ] Documentation updated

### Quick Issue Reference

| Issue | Quick Fix |
|-------|-----------|
| Text not RTL | Check `dir="rtl"` on `<html>` |
| Wrong margins | Use `margin-inline-start/end` |
| Icons flipped wrong | Add `transform: scaleX(-1)` in RTL |
| Sidebar wrong side | Use logical positioning |
| Buttons wrong side | Use flexbox with `flex-direction: row-reverse` in RTL |
| Scroll feels wrong | Check `overflow` and `scroll-snap` |

## Additional Resources

### Official Documentation
- [Docusaurus i18n Guide](https://docusaurus.io/docs/i18n/introduction)
- [MDN RTL CSS](https://developer.mozilla.org/en-US/docs/Web/CSS/CSS_Logical_Properties)
- [W3C Writing Modes](https://www.w3.org/TR/css-writing-modes-3/)

### Tools
- [RTL Tester Browser Extension](https://chrome.google.com/webstore/detail/rtl-tester)
- [BiDi Checker](https://www.w3.org/International/questions/qa-bidi-unicode-controls)

### Social Media Handles
- [YouTube](https://www.youtube.com/@RomaSportsLive)
- [LinkedIn](http://www.linkedin.com/in/irfan-hussain-12b66361)
- [Facebook](https://www.facebook.com/irfan.zaidi.56)
---

**Last Updated:** 2025-12-10
**Version:** 1.0
**Maintainer:** Physical AI Textbook Team
