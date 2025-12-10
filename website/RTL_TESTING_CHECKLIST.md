# RTL Testing Checklist for Urdu Language Support

## Quick Reference

**Test URL:** `http://localhost:3000/ur/`
**Test Command:** `npm run start -- --locale ur`
**Browser DevTools Check:** `document.documentElement.dir === "rtl"`

---

## Pre-Testing Setup

- [ ] Install dependencies: `npm install`
- [ ] Clear cache: `npm run clear`
- [ ] Start dev server: `npm run start`
- [ ] Navigate to: `http://localhost:3000/ur/`
- [ ] Verify `dir="rtl"` in HTML tag (DevTools)
- [ ] Verify `lang="ur-PK"` in HTML tag
- [ ] Open browser console for debugging

---

## 1. Layout & Direction Fundamentals

### Global Layout
- [ ] Page flows right-to-left
- [ ] Scrollbar appears on left side
- [ ] Text aligns to the right by default
- [ ] No horizontal overflow issues
- [ ] Main content area positioned correctly

### Visual Inspection
- [ ] No elements overlap or collision
- [ ] Spacing looks balanced and symmetric
- [ ] No awkward gaps or misalignments
- [ ] Layout feels natural for RTL reading

---

## 2. Navbar Testing

### Desktop Navbar
- [ ] Logo positioned on RIGHT side
- [ ] Site title appears LEFT of logo (if applicable)
- [ ] Navigation links flow from right to left
- [ ] "Textbook" link appears after logo (to the left)
- [ ] "Blog" link positioned correctly
- [ ] Auth buttons (Sign In/Sign Up) on LEFT side
- [ ] GitHub link on LEFT side
- [ ] Language switcher on LEFT side
- [ ] Spacing between items is consistent
- [ ] Hover states work correctly
- [ ] Active link styling displays properly

### Dropdown Menus
- [ ] Dropdown icon (chevron) points correctly
- [ ] Dropdown menu opens to the LEFT
- [ ] Dropdown items align RIGHT
- [ ] Submenu items flow RTL
- [ ] Hover/focus states work
- [ ] Clicking outside closes dropdown

### Mobile Navbar (< 996px)
- [ ] Hamburger icon on LEFT side
- [ ] Hamburger menu slides from LEFT
- [ ] Menu items align RIGHT
- [ ] Close button (×) positioned correctly
- [ ] Logo visible in mobile menu
- [ ] Menu items stack naturally
- [ ] Touch targets are adequate (44x44px minimum)

---

## 3. Homepage Testing

### Hero Section
- [ ] Hero title aligns RIGHT
- [ ] Hero subtitle aligns RIGHT
- [ ] CTA button ("Start Learning") positioned correctly
- [ ] Button text centered
- [ ] Button icon (if any) on LEFT side of text
- [ ] Background image/gradient doesn't interfere with RTL

### Chapters Showcase
- [ ] Section heading aligns RIGHT
- [ ] Chapter cards flow from right to left
- [ ] First card appears on RIGHT
- [ ] Card titles align RIGHT
- [ ] Card descriptions align RIGHT
- [ ] Card images/icons positioned on RIGHT
- [ ] "Read More" links positioned correctly
- [ ] Grid gaps are symmetric
- [ ] Hover effects work smoothly
- [ ] Cards stack naturally on mobile

### Call-to-Action Sections
- [ ] CTA text aligns RIGHT
- [ ] CTA buttons positioned correctly
- [ ] Button groups flow RTL

---

## 4. Documentation Pages

### Sidebar (Desktop)
- [ ] Sidebar positioned on LEFT side (mirrored from LTR)
- [ ] Sidebar border on LEFT edge
- [ ] Category headings align RIGHT
- [ ] Menu items align RIGHT
- [ ] Collapse/expand icons point correctly
- [ ] Nested items indented from LEFT
- [ ] Active page indicator on RIGHT edge
- [ ] Scrollbar on LEFT of sidebar
- [ ] Hover states work correctly

### Sidebar (Mobile)
- [ ] Sidebar button positioned correctly
- [ ] Sidebar slides from LEFT
- [ ] Backdrop covers correctly

### Content Area
- [ ] Content positioned to RIGHT of sidebar (desktop)
- [ ] Content full-width on mobile
- [ ] Headings (H1-H6) align RIGHT
- [ ] Paragraphs align RIGHT
- [ ] Text flows naturally RTL

### Lists
- [ ] Unordered lists: bullets on RIGHT
- [ ] Ordered lists: numbers on RIGHT
- [ ] List items align RIGHT
- [ ] Nested lists indent from LEFT
- [ ] List spacing looks natural

### Code Blocks
- [ ] Code blocks remain LTR (correct for code)
- [ ] Code block container aligns RIGHT
- [ ] Line numbers (if any) on LEFT of code
- [ ] Copy button positioned correctly
- [ ] Language label positioned correctly
- [ ] Syntax highlighting works

### Blockquotes
- [ ] Border on RIGHT side (not left)
- [ ] Padding on LEFT (inside border)
- [ ] Text aligns RIGHT
- [ ] Icon/indicator on RIGHT
- [ ] Nested blockquotes work

### Tables
- [ ] Tables flow RTL
- [ ] Headers align RIGHT
- [ ] Cell content aligns RIGHT
- [ ] Borders render correctly
- [ ] Responsive tables scroll naturally
- [ ] Table overflow handled correctly

### Images & Figures
- [ ] Images align RIGHT (or center)
- [ ] Captions align RIGHT
- [ ] Image margins symmetric
- [ ] Lightbox/zoom works
- [ ] Images don't flip (unless intentional)

### Admonitions (Notes/Warnings)
- [ ] Admonition boxes align RIGHT
- [ ] Icon positioned on RIGHT
- [ ] Title aligns RIGHT
- [ ] Content aligns RIGHT
- [ ] Border on correct side

---

## 5. Authentication Pages

### Sign In Page
- [ ] Page title aligns RIGHT
- [ ] Subtitle aligns RIGHT
- [ ] Form container centered
- [ ] Form fields align naturally
- [ ] Labels align RIGHT
- [ ] Input text flows RTL
- [ ] Placeholder text aligns RIGHT
- [ ] Required asterisk (*) on LEFT of label
- [ ] Error messages align RIGHT
- [ ] Error icons positioned correctly
- [ ] Submit button centered/full-width
- [ ] "Don't have an account?" text aligns appropriately
- [ ] "Sign Up" link positioned correctly
- [ ] Focus states visible
- [ ] Tab order flows RTL

### Sign Up Page
- [ ] All form sections align RIGHT
- [ ] Multi-step indicator (if any) mirrors correctly
- [ ] Progress bar fills from right to left
- [ ] Personal info fields flow RTL
- [ ] Dropdown selects align RIGHT
- [ ] Radio buttons on RIGHT of labels
- [ ] Checkboxes on RIGHT of labels
- [ ] Password strength indicator positioned correctly
- [ ] Submit button positioned correctly
- [ ] "Already have account?" link correct

### Form Validation
- [ ] Error messages appear below fields
- [ ] Error messages align RIGHT
- [ ] Success icons positioned correctly
- [ ] Validation happens in real-time
- [ ] Error summary (if any) aligns RIGHT

---

## 6. ChatBot Component

### Floating Button
- [ ] Button positioned on BOTTOM-LEFT (mirrored)
- [ ] Icon orientation correct
- [ ] Hover/focus states work
- [ ] Tooltip (if any) appears correctly
- [ ] Button doesn't cover content

### Chat Window
- [ ] Window positioned on LEFT side
- [ ] Window slides in from LEFT
- [ ] Header aligns RIGHT
- [ ] "Physical AI Assistant" title on RIGHT
- [ ] "Clear" button on LEFT of header
- [ ] Close button (×) on LEFT

### Messages Container
- [ ] User messages align to LEFT (mirrored)
- [ ] Assistant messages align to RIGHT (mirrored)
- [ ] Message bubbles have correct shape/tail
- [ ] Text within bubbles flows RTL
- [ ] "You:" label aligns correctly
- [ ] "Assistant:" label aligns correctly
- [ ] Timestamps positioned correctly
- [ ] Scrollbar on LEFT of container
- [ ] Auto-scroll works smoothly

### Welcome Message
- [ ] Welcome text aligns RIGHT
- [ ] Bullet list items align RIGHT
- [ ] Icons positioned correctly

### Citations
- [ ] "Sources:" heading aligns RIGHT
- [ ] Citation links align RIGHT
- [ ] Relevance scores positioned correctly
- [ ] Citation hover states work
- [ ] Chapter links work

### Input Area
- [ ] Textarea text flows RTL
- [ ] Placeholder aligns RIGHT
- [ ] Send button on LEFT side
- [ ] Send icon (➤) points correctly
- [ ] Disabled state shows properly
- [ ] Focus state visible
- [ ] Resize handle (if any) positioned correctly

### Context Banner
- [ ] "Context:" label aligns RIGHT
- [ ] Selected text preview aligns RIGHT
- [ ] Close button (×) positioned correctly

---

## 7. Footer Testing

### Footer Layout
- [ ] Footer sections flow RTL
- [ ] Column headings align RIGHT
- [ ] Links align RIGHT
- [ ] Columns appear from right to left

### Footer Links
- [ ] Social media icons ordered RTL
- [ ] External link icons on LEFT of text
- [ ] Hover states work
- [ ] Links navigate correctly

### Footer Bottom
- [ ] Copyright text aligns CENTER or RIGHT
- [ ] "Built with Docusaurus" aligns appropriately
- [ ] Footer logo (if any) positioned correctly

---

## 8. Navigation & Pagination

### Breadcrumbs
- [ ] Breadcrumbs flow RTL
- [ ] Separators point correctly (← not →)
- [ ] Current page highlighted
- [ ] Links work correctly

### Previous/Next Navigation
- [ ] "Previous" button on LEFT
- [ ] "Next" button on RIGHT
- [ ] Button labels correct
- [ ] Icons point correctly
- [ ] Hover states work

### Table of Contents (TOC)
- [ ] TOC positioned correctly
- [ ] Headings align RIGHT
- [ ] Active link highlighted
- [ ] Border/indicator on RIGHT edge
- [ ] Scrollspy works (highlights current section)

---

## 9. Interactive Elements

### Buttons
- [ ] Button text centered
- [ ] Icons positioned correctly (usually on LEFT in RTL)
- [ ] Hover effects work
- [ ] Focus states visible
- [ ] Disabled states show correctly
- [ ] Loading states (spinners) positioned correctly

### Dropdowns & Selects
- [ ] Dropdown arrow on LEFT side
- [ ] Options align RIGHT
- [ ] Selected value aligns RIGHT
- [ ] Dropdown opens downward (or upward if space)
- [ ] Options text flows RTL

### Modals & Dialogs
- [ ] Modal centered on screen
- [ ] Close button on LEFT (top-left corner)
- [ ] Modal title aligns RIGHT
- [ ] Modal content aligns RIGHT
- [ ] Action buttons positioned correctly

### Tooltips & Popovers
- [ ] Tooltip appears on correct side
- [ ] Tooltip text aligns appropriately
- [ ] Arrow points to trigger element

### Tabs
- [ ] Tab items flow RTL
- [ ] Active tab highlighted correctly
- [ ] Tab panels align RIGHT
- [ ] Border/underline on correct edge

---

## 10. Typography & Text

### Urdu Text
- [ ] Urdu text renders correctly (no broken characters)
- [ ] Font supports Urdu characters
- [ ] Text is readable and clear
- [ ] Line height appropriate
- [ ] Letter spacing (if any) appropriate

### Mixed Text (Urdu + English)
- [ ] English words embedded in Urdu flow correctly
- [ ] Numbers maintain LTR within RTL text
- [ ] Punctuation positioned correctly
- [ ] Parentheses mirror correctly: (text) becomes (text)

### Text Truncation
- [ ] Ellipsis (...) on LEFT side
- [ ] "Show more" links positioned correctly
- [ ] Truncated text readable

---

## 11. Responsive Design

### Mobile (< 768px)
- [ ] All elements stack naturally
- [ ] Touch targets adequate size
- [ ] No horizontal scroll
- [ ] Text readable without zoom
- [ ] Forms usable on mobile

### Tablet (768px - 996px)
- [ ] Layout adapts correctly
- [ ] Sidebar behavior correct
- [ ] Content readable

### Desktop (> 996px)
- [ ] Full layout displays correctly
- [ ] Sidebar and content side-by-side
- [ ] Wide screens handled well

---

## 12. Icons & Images

### Directional Icons
- [ ] Arrow icons flipped correctly
- [ ] Chevron icons point correctly
- [ ] Back/forward icons appropriate
- [ ] Navigation icons make sense

### Non-Directional Icons
- [ ] Static icons don't flip (checkmarks, stars, etc.)
- [ ] Brand logos remain unflipped
- [ ] Social media icons unchanged

### Images
- [ ] Photos don't flip
- [ ] Diagrams appropriate for RTL (or remain LTR if code/technical)
- [ ] Image captions align RIGHT

---

## 13. Animations & Transitions

### Slide Animations
- [ ] Slide-in animations from correct direction
- [ ] Slide-out animations to correct direction
- [ ] Smooth and natural movement

### Progress Indicators
- [ ] Progress bars fill from right to left
- [ ] Loading spinners position correctly
- [ ] Skeleton loaders align correctly

### Hover Effects
- [ ] Underlines appear correctly
- [ ] Shadows don't break layout
- [ ] Color transitions work

---

## 14. Accessibility (A11y)

### Screen Readers
- [ ] NVDA reads content correctly (Windows)
- [ ] JAWS reads content correctly (Windows)
- [ ] VoiceOver reads content correctly (macOS/iOS)
- [ ] TalkBack reads content correctly (Android)
- [ ] Reading order follows RTL

### Keyboard Navigation
- [ ] Tab order flows RTL (right to left, top to bottom)
- [ ] Focus indicators visible
- [ ] Skip links work
- [ ] Arrow keys work in menus/dropdowns
- [ ] Enter/Space activate buttons
- [ ] Escape closes modals/dropdowns

### ARIA Attributes
- [ ] ARIA labels don't break RTL
- [ ] `aria-label` and `aria-describedby` work
- [ ] Live regions announce correctly

### Color Contrast
- [ ] Text contrast meets WCAG AA (4.5:1 for normal, 3:1 for large)
- [ ] Interactive elements contrast meets WCAG AA (3:1)
- [ ] Focus indicators meet contrast requirements

---

## 15. Browser Testing

### Chrome (Latest)
- [ ] All features work
- [ ] Layout correct
- [ ] No console errors

### Firefox (Latest)
- [ ] All features work
- [ ] Layout correct
- [ ] No console errors

### Safari (Latest)
- [ ] All features work
- [ ] Layout correct
- [ ] No console errors
- [ ] Logical properties supported

### Edge (Latest)
- [ ] All features work
- [ ] Layout correct
- [ ] No console errors

### Mobile Safari (iOS)
- [ ] Touch interactions work
- [ ] Layout responsive
- [ ] Keyboard appears correctly

### Chrome Mobile (Android)
- [ ] Touch interactions work
- [ ] Layout responsive
- [ ] Keyboard appears correctly

---

## 16. Performance

### Load Time
- [ ] Page loads in < 3 seconds
- [ ] Fonts load without FOUT (Flash of Unstyled Text)
- [ ] Images lazy-load correctly

### Layout Shifts
- [ ] No Cumulative Layout Shift (CLS) when switching languages
- [ ] Content doesn't jump during load
- [ ] Smooth transitions

### Scroll Performance
- [ ] Smooth scrolling
- [ ] No jank or lag
- [ ] Scroll position maintained

---

## 17. Edge Cases

### Very Long Words
- [ ] Long Urdu words wrap correctly
- [ ] No overflow
- [ ] Hyphens (if any) appropriate

### Empty States
- [ ] Empty lists display correctly
- [ ] No data messages align RIGHT
- [ ] Placeholder content aligns RIGHT

### Error States
- [ ] Error pages (404, 500) align RIGHT
- [ ] Error messages clear
- [ ] Recovery options visible

### Loading States
- [ ] Skeletons align correctly
- [ ] Spinners positioned correctly
- [ ] Loading text aligns RIGHT

---

## 18. Data & Forms

### Date Pickers
- [ ] Calendar opens correctly
- [ ] Dates flow RTL
- [ ] Month/year selectors work

### Time Pickers
- [ ] Time input flows RTL (if localized)
- [ ] AM/PM positioned correctly

### Search
- [ ] Search input flows RTL
- [ ] Search icon positioned correctly
- [ ] Results align RIGHT
- [ ] Autocomplete suggestions align RIGHT

---

## 19. Third-Party Integrations

### Analytics
- [ ] Analytics tracking RTL interactions correctly
- [ ] Event tracking works

### Social Sharing
- [ ] Share buttons positioned correctly
- [ ] Share dialogs work

### Embeds (YouTube, etc.)
- [ ] Embedded content displays correctly
- [ ] Captions (if any) align correctly

---

## 20. Build & Deployment

### Build Process
- [ ] `npm run build -- --locale ur` succeeds
- [ ] No build warnings
- [ ] No console errors in built site

### Serve Built Site
- [ ] `npm run serve -- --locale ur` works
- [ ] Production build looks identical to dev

### Deployment
- [ ] RTL CSS included in bundle
- [ ] All fonts loaded
- [ ] No missing assets

---

## Common Issues Quick Fix

| Issue | Quick Fix |
|-------|-----------|
| Text not RTL | Check `<html dir="rtl">` |
| Wrong margins | Use `margin-inline-start/end` |
| Icons wrong side | Add RTL selector to flip |
| Sidebar wrong side | Check `border-inline-start` |
| Buttons wrong order | Use `flex-direction: row-reverse` |
| Scrollbar wrong side | Browser handles automatically |
| Text not aligned | Add `text-align: right` or `start` |
| Forms broken | Check input `direction: rtl` |

---

## Sign-Off

### Tester Information
- **Tester Name:** ___________________________
- **Date:** ___________________________
- **Browser(s) Used:** ___________________________
- **Device(s) Used:** ___________________________

### Final Checklist
- [ ] All critical issues resolved
- [ ] No console errors
- [ ] Accessible to screen readers
- [ ] Works on mobile devices
- [ ] Performance acceptable
- [ ] Ready for production

### Notes
_Add any additional observations or issues here:_

---

**Total Items:** 300+
**Estimated Testing Time:** 3-4 hours (comprehensive)
**Priority:** High (before Urdu launch)

---

**Document Version:** 1.0
**Last Updated:** 2025-12-10
