# RTL CSS Integration Steps

## Quick Integration Guide

Follow these steps to integrate the RTL CSS overrides into your Physical AI Textbook website.

---

## Step 1: Verify RTL CSS File Exists

Ensure the RTL CSS file is in place:

```
D:\physical-ai-textbook\website\src\css\rtl-overrides.css
```

If not, the file has been created for you with comprehensive RTL overrides.

---

## Step 2: Update Docusaurus Config

Edit `D:\physical-ai-textbook\website\docusaurus.config.ts` to include the RTL CSS:

### Option A: Single Custom CSS (Recommended)

If you want to keep a single CSS file, import the RTL overrides in your existing `custom.css`:

**File:** `D:\physical-ai-textbook\website\src\css\custom.css`

Add this line at the END of the file:

```css
/* Import RTL overrides */
@import './rtl-overrides.css';
```

**No changes needed** to `docusaurus.config.ts` with this approach.

### Option B: Multiple Custom CSS Files

Alternatively, you can specify multiple CSS files in the config:

**File:** `D:\physical-ai-textbook\website\docusaurus.config.ts`

Find the `theme` section (around line 78) and update it:

```typescript
theme: {
  customCss: [
    './src/css/custom.css',
    './src/css/rtl-overrides.css', // Add this line
  ],
},
```

**Change from:**
```typescript
theme: {
  customCss: './src/css/custom.css',
},
```

**To:**
```typescript
theme: {
  customCss: [
    './src/css/custom.css',
    './src/css/rtl-overrides.css',
  ],
},
```

---

## Step 3: Test the Integration

### Start Development Server

```bash
cd D:\physical-ai-textbook\website
npm run start
```

### Test Urdu Version

Visit: `http://localhost:3000/ur/`

### Verify RTL CSS is Loaded

1. Open browser DevTools (F12)
2. Go to **Network** tab
3. Filter by **CSS**
4. Refresh the page
5. Verify `rtl-overrides.css` is loaded (if using Option B)

OR

1. Inspect any element
2. Check **Styles** tab
3. Look for styles from `rtl-overrides.css`

### Quick Visual Test

1. Check that text aligns to the right
2. Check that navbar logo is on the right
3. Check that sidebar is on the left
4. Check that buttons and icons are mirrored

---

## Step 4: Build and Test Production

### Build the Urdu Site

```bash
npm run build -- --locale ur
```

### Serve the Built Site

```bash
npm run serve -- --locale ur
```

Visit: `http://localhost:3000/ur/`

### Verify Production Build

- [ ] No build errors
- [ ] CSS file included in bundle
- [ ] RTL styles apply correctly
- [ ] No console errors
- [ ] Performance acceptable

---

## Step 5: Run Comprehensive Testing

Use the provided testing resources:

1. **RTL Testing Guide:** `D:\physical-ai-textbook\website\RTL_TESTING_GUIDE.md`
   - Step-by-step testing instructions
   - Common issues and fixes
   - Browser compatibility notes

2. **RTL Testing Checklist:** `D:\physical-ai-textbook\website\RTL_TESTING_CHECKLIST.md`
   - 300+ checkpoints
   - Component-by-component verification
   - Sign-off sheet

---

## Troubleshooting

### Issue: RTL Styles Not Applying

**Solution:**

1. Clear cache: `npm run clear`
2. Restart dev server: `npm run start`
3. Hard refresh browser (Ctrl+Shift+R or Cmd+Shift+R)
4. Check browser console for CSS load errors

### Issue: Some Elements Still LTR

**Solution:**

1. Verify `<html dir="rtl">` is set (check in DevTools)
2. Check if specific component needs override in `rtl-overrides.css`
3. Add custom RTL styles to `rtl-overrides.css` at the bottom

### Issue: Build Fails

**Solution:**

1. Check for syntax errors in `rtl-overrides.css`
2. Ensure file path is correct in config
3. Run `npm run clear` and rebuild

### Issue: Performance Degradation

**Solution:**

1. Use logical CSS properties (they're more efficient)
2. Minimize RTL-specific selectors
3. Use browser DevTools Performance tab to identify bottlenecks

---

## Advanced: Adding Custom RTL Styles

If you need to add RTL styles for custom components:

### 1. Locate the Custom Component Section

Open `D:\physical-ai-textbook\website\src\css\rtl-overrides.css`

Scroll to the bottom (around line 800+):

```css
/* ============================================
   CUSTOM COMPONENT OVERRIDES
   ============================================ */

/**
 * Add any project-specific RTL overrides below this line
 */
```

### 2. Add Your Custom Styles

Example for a custom component:

```css
/* Custom feature card component */
html[dir="rtl"] .featureCard {
  text-align: right;
}

html[dir="rtl"] .featureCard__icon {
  margin-inline-end: 0;
  margin-inline-start: 1rem;
}

html[dir="rtl"] .featureCard__title {
  text-align: right;
}

html[dir="rtl"] .featureCard__cta {
  flex-direction: row-reverse;
}
```

### 3. Use Logical Properties (Preferred)

When possible, update the original component to use logical properties:

**Instead of:**
```css
.featureCard {
  margin-left: 1rem;
  padding-right: 2rem;
}
```

**Use:**
```css
.featureCard {
  margin-inline-start: 1rem; /* Auto-adjusts for RTL */
  padding-inline-end: 2rem;
}
```

This way, you don't need RTL-specific overrides!

---

## Testing Commands Reference

```bash
# Start dev server (all locales)
npm run start

# Start dev server (Urdu only)
npm run start -- --locale ur

# Build all locales
npm run build

# Build Urdu only
npm run build -- --locale ur

# Serve built site (Urdu)
npm run serve -- --locale ur

# Clear cache
npm run clear

# Write Urdu translations
npm run write-translations -- --locale ur
```

---

## Integration Checklist

- [ ] RTL CSS file created at `src/css/rtl-overrides.css`
- [ ] RTL CSS imported in `custom.css` OR added to `docusaurus.config.ts`
- [ ] Development server tested with Urdu locale
- [ ] Visual inspection shows RTL layout
- [ ] Browser DevTools confirms CSS loaded
- [ ] Production build succeeds
- [ ] Production site tested
- [ ] No console errors
- [ ] Comprehensive testing completed using checklist
- [ ] Custom components have RTL styles (if needed)
- [ ] Performance acceptable

---

## Next Steps

1. **Complete Testing:** Use `RTL_TESTING_GUIDE.md` and `RTL_TESTING_CHECKLIST.md`
2. **Fix Issues:** Document and fix any RTL issues found
3. **Translate Content:** Ensure all Urdu translations are complete
4. **Deploy:** Deploy to production with confidence

---

## Need Help?

### Resources
- [Docusaurus i18n Documentation](https://docusaurus.io/docs/i18n/introduction)
- [CSS Logical Properties MDN](https://developer.mozilla.org/en-US/docs/Web/CSS/CSS_Logical_Properties)
- [RTL Testing Guide](./RTL_TESTING_GUIDE.md)
- [RTL Testing Checklist](./RTL_TESTING_CHECKLIST.md)

### Common Questions

**Q: Do I need to update component CSS files?**
A: Ideally, use logical properties in component CSS. Otherwise, add overrides to `rtl-overrides.css`.

**Q: How do I test on mobile?**
A: Use browser DevTools responsive mode, or test on actual devices.

**Q: What if Docusaurus updates break RTL?**
A: Check for breaking changes in Docusaurus release notes. Update `rtl-overrides.css` as needed.

**Q: Can I use SASS/SCSS for RTL styles?**
A: Yes, rename `rtl-overrides.css` to `rtl-overrides.scss` and update imports. Docusaurus supports SASS.

---

**Last Updated:** 2025-12-10
**Version:** 1.0
