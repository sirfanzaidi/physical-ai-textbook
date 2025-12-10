# RTL Quick Reference Card

## Essential Commands

```bash
# Start Urdu development server
npm run start -- --locale ur

# Build Urdu version
npm run build -- --locale ur

# Serve built Urdu site
npm run serve -- --locale ur

# Clear cache
npm run clear
```

## CSS Logical Properties Cheat Sheet

### Margins

| Physical | Logical | RTL Behavior |
|----------|---------|--------------|
| `margin-left` | `margin-inline-start` | Becomes right |
| `margin-right` | `margin-inline-end` | Becomes left |
| `margin-top` | `margin-block-start` | Stays top |
| `margin-bottom` | `margin-block-end` | Stays bottom |

### Padding

| Physical | Logical | RTL Behavior |
|----------|---------|--------------|
| `padding-left` | `padding-inline-start` | Becomes right |
| `padding-right` | `padding-inline-end` | Becomes left |
| `padding-top` | `padding-block-start` | Stays top |
| `padding-bottom` | `padding-block-end` | Stays bottom |

### Positioning

| Physical | Logical | RTL Behavior |
|----------|---------|--------------|
| `left` | `inset-inline-start` | Becomes right |
| `right` | `inset-inline-end` | Becomes left |
| `top` | `inset-block-start` | Stays top |
| `bottom` | `inset-block-end` | Stays bottom |

### Borders

| Physical | Logical | RTL Behavior |
|----------|---------|--------------|
| `border-left` | `border-inline-start` | Becomes right |
| `border-right` | `border-inline-end` | Becomes left |
| `border-top` | `border-block-start` | Stays top |
| `border-bottom` | `border-block-end` | Stays bottom |

### Border Radius

| Physical | Logical | RTL Behavior |
|----------|---------|--------------|
| `border-top-left-radius` | `border-start-start-radius` | Becomes top-right |
| `border-top-right-radius` | `border-start-end-radius` | Becomes top-left |
| `border-bottom-left-radius` | `border-end-start-radius` | Becomes bottom-right |
| `border-bottom-right-radius` | `border-end-end-radius` | Becomes bottom-left |

### Text Alignment

| Physical | Logical | RTL Behavior |
|----------|---------|--------------|
| `text-align: left` | `text-align: start` | Becomes right |
| `text-align: right` | `text-align: end` | Becomes left |
| `text-align: center` | `text-align: center` | Stays center |

## Common RTL Patterns

### 1. Basic Text & Layout

```css
/* ❌ Wrong (LTR only) */
.element {
  margin-left: 1rem;
  padding-right: 2rem;
  text-align: left;
}

/* ✅ Correct (Auto RTL/LTR) */
.element {
  margin-inline-start: 1rem;
  padding-inline-end: 2rem;
  text-align: start;
}
```

### 2. Floating Elements

```css
/* ❌ Wrong */
.float-left {
  float: left;
}

/* ✅ Correct */
.float-start {
  float: inline-start;
}

/* 🔧 Or use RTL selector */
html[dir="rtl"] .float-left {
  float: right;
}
```

### 3. Flexbox Direction

```css
/* ❌ Wrong (always LTR) */
.container {
  display: flex;
  flex-direction: row;
}

/* ✅ Correct (auto-reverses in RTL) */
.container {
  display: flex;
  /* flex-direction: row is default and auto-reverses */
}

/* 🔧 Or explicitly for RTL */
html[dir="rtl"] .container {
  flex-direction: row-reverse;
}
```

### 4. Absolute Positioning

```css
/* ❌ Wrong */
.positioned {
  position: absolute;
  left: 20px;
}

/* ✅ Correct */
.positioned {
  position: absolute;
  inset-inline-start: 20px;
}

/* 🔧 Or use RTL selector */
html[dir="rtl"] .positioned {
  left: auto;
  right: 20px;
}
```

### 5. Icons Next to Text

```css
/* ❌ Wrong */
.icon-text .icon {
  margin-right: 0.5rem;
}

/* ✅ Correct */
.icon-text .icon {
  margin-inline-end: 0.5rem;
}
```

### 6. Directional Icons (Arrows, Chevrons)

```css
/* ✅ Flip in RTL */
html[dir="rtl"] .arrow-right,
html[dir="rtl"] .chevron-right {
  transform: scaleX(-1);
}
```

### 7. Sidebar Layout

```css
/* ❌ Wrong */
.sidebar {
  float: left;
  border-right: 1px solid #ccc;
  margin-right: 20px;
}

/* ✅ Correct */
.sidebar {
  float: inline-start;
  border-inline-end: 1px solid #ccc;
  margin-inline-end: 20px;
}

/* 🔧 Or use RTL selector */
html[dir="rtl"] .sidebar {
  float: right;
  border-left: 1px solid #ccc;
  border-right: none;
  margin-left: 20px;
  margin-right: 0;
}
```

### 8. Form Inputs

```css
/* ✅ RTL inputs */
html[dir="rtl"] input,
html[dir="rtl"] textarea {
  text-align: right;
  direction: rtl;
}

html[dir="rtl"] input::placeholder {
  text-align: right;
}
```

### 9. Buttons with Icons

```css
/* ✅ Reverse icon order */
html[dir="rtl"] .btn-with-icon {
  flex-direction: row-reverse;
}

/* Icon spacing */
html[dir="rtl"] .btn-icon {
  margin-inline-start: 0.5rem;
  margin-inline-end: 0;
}
```

### 10. Dropdown Menus

```css
/* ✅ RTL dropdown */
html[dir="rtl"] .dropdown-menu {
  left: auto;
  right: 0;
  text-align: right;
}

html[dir="rtl"] .dropdown-arrow {
  transform: rotate(180deg);
}
```

## RTL Selector Patterns

### Basic RTL Selector

```css
html[dir="rtl"] .element {
  /* RTL-specific styles */
}
```

### RTL + Dark Mode

```css
[data-theme="dark"] html[dir="rtl"] .element {
  /* RTL + dark mode styles */
}
```

### RTL + Responsive

```css
@media (max-width: 768px) {
  html[dir="rtl"] .element {
    /* RTL mobile styles */
  }
}
```

### RTL + Hover/Focus

```css
html[dir="rtl"] .element:hover {
  /* RTL hover styles */
}

html[dir="rtl"] .element:focus {
  /* RTL focus styles */
}
```

## Testing Checklist (Essential)

### Visual Checks
- [ ] Text flows right-to-left
- [ ] Navbar logo on right
- [ ] Sidebar on left
- [ ] Buttons and links positioned correctly
- [ ] Icons flipped appropriately
- [ ] No layout overflow

### DevTools Checks
- [ ] `<html dir="rtl">` is set
- [ ] `<html lang="ur-PK">` is set
- [ ] No CSS errors in console
- [ ] RTL CSS loaded

### Functional Checks
- [ ] Forms work (input, submit)
- [ ] Navigation works
- [ ] Dropdowns open correctly
- [ ] Modals display properly
- [ ] Search functions

## Common Mistakes

### 1. Hardcoded Directions

```css
/* ❌ Don't do this */
.element {
  padding-left: 20px !important;
}

/* ✅ Do this */
.element {
  padding-inline-start: 20px;
}
```

### 2. Forgetting to Flip Icons

```css
/* ❌ Missing RTL flip */
.arrow {
  /* Arrow always points right, even in RTL */
}

/* ✅ Flip in RTL */
html[dir="rtl"] .arrow {
  transform: scaleX(-1);
}
```

### 3. Absolute Positioning Without RTL

```css
/* ❌ Fixed to left side only */
.floating-button {
  position: fixed;
  left: 20px;
}

/* ✅ RTL-aware */
.floating-button {
  position: fixed;
  inset-inline-start: 20px;
}
```

### 4. Not Testing in RTL

```css
/* ❌ Assuming it works */
/* (No RTL testing) */

/* ✅ Always test in RTL */
npm run start -- --locale ur
```

## Quick Fixes

### Issue: Text Not RTL

```css
html[dir="rtl"] .element {
  direction: rtl;
  text-align: right;
}
```

### Issue: Margins Wrong

```css
/* Replace */
margin-left: 1rem; → margin-inline-start: 1rem;
margin-right: 1rem; → margin-inline-end: 1rem;
```

### Issue: Sidebar Wrong Side

```css
html[dir="rtl"] .sidebar {
  left: auto;
  right: 0;
  border-left: 1px solid #ccc;
  border-right: none;
}
```

### Issue: Icons Wrong Side

```css
html[dir="rtl"] .icon {
  margin-inline-end: 0;
  margin-inline-start: 0.5rem;
}
```

### Issue: Buttons Wrong Order

```css
html[dir="rtl"] .button-group {
  flex-direction: row-reverse;
}
```

## Debugging Tips

### 1. Check HTML Direction

```javascript
// In browser console
console.log(document.documentElement.dir); // Should be "rtl"
```

### 2. Check Computed Styles

```javascript
// In browser console
const el = document.querySelector('.element');
console.log(getComputedStyle(el).direction); // Should be "rtl"
console.log(getComputedStyle(el).textAlign); // Should be "right" or "start"
```

### 3. Toggle RTL in DevTools

```javascript
// Force RTL
document.documentElement.setAttribute('dir', 'rtl');

// Force LTR
document.documentElement.setAttribute('dir', 'ltr');
```

### 4. Visualize Logical Properties

In Chrome DevTools:
1. Inspect element
2. Check "Computed" tab
3. Look for logical property names

## Performance Tips

1. **Use logical properties** (faster than RTL selectors)
2. **Minimize RTL-specific overrides** (less CSS to parse)
3. **Avoid `!important`** (harder to override)
4. **Combine selectors** (fewer rules)
5. **Use CSS variables** (easier to maintain)

## Browser Support

| Feature | Chrome | Firefox | Safari | Edge |
|---------|--------|---------|--------|------|
| Logical Properties | 89+ | 88+ | 14.1+ | 89+ |
| RTL Selectors | All | All | All | All |
| `dir="rtl"` | All | All | All | All |

## Resources

- **Testing Guide:** `RTL_TESTING_GUIDE.md`
- **Checklist:** `RTL_TESTING_CHECKLIST.md`
- **Integration:** `RTL_INTEGRATION_STEPS.md`
- **CSS File:** `src/css/rtl-overrides.css`

## Need Help?

1. Check the comprehensive [RTL Testing Guide](./RTL_TESTING_GUIDE.md)
2. Use the [Testing Checklist](./RTL_TESTING_CHECKLIST.md)
3. Refer to [Integration Steps](./RTL_INTEGRATION_STEPS.md)
4. Consult [MDN CSS Logical Properties](https://developer.mozilla.org/en-US/docs/Web/CSS/CSS_Logical_Properties)

---

**Quick Reference Version:** 1.0
**Last Updated:** 2025-12-10

**Print this page and keep it handy during RTL development!**
