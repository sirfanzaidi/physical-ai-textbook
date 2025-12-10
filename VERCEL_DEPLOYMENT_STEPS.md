# Vercel Deployment Guide - Physical AI Textbook

**Status:** ✅ Code pushed to GitHub and ready for Vercel deployment
**Date:** 2025-12-10
**Locales:** English (/) and Urdu (/ur/) with RTL support

---

## Prerequisites

- ✅ GitHub account with access to `sirfanzaidi/physical-ai-textbook`
- ✅ Vercel account (free tier available)
- ✅ Code pushed to GitHub (DONE ✓)

---

## Step 1: Deploy to Vercel (Dashboard Method - Recommended)

### 1.1 Go to Vercel

Visit: https://vercel.com/new

### 1.2 Import Your Repository

1. Click **"Import Git Repository"**
2. Search for: `sirfanzaidi/physical-ai-textbook`
3. Click **"Import"**

### 1.3 Configure Your Project

**Project Settings:**

| Setting | Value |
|---------|-------|
| **Project Name** | `physical-ai-textbook` (or custom) |
| **Framework Preset** | **Docusaurus** |
| **Root Directory** | `website` |
| **Build Command** | `npm run build` |
| **Output Directory** | `build` |
| **Install Command** | `npm install` (default) |

### 1.4 Environment Variables (Optional)

For backend API integration:

| Key | Value | Notes |
|-----|-------|-------|
| `BACKEND_URL` | `https://your-backend.com` | Replace with your backend URL |

*(Currently using inline URL detection, so this is optional)*

### 1.5 Deploy

1. Click **"Deploy"**
2. Wait for build to complete (~5 minutes)
3. You'll see: ✅ **Production: Ready**

---

## Step 2: Verify Deployment

Once deployment completes, Vercel will provide your URL:

```
https://physical-ai-textbook-[random].vercel.app
```

### 2.1 Test English Version

Visit: `https://your-vercel-url.vercel.app/`

**Verify:**
- [ ] Homepage loads
- [ ] Navigation bar visible (English)
- [ ] Sign In / Sign Up buttons work
- [ ] Docusaurus docs accessible

### 2.2 Test Urdu Version

Visit: `https://your-vercel-url.vercel.app/ur/`

**Verify:**
- [ ] Homepage in Urdu (فزیکل اے آئی ٹیکسٹ بک)
- [ ] RTL layout applied (text flows right-to-left)
- [ ] Navbar in Urdu (کتاب, بلاگ, سائن اپ, سائن ان)
- [ ] All routes work (/ur/signin, /ur/signup, etc.)

### 2.3 Check Locale Routing

**English Routes:**
- `https://your-url/` → English
- `https://your-url/signin` → English sign in
- `https://your-url/docs` → English docs

**Urdu Routes:**
- `https://your-url/ur/` → Urdu
- `https://your-url/ur/signin` → Urdu sign in
- `https://your-url/ur/docs` → Urdu docs

### 2.4 Verify SEO Tags

Open browser DevTools (F12) → Elements tab:

**English page should have:**
```html
<html lang="en-US" dir="ltr">
<link rel="alternate" hreflang="en" href="https://.../">
<link rel="alternate" hreflang="ur" href="https://.../ur/">
<link rel="alternate" hreflang="x-default" href="https://.../">
```

**Urdu page should have:**
```html
<html lang="ur-PK" dir="rtl">
<link rel="alternate" hreflang="ur" href="https://.../ur/">
<link rel="alternate" hreflang="en" href="https://.../">
```

---

## Step 3: Custom Domain (Optional)

1. In Vercel Dashboard → Settings → Domains
2. Add your custom domain (e.g., `physical-ai-textbook.com`)
3. Follow DNS configuration steps
4. Vercel auto-configures SSL certificate

---

## Step 4: Monitor & Maintain

### 4.1 Analytics

Vercel Dashboard → Analytics tab:
- Page load times
- Core Web Vitals
- Error rates

### 4.2 Deployments

Each time you push to `main`:
- Vercel auto-builds and deploys
- Previous version saved for rollback
- Automatic production deployment

### 4.3 Environment-Specific Builds

**Production (Recommended):**
- Automatic on every push to `main`
- Full optimization
- Public URL: `https://your-domain`

**Preview Deployments:**
- Automatic on pull requests
- Separate preview URL for each PR
- Good for testing before merge

---

## Step 5: Backend API Integration

### Option A: Same-Origin Backend (Recommended for Future)

If you deploy backend to the same domain:
```
Frontend: https://your-domain/
Backend:  https://your-domain/api/
```

**Current Frontend Configuration** (in `signin.tsx`, `signup.tsx`, `AuthContext.tsx`):
```typescript
const apiUrl = typeof window !== 'undefined' &&
  (window.location.hostname === 'localhost' ||
   window.location.hostname === '127.0.0.1')
  ? 'http://localhost:8001'
  : '/api';
```

**For Production:** Backend must be accessible at `/api` paths
- Option 1: Deploy backend to Vercel Functions at `/api`
- Option 2: Configure reverse proxy
- Option 3: Use separate domain with CORS

### Option B: Separate Backend Domain

If backend is on different domain (e.g., `api.your-domain.com`):

1. Update inline URL detection in `signin.tsx`, `signup.tsx`, `AuthContext.tsx`:
   ```typescript
   const apiUrl = window.location.hostname === 'localhost'
     ? 'http://localhost:8001'
     : 'https://api.your-domain.com';
   ```

2. Backend must have CORS headers:
   ```
   Access-Control-Allow-Origin: https://your-domain
   Access-Control-Allow-Methods: GET, POST, PUT, DELETE, OPTIONS
   Access-Control-Allow-Credentials: true
   ```

---

## Troubleshooting

### Issue: Build Fails

**Error:** `npm: command not found`

**Solution:** Vercel automatically installs Node.js, but verify:
1. `Root Directory` is set to `website`
2. `Build Command` is `npm run build`

### Issue: 404 on `/ur/` routes

**Cause:** Docusaurus i18n not configured

**Verify:**
1. Check `website/docusaurus.config.ts` has:
   ```typescript
   locales: ['en', 'ur'],
   ```
2. Translation files exist: `website/i18n/ur/`

### Issue: RTL Not Applying

**Cause:** CSS not loading or cache issue

**Solutions:**
1. Hard refresh: `Ctrl+Shift+Delete` (clear cache)
2. Verify `rtl-overrides.css` in build: DevTools → Network tab → css files
3. Check `<html dir="rtl">` attribute present

### Issue: Authentication Returns 404

**Cause:** Backend API not accessible at `/api` paths

**Solutions:**
1. Configure Vercel `vercel.json` with rewrites:
   ```json
   {
     "rewrites": [{
       "source": "/api/:path*",
       "destination": "https://your-backend.com/:path*"
     }]
   }
   ```
2. Or deploy backend to same origin
3. Or update API URL in frontend code

### Issue: Slow Performance

**Causes:**
- Large bundle size
- Unoptimized images
- Slow API calls

**Solutions:**
1. Enable Vercel Analytics to identify bottleneck
2. Use `vercel/analytics` for monitoring
3. Implement image optimization
4. Add caching headers

---

## Performance Optimization

### 1. Image Optimization

If you add images, use Next.js Image component:
```tsx
import Image from 'next/image';
<Image src="/img/logo.png" alt="Logo" width={200} height={200} />
```

### 2. Code Splitting

Docusaurus automatically code-splits per route.

### 3. Caching

Vercel automatically caches:
- Static assets: 365 days
- HTML: 60 seconds (can customize)

### 4. CDN

All assets automatically served via Vercel's edge CDN.

---

## Rollback Procedure

If deployment has issues:

1. Vercel Dashboard → Deployments
2. Find previous successful deployment
3. Click **"Promote to Production"**
4. Instant rollback (~1 minute)

---

## Next Steps

### Immediate (Post-Deployment)
- [ ] Test all URLs and routes
- [ ] Verify RTL layout on `/ur/`
- [ ] Test authentication (if backend configured)
- [ ] Check performance metrics

### Short-term (1-2 weeks)
- [ ] Set up custom domain
- [ ] Configure backend API integration
- [ ] Add more Urdu translations
- [ ] Monitor Vercel Analytics

### Medium-term (1-3 months)
- [ ] Deploy backend to Vercel Functions or similar
- [ ] Implement user feedback
- [ ] Monitor and optimize performance
- [ ] Add more language support if needed

---

## Support & Resources

**Docusaurus i18n:**
- https://docusaurus.io/docs/i18n/introduction

**Vercel Deployment:**
- https://vercel.com/docs

**Performance Monitoring:**
- https://vercel.com/analytics

**Custom Domain Setup:**
- https://vercel.com/docs/concepts/projects/domains

---

## Deployment Checklist

### Before Deployment
- [x] Code pushed to GitHub
- [x] English build successful
- [x] Urdu build successful
- [x] DEPLOYMENT.md reviewed

### During Deployment
- [ ] Connect GitHub repo to Vercel
- [ ] Configure build settings (website root dir)
- [ ] Deploy to production

### After Deployment
- [ ] Test English version: `/`
- [ ] Test Urdu version: `/ur/`
- [ ] Check all routes work
- [ ] Verify RTL layout
- [ ] Monitor first 24 hours
- [ ] Share deployed URL

---

## Summary

**Your Physical AI Textbook is now:**
- ✅ Fully built (English + Urdu)
- ✅ Code pushed to GitHub
- ✅ Ready for Vercel deployment
- ✅ Production-ready with RTL support

**Estimated deployment time:** 5-10 minutes

**Result:** Professional, bilingual (English/Urdu) website with RTL layout automatically applied!

---

**Last Updated:** 2025-12-10
**Next Phase:** Monitor production deployment & gather user feedback
