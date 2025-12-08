# Vercel Deployment Guide

This guide walks through deploying the Physical AI Textbook to Vercel with automatic CI/CD.

## Quick Start (5 minutes)

### 1. Create Vercel Account
- Visit https://vercel.com
- Sign up with GitHub account

### 2. Import Project
- Go to https://vercel.com/new
- Select your `physical-ai-textbook` repository
- Click "Import"

### 3. Configure Project Settings
- **Project Name**: `physical-ai-textbook` (or your preference)
- **Framework Preset**: `Docusaurus`
- **Root Directory**: `.` (repository root)

### 4. Build Settings
Vercel should auto-detect Docusaurus. If not, configure manually:

- **Build Command**:
  ```
  cd website && npm run build
  ```
- **Output Directory**:
  ```
  website/build
  ```
- **Install Command**:
  ```
  npm ci
  ```

### 5. Environment Variables
In Vercel Dashboard → Settings → Environment Variables, add:

```
DOCUSAURUS_BASEURL = /
NODE_ENV = production
```

### 6. Deploy
Click **Deploy** button. Your site will be live at:
- `https://physical-ai-textbook.vercel.app` (default)
- Or your custom domain

## Advanced Setup

### GitHub Secrets for CI/CD

Add these secrets to your GitHub repository (Settings → Secrets):

```
VERCEL_TOKEN: <your-vercel-token>
VERCEL_ORG_ID: <your-org-id>
VERCEL_PROJECT_ID: <your-project-id>
```

**How to get these:**

1. **VERCEL_TOKEN**:
   - Go to Vercel Dashboard → Settings → Tokens
   - Create new token → copy it

2. **VERCEL_ORG_ID** & **VERCEL_PROJECT_ID**:
   - Go to project settings in Vercel
   - Copy values from top of settings page
   - Or run: `vercel project list`

### CI/CD Workflow

The `.github/workflows/vercel-deploy.yml` workflow:

- ✅ Builds on every push to `master`
- ✅ Creates preview deployments for PRs
- ✅ Auto-deploys to production on merge
- ✅ Zero-downtime deployments

### Custom Domain

1. In Vercel Dashboard → Settings → Domains
2. Add your custom domain (e.g., `textbook.yoursite.com`)
3. Follow DNS configuration instructions
4. Wait for DNS propagation (~10 minutes)

## Monitoring

### Deployment Status
- Vercel Dashboard shows all deployments
- GitHub shows CI/CD status in PRs
- Production: https://vercel.com/deployments/physical-ai-textbook

### Logs
- Build logs: Vercel Dashboard → Deployments → [deployment] → Logs
- Runtime logs: Real-time in Vercel Dashboard
- GitHub Actions: Repository → Actions → Workflow runs

### Performance
- Vercel Analytics (free tier) included
- Web Vitals monitoring
- Dashboard → Analytics

## Troubleshooting

### Build Fails
1. Check logs in Vercel Dashboard
2. Verify `website/build` directory generates locally:
   ```bash
   cd website && npm run build
   ```
3. Ensure all environment variables are set

### Deploy Doesn't Trigger
- Check GitHub Actions workflow in `.github/workflows/vercel-deploy.yml`
- Verify `VERCEL_TOKEN` secret is set
- Manual trigger: Push to `master` branch

### Site Shows 404
- Verify `DOCUSAURUS_BASEURL=/` environment variable
- Clear Vercel cache: Dashboard → Settings → Git → Clear Cache
- Redeploy

### Custom Domain Not Working
- Check DNS records (CNAME, A record)
- Wait for propagation (up to 48 hours)
- Verify in Vercel Dashboard → Domains → Status

## File Structure

```
physical-ai-textbook/
├── vercel.json                  # Vercel configuration
├── .github/
│   └── workflows/
│       └── vercel-deploy.yml    # Auto-deployment workflow
├── website/
│   ├── build/                   # Built static files (auto-generated)
│   ├── src/                     # React components
│   ├── blog/                    # Blog posts
│   ├── docs/                    # Textbook chapters
│   ├── docusaurus.config.ts     # Docusaurus config
│   └── package.json             # Dependencies
└── README.md                    # This file
```

## Cost

**Vercel Pricing (Hobby Plan - Free)**:
- ✅ Unlimited deployments
- ✅ Automatic HTTPS
- ✅ Custom domains
- ✅ Git integration
- ✅ Preview deployments
- ✅ 50 GB bandwidth/month
- ✅ 100 GB serverless function invocations/month

No credit card required for hobby plan.

**Scaling to Pro ($20/month)**:
- Unlimited bandwidth
- Priority support
- Advanced analytics
- Team collaboration

## Best Practices

1. **Always test locally first**:
   ```bash
   cd website && npm run build
   npm run start
   ```

2. **Use preview deployments** for PRs to test before merge

3. **Monitor Web Vitals** in Vercel Analytics

4. **Enable branch protection** requiring CI to pass before merge

5. **Use semantic versioning** in git tags for releases:
   ```bash
   git tag -a v1.0.0 -m "Release v1.0.0"
   git push origin v1.0.0
   ```

## Additional Resources

- [Vercel Documentation](https://vercel.com/docs)
- [Docusaurus on Vercel](https://docusaurus.io/docs/deployment#deploying-to-vercel)
- [GitHub Actions Documentation](https://docs.github.com/en/actions)

---

**Need help?**
- Check Vercel logs first
- Search [Vercel Community](https://vercel.com/help)
- Open an issue on GitHub
