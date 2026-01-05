# GitHub Pages

**GitHub Pages** is a free static site hosting service from GitHub. It serves HTML, CSS, and JavaScript directly from a repository, making it ideal for project documentation, personal sites, blogs, and web-based games. Sites are served from `username.github.io` or custom domains.

---

## 📚 Overview

GitHub Pages integrates tightly with Git workflows—push to a branch and your site updates automatically. It supports Jekyll for static site generation out of the box, but works with any static content including [[PWA]] apps, [[Phaser]] games, or documentation built with tools like MkDocs or Docusaurus.

Key highlights:
- Free hosting for public repositories
- Automatic HTTPS via Let's Encrypt
- Custom domain support
- Integrates with [[GitHub Actions]] for build pipelines
- No server configuration needed

---

## 🧠 Core Concepts

- **Source Branch**
  Configure which branch/folder to deploy from (e.g., `main`, `gh-pages`, or `/docs`)

- **Jekyll Integration**
  Built-in static site generator for Markdown-based sites

- **Custom Domains**
  Point your own domain via CNAME record

- **HTTPS**
  Automatic SSL certificates for secure connections

- **Build and Deploy**
  Use GitHub Actions for custom build steps before deployment

- **404 Pages**
  Custom `404.html` for missing routes

---

## 📊 Comparison Chart

| Platform | Cost | Custom Domain | SSL | Build Tools | Best For |
|----------|------|---------------|-----|-------------|----------|
| **GitHub Pages** | Free | Yes | Auto | Jekyll, Actions | Docs, portfolios, small sites |
| **Netlify** | Free tier | Yes | Auto | Many | JAMstack, forms, functions |
| **Vercel** | Free tier | Yes | Auto | Next.js, etc. | React/Next.js apps |
| **Cloudflare Pages** | Free tier | Yes | Auto | Many | Edge performance |
| **GitLab Pages** | Free | Yes | Auto | GitLab CI | GitLab users |
| **Surge** | Free tier | Yes | Paid | CLI deploy | Quick prototypes |
| **Render** | Free tier | Yes | Auto | Many | Static + backends |

---

## 🔧 Use Cases

- Project documentation and wikis
- Personal portfolio sites
- Technical blogs with Jekyll or Hugo
- [[PWA]] game hosting ([[Kaplay]], [[Phaser]])
- API documentation (Swagger UI, Redoc)
- Landing pages for open-source projects
- Demo sites for libraries and tools

---

## ✅ Pros

- Completely free for public repos
- Zero configuration for basic sites
- Automatic deploys on git push
- HTTPS included automatically
- Custom domain support
- Tight GitHub ecosystem integration

---

## ❌ Cons

- Static sites only (no server-side code)
- Public repos required for free tier
- 1GB storage / 100GB bandwidth soft limits
- No built-in forms or functions
- Jekyll builds can be slow for large sites
- Limited redirect/rewrite options

---

## 🔧 Setup Options

**Option 1: Deploy from branch**
1. Go to repo Settings → Pages
2. Select source branch (`main` or `gh-pages`)
3. Choose folder (`/` or `/docs`)
4. Site deploys to `username.github.io/repo`

**Option 2: GitHub Actions workflow**
```yaml
name: Deploy to Pages

on:
  push:
    branches: [main]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Build
        run: npm run build
      - name: Deploy
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./dist
```

---

## 🔩 Compatible Items

- [[GitHub Actions]] - Custom build pipelines
- [[Jekyll]] - Built-in static site generator
- [[PWA]] - Offline-capable web apps
- [[Phaser]] - Browser game hosting
- [[Kaplay]] - Browser game hosting
- [[TypeScript]] - Build before deploy

---

## 🔗 Related Concepts

- [[GitHub Actions]] (Automation and builds)
- [[CI-CD Pipelines]] (Deployment automation)
- [[PWA]] (Progressive Web Apps)
- [[Jekyll]] (Static site generator)

---

## 📚 External Resources

- [GitHub Pages Documentation](https://docs.github.com/en/pages)
- [Configuring Custom Domains](https://docs.github.com/en/pages/configuring-a-custom-domain-for-your-github-pages-site)
- [GitHub Actions for Pages](https://github.com/marketplace/actions/github-pages-action)
- [Jekyll Documentation](https://jekyllrb.com/docs/)
