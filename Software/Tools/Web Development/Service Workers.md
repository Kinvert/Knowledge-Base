# Service Workers

A **Service Worker** is a **browser-side** [[JavaScript]] API that runs in a background thread, separate from web pages. It acts as a programmable network proxy sitting between your web app and the internet, intercepting requests and enabling features like offline caching, push notifications, and background sync.

**Important clarification:** Service workers run entirely in the user's web browser—they have nothing to do with server-side languages like [[Elixir]], [[Python]], or [[Zig]]. Your server (Phoenix, Django, Node, etc.) serves the initial files, but the service worker executes on the client's machine.

---

## 📚 Overview

When a user visits your website, the browser downloads and registers your service worker script. From then on, the service worker can intercept every network request the page makes and decide how to respond—from cache, from network, or with generated content.

Key highlights:
- Runs in browser, not on server
- Background thread (no DOM access)
- Intercepts and controls network requests
- Enables offline-first architecture
- Requires HTTPS (except localhost for development)
- Event-driven lifecycle

---

## 🧠 Where Does It Run?

```
┌─────────────────────────────────────────────────────────┐
│                    USER'S BROWSER                        │
│  ┌─────────────┐    ┌──────────────────┐                │
│  │  Web Page   │◄──►│  Service Worker  │                │
│  │  (Main      │    │  (Background     │                │
│  │   Thread)   │    │   Thread)        │                │
│  └─────────────┘    └────────┬─────────┘                │
│         │                    │                          │
│         └────────┬───────────┘                          │
│                  ▼                                      │
│         ┌─────────────────┐                             │
│         │  Browser Cache  │                             │
│         └────────┬────────┘                             │
└──────────────────┼──────────────────────────────────────┘
                   │ (if not cached)
                   ▼
┌─────────────────────────────────────────────────────────┐
│                YOUR SERVER                               │
│  (Elixir/Phoenix, Node, Python, Go, etc.)               │
│  Serves initial HTML, JS, CSS, API responses            │
└─────────────────────────────────────────────────────────┘
```

The service worker is **not** a server technology. It's browser JavaScript running on the user's device.

---

## 🧠 Core Concepts

- **Registration**
  Your main page registers the service worker: `navigator.serviceWorker.register('/sw.js')`

- **Scope**
  Service worker controls pages under its URL path (e.g., `/app/sw.js` controls `/app/*`)

- **Lifecycle Events**
  - `install` - Cache initial assets
  - `activate` - Clean up old caches
  - `fetch` - Intercept network requests

- **Cache API**
  Programmatic browser cache: `caches.open()`, `cache.put()`, `cache.match()`

- **Fetch Event**
  Intercept requests and decide: serve from cache, fetch from network, or generate response

- **Update Flow**
  New versions install in background, activate when all old tabs close

---

## 🖥️ Threading Model

Service workers run on a **separate browser thread** from your page. This is different from server threads.

| Concept | Where It Runs | What It Does |
|---------|---------------|--------------|
| **Main Thread** | Browser | DOM manipulation, page JavaScript |
| **Service Worker Thread** | Browser | Network interception, caching |
| **Web Worker Thread** | Browser | CPU-intensive JS tasks |
| **Server Thread** | Your server | Handle HTTP requests, database, etc. |

Service workers **cannot**:
- Access the DOM directly
- Use `window` or `document` objects
- Run synchronous code that blocks
- Access GPU directly (that's [[WebGL]]/[[WebGPU]])

---

## 📊 Comparison: Browser Background Technologies

| Technology | Runs In | DOM Access | Network Control | GPU Access | Survives Page Close | Use Case |
|------------|---------|------------|-----------------|------------|---------------------|----------|
| **Service Worker** | Browser thread | No | Yes | No | Yes | Caching, offline, push |
| **Web Worker** | Browser thread | No | No | No | No | Heavy computation |
| **Shared Worker** | Browser thread | No | No | No | Shared across tabs | Cross-tab state |
| **Worklet** | Render thread | No | No | No | No | Audio, paint |
| **WebGL/WebGPU** | GPU | No | No | **Yes** | No | Graphics, ML inference |

---

## 🔧 Caching Strategies

| Strategy | How It Works | Best For |
|----------|--------------|----------|
| **Cache First** | Check cache → fallback to network | Static assets, fonts, images |
| **Network First** | Try network → fallback to cache | API data, fresh content |
| **Stale While Revalidate** | Serve cache immediately, update in background | Balanced freshness/speed |
| **Cache Only** | Only serve from cache | Fully offline apps |
| **Network Only** | Always fetch from network | Non-cacheable requests |

---

## 🦀 WebAssembly and Service Workers

[[WebAssembly]] (WASM) modules can be cached by service workers just like any other asset. This is useful because WASM files can be large (ML models, game engines, image processing).

**How it works:**
1. Service worker caches the `.wasm` file on install
2. Subsequent page loads get WASM from cache instantly
3. No network round-trip for the binary

```javascript
// sw.js - caching WASM
const CACHE_NAME = 'wasm-cache-v1';
const WASM_FILES = [
  '/app.wasm',
  '/opencv.wasm',
  '/ffmpeg-core.wasm'
];

self.addEventListener('install', (event) => {
  event.waitUntil(
    caches.open(CACHE_NAME)
      .then(cache => cache.addAll(WASM_FILES))
  );
});
```

**Languages that compile to WASM** (and thus work with service worker caching):
- [[Rust]] (via wasm-pack, wasm-bindgen)
- [[Zig]] (native WASM target)
- C/C++ (via Emscripten)
- Go (native WASM target)
- AssemblyScript (TypeScript-like)

Note: The WASM runs in the **main thread** or a **Web Worker**, not in the service worker itself. The service worker just caches and serves the file.

---

## 🔄 Alternatives and Related Technologies

Service workers solve specific problems. Here's how they compare to alternatives:

| Need | Solution | Notes |
|------|----------|-------|
| Offline web app | **Service Worker** | Browser-side caching |
| Heavy computation in browser | **Web Worker** | Background thread for CPU work |
| GPU computation in browser | **WebGPU** / **WebGL** | Graphics and ML inference |
| Edge caching/compute | **[[Cloudflare Workers]]** | Server-side, runs at CDN edge |
| Server-side caching | **Redis**, **Varnish** | Traditional server caching |
| Real-time server push | **[[WebSockets]]**, **SSE** | Different problem (live data) |
| Background sync | **Service Worker + Background Sync API** | Retry failed requests when online |

---

## 🔧 Common Use Cases

- **Offline-capable web apps** - Work without internet connection
- **[[PWA]]** (Progressive Web Apps) - Installable web apps
- **Caching static assets** - CSS, JS, images, fonts load instantly
- **Caching API responses** - Faster subsequent page loads
- **Push notifications** - Alerts even when site is closed
- **Background sync** - Queue requests while offline, sync when online
- **Precaching** - Download critical resources before user needs them

---

## ✅ Pros

- Enables true offline functionality
- Reduces server load via client-side caching
- Dramatically improves perceived performance
- Push notifications without native app
- Works with any web framework (React, Vue, vanilla JS)
- Fine-grained cache control per request type

---

## ❌ Cons

- Requires HTTPS in production (security requirement)
- Complex debugging and lifecycle management
- No DOM access (cannot manipulate page directly)
- Cache invalidation is error-prone
- Can accidentally serve stale content forever
- Not supported in some WebViews and older browsers
- Adds complexity to deployment (cache versioning)

---

## 🔧 Basic Example

```javascript
// main.js - Register the service worker
if ('serviceWorker' in navigator) {
  navigator.serviceWorker.register('/sw.js')
    .then(reg => console.log('SW registered:', reg.scope))
    .catch(err => console.error('SW registration failed:', err));
}
```

```javascript
// sw.js - The service worker itself
const CACHE_NAME = 'my-app-v1';
const ASSETS = [
  '/',
  '/index.html',
  '/styles.css',
  '/app.js',
  '/logo.png'
];

// Install: cache assets
self.addEventListener('install', (event) => {
  event.waitUntil(
    caches.open(CACHE_NAME)
      .then(cache => {
        console.log('Caching assets');
        return cache.addAll(ASSETS);
      })
  );
});

// Activate: clean old caches
self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches.keys().then(keys => {
      return Promise.all(
        keys.filter(key => key !== CACHE_NAME)
            .map(key => caches.delete(key))
      );
    })
  );
});

// Fetch: serve from cache, fallback to network
self.addEventListener('fetch', (event) => {
  event.respondWith(
    caches.match(event.request)
      .then(cachedResponse => {
        if (cachedResponse) {
          return cachedResponse;
        }
        return fetch(event.request);
      })
  );
});
```

---

## 🔧 Using with Server Frameworks

Your server just serves the service worker file like any other static file:

| Server Framework | How to Serve SW |
|------------------|-----------------|
| **[[Phoenix]]** (Elixir) | Put `sw.js` in `priv/static/`, served automatically |
| **[[Django]]** (Python) | Put in `static/`, configure `STATIC_URL` |
| **[[Node.js]]** / Express | `app.use(express.static('public'))` |
| **[[Nginx]]** | Serve from static files directory |
| **[[GitHub Pages]]** | Just commit `sw.js` to repo |

The server doesn't "run" the service worker—it just serves the file. The browser does the rest.

---

## 🔩 Compatible Items

- [[PWA]] - Primary use case for service workers
- [[WebAssembly]] - Cache WASM modules for fast loading
- [[IndexedDB]] - Client-side database for offline data
- [[WebSockets]] - Real-time data (different but complementary)
- [[Electron]] - Desktop apps using web tech
- [[Workbox]] - Google's library for easier SW development
- [[Webpack]] - Build tool plugins for SW generation

---

## 🔗 Related Concepts

- [[PWA]] (Progressive Web Apps)
- [[Cloudflare Workers]] (Edge workers - server side, different concept)
- [[WebAssembly]] (Can be cached and served by service workers)
- [[WebGL]] / [[WebGPU]] (GPU access - separate from service workers)
- [[REST API]] (Cache API responses)
- [[JavaScript]] (Service workers are JS)

---

## 📚 External Resources

- [MDN Service Worker API](https://developer.mozilla.org/en-US/docs/Web/API/Service_Worker_API)
- [Google Workbox](https://developer.chrome.com/docs/workbox/)
- [Service Worker Cookbook](https://serviceworke.rs/)
- [web.dev Offline Guide](https://web.dev/offline-cookbook/)
- [Can I Use: Service Workers](https://caniuse.com/serviceworkers)
- [Jake Archibald's Offline Cookbook](https://jakearchibald.com/2014/offline-cookbook/)
