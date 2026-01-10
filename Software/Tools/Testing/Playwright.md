# Playwright

Playwright is a modern, high-performance browser automation framework created by Microsoft. It provides reliable end-to-end testing, scraping, interaction, and automation for web applications across Chromium, Firefox, and WebKit. Unlike older tools such as Selenium, Playwright emphasizes determinism, auto-waiting, and cross-browser consistency. Its design makes it suitable not only for QA automation but also for research workflows, data collection systems, and agent-based RL environments that require programmatic interaction with real browsers.

---

## ⚙️ Overview

Playwright enables developers to script fully automated interactions with websites using JavaScript/TypeScript, Python, Java, or .NET. It launches browsers in isolated contexts, manages lifecycle and permissions, and supports headless or headed modes. Playwright is designed around reliability—each API action auto-waits for the browser to be ready, which drastically reduces flaky tests.

Key advantages:
- True cross-browser automation (Chrome, Edge, Firefox, Safari/WebKit)
- Multiple language bindings
- Deterministic behavior via auto-waiting and event synchronization
- Browser contexts, tracing, and robust debugging tools

---

## 🧠 Core Concepts

- **Browser Contexts**: Lightweight, isolated browser sessions. Useful for parallel tests, multi-user flows, or RL agents running simultaneously.
- **Selectors**: CSS, text, role-based, XPath, and Playwright-specific selectors.
- **Auto-Waiting**: Built-in mechanism that waits for elements to be ready before interacting.
- **Tracing and Debugging**: Record snapshots, DOM states, and videos of test runs.
- **Network Control**: Intercept requests, mock responses, throttle bandwidth.
- **Multi-Tab / Multi-Page Control**: Manage multiple pages within one context.
- **Cross-Browser Engine**: WebKit, Firefox, and Chromium abstraction.

---

## 📊 Comparison Chart

| Feature | Playwright | [[Selenium]] | Puppeteer | Cypress | WebDriverIO |
|---------|-----------|----------|-----------|---------|-------------|
| Cross-Browser | Yes (Chromium, WebKit, Firefox) | Yes | Chromium Only | Chromium Only | Yes |
| Auto-Waiting | Excellent | Weak | Good | Good | Moderate |
| Language Support | JS/TS, Python, Java, .NET | Many | JS/TS | JS/TS | JS/TS |
| Network Interception | Strong | Moderate | Strong | Limited | Moderate |
| Speed | High | Medium | High | High | Medium |
| Test Runner Built-In | Yes (Playwright Test) | No | No | Yes | Yes |
| Parallelization | Easy | Moderately complex | Manual | Good | Good |
| Reliability | High | Low–Medium | High | Medium | Medium |

---

## 🚀 Use Cases

- **End-to-End Testing** for web apps  
- **Data scraping** when static scraping fails  
- **Web automation** for CI pipelines  
- **Robust browser-based workflows** (report generation, login automation)  
- **Multi-agent RL environments** that interact with real webpages  
- **Adversarial testing** against login systems, rate limits, or UI changes  
- **Cross-browser compatibility testing**

---

## ⭐ Strengths

- High reliability due to auto-waiting  
- First-class cross-browser support including WebKit  
- Built-in test runner with parallelization and fixtures  
- Excellent debugging: inspector, snapshotting, tracing  
- Multi-tab, multi-context capabilities  
- Great API ergonomics across all languages  

---

## ❌ Weaknesses

- Browser binaries are large and bundled per language binding  
- More opinionated than generic WebDriver solutions  
- Some corporate environments restrict its bundled browsers  
- Not ideal for non-browser automation (obviously)  

---

## 🧰 Developer Tools

- `npx playwright test` (JS/TS runner)  
- Playwright Inspector for interactive debugging  
- Trace Viewer (.zip artifacts)  
- Screenshot and video recording  
- `pip install playwright` for Python  
- Network interception tools  
- CI integrations: GitHub Actions, GitLab, Jenkins  

---

## E2E Testing WebAssembly 3D Renderers

Testing [[WebAssembly]]-based 3D renderers (Three.js, Babylon.js, Bevy WASM, custom WebGL/WebGPU engines) presents unique challenges. Unlike DOM-based UIs, canvas content is opaque—you can't query for elements. GPU differences cause pixel-level variations across machines. WASM initialization is asynchronous. This section covers a production-ready workflow for reliable visual regression testing of 3D web applications.

### The Core Challenges

| Challenge | Why It's Hard | Solution |
|-----------|---------------|----------|
| **GPU variance** | Different GPUs render slightly differently | Software rendering (SwiftShader) |
| **Floating-point** | Minor precision differences across CPUs | Tolerance-based comparison |
| **Antialiasing** | Edge pixels vary by driver | Perceptual comparison (SSIM) |
| **WASM init** | Async compilation, unpredictable timing | Custom ready events |
| **Animations** | Screenshots during motion are non-deterministic | Scene stabilization waits |
| **Canvas opacity** | Can't query canvas like DOM | Screenshot comparison |

### Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                     E2E Test Pipeline                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  1. BROWSER SETUP                                                │
│     └─► SwiftShader (software GL) for deterministic rendering   │
│                                                                  │
│  2. WASM INITIALIZATION                                          │
│     └─► Wait for custom 'renderer-ready' event                  │
│     └─► Poll for canvas dimensions / GL context                 │
│                                                                  │
│  3. SCENE LOADING                                                │
│     └─► Wait for assets (models, textures) to load              │
│     └─► Wait for specific object count or scene state           │
│                                                                  │
│  4. SCENE STABILIZATION                                          │
│     └─► Pause physics / animations                              │
│     └─► Wait N frames or fixed time                             │
│     └─► Set camera to known position                            │
│                                                                  │
│  5. SCREENSHOT CAPTURE                                           │
│     └─► Capture at fixed viewport size                          │
│     └─► Multiple angles for 3D coverage                         │
│     └─► Store with platform/GPU metadata                        │
│                                                                  │
│  6. INTELLIGENT COMPARISON                                       │
│     └─► SSIM (Structural Similarity Index)                      │
│     └─► Perceptual hashing for major regressions                │
│     └─► Tolerance masks for known variable regions              │
│                                                                  │
│  7. DIFF GENERATION                                              │
│     └─► Visual diff images for failures                         │
│     └─► Artifact upload for debugging                           │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Step 1: Browser Setup for Deterministic Rendering

The key to consistent 3D screenshots across machines is **software rendering**. Chrome's SwiftShader provides a CPU-based OpenGL implementation that produces identical output regardless of GPU.

```typescript
// playwright.config.ts
import { defineConfig, devices } from '@playwright/test';

export default defineConfig({
  use: {
    // Use software rendering for consistent results
    launchOptions: {
      args: [
        '--use-gl=swiftshader',           // CPU-based OpenGL
        '--disable-gpu-sandbox',
        '--disable-software-rasterizer',
        '--no-sandbox',                   // Required for some CI
        '--disable-dev-shm-usage',        // Prevent /dev/shm issues in Docker
      ],
    },
    viewport: { width: 1280, height: 720 },  // Fixed viewport
    deviceScaleFactor: 1,                     // No HiDPI scaling
  },

  // Separate projects for visual tests
  projects: [
    {
      name: 'visual-regression',
      testMatch: '**/*.visual.spec.ts',
      use: { ...devices['Desktop Chrome'] },
    },
  ],
});
```

**CI environment (GitHub Actions):**

```yaml
# .github/workflows/visual-tests.yml
jobs:
  visual-tests:
    runs-on: ubuntu-latest
    container:
      image: mcr.microsoft.com/playwright:v1.40.0-jammy
    steps:
      - uses: actions/checkout@v4
      - run: npm ci
      - run: npx playwright install --with-deps chromium
      - run: npx playwright test --project=visual-regression
      - uses: actions/upload-artifact@v4
        if: failure()
        with:
          name: visual-diff-report
          path: test-results/
```

### Step 2: Waiting for WASM Initialization

WASM modules compile asynchronously. Your 3D app should emit a custom event when ready:

**In your WASM/JS application:**

```javascript
// Inside your 3D app initialization
async function initRenderer() {
  const wasm = await import('./renderer.wasm');
  await wasm.init();

  const canvas = document.getElementById('canvas');
  const gl = canvas.getContext('webgl2');

  // ... setup scene ...

  // Signal to Playwright that we're ready
  window.__RENDERER_READY__ = true;
  window.dispatchEvent(new CustomEvent('renderer-ready', {
    detail: {
      width: canvas.width,
      height: canvas.height,
      renderer: 'webgl2',
      objectCount: scene.objects.length,
    }
  }));
}
```

**In your Playwright test:**

```typescript
// tests/render.visual.spec.ts
import { test, expect, Page } from '@playwright/test';

async function waitForRenderer(page: Page, timeout = 30000): Promise<void> {
  // Method 1: Wait for custom event
  await page.evaluate(() => {
    return new Promise<void>((resolve, reject) => {
      if (window.__RENDERER_READY__) {
        resolve();
        return;
      }

      const timer = setTimeout(() => reject(new Error('Renderer init timeout')), 30000);
      window.addEventListener('renderer-ready', () => {
        clearTimeout(timer);
        resolve();
      }, { once: true });
    });
  });

  // Method 2: Poll for canvas context (fallback)
  await expect(async () => {
    const hasContext = await page.evaluate(() => {
      const canvas = document.querySelector('canvas');
      return canvas && canvas.getContext('webgl2') !== null;
    });
    expect(hasContext).toBe(true);
  }).toPass({ timeout });
}

test('renderer initializes correctly', async ({ page }) => {
  await page.goto('/');
  await waitForRenderer(page);

  // Verify canvas dimensions
  const dimensions = await page.evaluate(() => {
    const canvas = document.querySelector('canvas');
    return { width: canvas.width, height: canvas.height };
  });
  expect(dimensions.width).toBeGreaterThan(0);
});
```

### Step 3: Scene Stabilization

Before taking screenshots, ensure the scene is in a deterministic state:

```typescript
interface SceneState {
  objectCount: number;
  cameraPosition: [number, number, number];
  frameCount: number;
  isAnimating: boolean;
}

async function stabilizeScene(page: Page): Promise<void> {
  // 1. Pause animations if running
  await page.evaluate(() => {
    if (window.__RENDERER__?.pauseAnimations) {
      window.__RENDERER__.pauseAnimations();
    }
  });

  // 2. Set camera to known position
  await page.evaluate(() => {
    window.__RENDERER__.setCamera({
      position: [0, 2, 5],
      target: [0, 0, 0],
      fov: 60,
    });
  });

  // 3. Wait for N frames to ensure GPU has flushed
  await page.evaluate(() => {
    return new Promise<void>((resolve) => {
      let framesWaited = 0;
      const targetFrames = 5;

      function waitFrame() {
        requestAnimationFrame(() => {
          framesWaited++;
          if (framesWaited >= targetFrames) {
            resolve();
          } else {
            waitFrame();
          }
        });
      }
      waitFrame();
    });
  });

  // 4. Optional: Wait for physics to settle
  await page.evaluate(() => {
    return new Promise<void>((resolve) => {
      const checkInterval = setInterval(() => {
        const velocities = window.__RENDERER__.getAllVelocities();
        const allSettled = velocities.every(v =>
          Math.abs(v.x) < 0.001 &&
          Math.abs(v.y) < 0.001 &&
          Math.abs(v.z) < 0.001
        );
        if (allSettled) {
          clearInterval(checkInterval);
          resolve();
        }
      }, 100);

      // Timeout after 5 seconds
      setTimeout(() => {
        clearInterval(checkInterval);
        resolve();
      }, 5000);
    });
  });
}
```

### Step 4: Screenshot Capture Strategy

Capture multiple angles for comprehensive 3D coverage:

```typescript
interface CameraPreset {
  name: string;
  position: [number, number, number];
  target: [number, number, number];
}

const CAMERA_PRESETS: CameraPreset[] = [
  { name: 'front',     position: [0, 0, 5],   target: [0, 0, 0] },
  { name: 'top',       position: [0, 5, 0],   target: [0, 0, 0] },
  { name: 'side',      position: [5, 0, 0],   target: [0, 0, 0] },
  { name: 'isometric', position: [3, 3, 3],   target: [0, 0, 0] },
  { name: 'detail',    position: [1, 1, 1.5], target: [0, 0.5, 0] },
];

async function captureMultiAngle(
  page: Page,
  testName: string
): Promise<Map<string, Buffer>> {
  const screenshots = new Map<string, Buffer>();

  for (const preset of CAMERA_PRESETS) {
    // Set camera
    await page.evaluate((p) => {
      window.__RENDERER__.setCamera({
        position: p.position,
        target: p.target,
      });
    }, preset);

    // Wait for render
    await page.waitForTimeout(100);  // Or wait for frame

    // Capture canvas specifically (not full page)
    const canvas = await page.locator('canvas');
    const screenshot = await canvas.screenshot({
      type: 'png',
      // Disable animations during capture
      animations: 'disabled',
    });

    screenshots.set(`${testName}-${preset.name}`, screenshot);
  }

  return screenshots;
}
```

### Step 5: Intelligent Image Comparison

Pixel-perfect comparison fails for 3D. Use **SSIM** (Structural Similarity Index) instead:

```typescript
// tests/utils/image-compare.ts
import { PNG } from 'pngjs';
import ssim from 'ssim.js';

interface ComparisonResult {
  match: boolean;
  ssimScore: number;        // 0-1, higher is more similar
  mssimScore: number;       // Mean SSIM across regions
  diffPixels: number;       // For reporting
  diffImage?: Buffer;       // Visual diff
}

export async function compareImages(
  actual: Buffer,
  expected: Buffer,
  options: {
    ssimThreshold?: number;  // Default 0.95 (95% similar)
    generateDiff?: boolean;
  } = {}
): Promise<ComparisonResult> {
  const { ssimThreshold = 0.95, generateDiff = true } = options;

  const actualPng = PNG.sync.read(actual);
  const expectedPng = PNG.sync.read(expected);

  // Ensure same dimensions
  if (actualPng.width !== expectedPng.width ||
      actualPng.height !== expectedPng.height) {
    return {
      match: false,
      ssimScore: 0,
      mssimScore: 0,
      diffPixels: actualPng.width * actualPng.height,
    };
  }

  // Calculate SSIM
  const { mssim, ssim_map } = ssim(
    { data: actualPng.data, width: actualPng.width, height: actualPng.height },
    { data: expectedPng.data, width: expectedPng.width, height: expectedPng.height }
  );

  // Generate visual diff if requested
  let diffImage: Buffer | undefined;
  if (generateDiff && mssim < ssimThreshold) {
    diffImage = generateVisualDiff(actualPng, expectedPng, ssim_map);
  }

  // Count pixels that differ significantly
  let diffPixels = 0;
  for (let i = 0; i < ssim_map.length; i++) {
    if (ssim_map[i] < 0.9) diffPixels++;
  }

  return {
    match: mssim >= ssimThreshold,
    ssimScore: mssim,
    mssimScore: mssim,
    diffPixels,
    diffImage,
  };
}

function generateVisualDiff(
  actual: PNG,
  expected: PNG,
  ssimMap: Float32Array
): Buffer {
  const diff = new PNG({ width: actual.width, height: actual.height });

  for (let y = 0; y < actual.height; y++) {
    for (let x = 0; x < actual.width; x++) {
      const idx = (y * actual.width + x) * 4;
      const ssimIdx = y * actual.width + x;
      const similarity = ssimMap[ssimIdx];

      if (similarity > 0.95) {
        // Matching: show grayscale of actual
        const gray = (actual.data[idx] + actual.data[idx+1] + actual.data[idx+2]) / 3;
        diff.data[idx] = gray * 0.3;
        diff.data[idx+1] = gray * 0.3;
        diff.data[idx+2] = gray * 0.3;
        diff.data[idx+3] = 255;
      } else {
        // Difference: highlight in red/magenta based on severity
        const severity = 1 - similarity;
        diff.data[idx] = 255;  // Red
        diff.data[idx+1] = Math.floor((1 - severity) * 100);
        diff.data[idx+2] = Math.floor(severity * 255);  // More magenta = worse
        diff.data[idx+3] = 255;
      }
    }
  }

  return PNG.sync.write(diff);
}
```

### Step 6: Putting It All Together

```typescript
// tests/kitchen-scene.visual.spec.ts
import { test, expect } from '@playwright/test';
import * as fs from 'fs';
import * as path from 'path';
import { compareImages } from './utils/image-compare';

const SNAPSHOTS_DIR = path.join(__dirname, '__snapshots__');
const DIFF_DIR = path.join(__dirname, '__diffs__');

test.describe('Kitchen Scene Visual Tests', () => {
  test.beforeAll(async () => {
    // Ensure directories exist
    fs.mkdirSync(SNAPSHOTS_DIR, { recursive: true });
    fs.mkdirSync(DIFF_DIR, { recursive: true });
  });

  test('kitchen renders correctly from all angles', async ({ page }) => {
    await page.goto('/kitchen-scene');
    await waitForRenderer(page);

    // Load specific scene configuration
    await page.evaluate(() => {
      window.__RENDERER__.loadScene('test-kitchen', {
        seed: 12345,  // Deterministic randomization
        objectCount: 10,
        lighting: 'standard',
      });
    });

    await stabilizeScene(page);

    // Capture all angles
    const screenshots = await captureMultiAngle(page, 'kitchen');

    // Compare each angle
    const failures: string[] = [];

    for (const [name, actual] of screenshots) {
      const snapshotPath = path.join(SNAPSHOTS_DIR, `${name}.png`);
      const diffPath = path.join(DIFF_DIR, `${name}-diff.png`);

      if (!fs.existsSync(snapshotPath)) {
        // First run: save as baseline
        fs.writeFileSync(snapshotPath, actual);
        console.log(`Created baseline: ${name}`);
        continue;
      }

      const expected = fs.readFileSync(snapshotPath);
      const result = await compareImages(actual, expected, {
        ssimThreshold: 0.95,
        generateDiff: true,
      });

      if (!result.match) {
        // Save diff for debugging
        if (result.diffImage) {
          fs.writeFileSync(diffPath, result.diffImage);
        }
        fs.writeFileSync(
          path.join(DIFF_DIR, `${name}-actual.png`),
          actual
        );

        failures.push(
          `${name}: SSIM ${result.ssimScore.toFixed(4)} < 0.95 ` +
          `(${result.diffPixels} pixels differ)`
        );
      }
    }

    expect(failures, failures.join('\n')).toHaveLength(0);
  });

  test('object interactions render correctly', async ({ page }) => {
    await page.goto('/kitchen-scene');
    await waitForRenderer(page);
    await stabilizeScene(page);

    // Test: Click on object, verify selection highlight
    const canvas = page.locator('canvas');
    await canvas.click({ position: { x: 640, y: 360 } });

    // Wait for selection animation
    await page.waitForTimeout(200);

    const afterClick = await canvas.screenshot();

    // Verify something changed (selection highlight appeared)
    const beforeClick = fs.readFileSync(
      path.join(SNAPSHOTS_DIR, 'kitchen-front.png')
    );
    const result = await compareImages(afterClick, beforeClick);

    // Should be different (selection visible) but not too different
    expect(result.ssimScore).toBeGreaterThan(0.8);  // Still mostly same scene
    expect(result.ssimScore).toBeLessThan(0.99);    // But with visible change
  });
});
```

### Step 7: Handling Platform-Specific Baselines

When software rendering isn't sufficient, maintain per-platform baselines:

```typescript
// tests/utils/platform-snapshots.ts
import * as os from 'os';
import * as path from 'path';

interface PlatformInfo {
  os: string;
  arch: string;
  glRenderer: string;
}

async function getPlatformInfo(page: Page): Promise<PlatformInfo> {
  const glRenderer = await page.evaluate(() => {
    const canvas = document.createElement('canvas');
    const gl = canvas.getContext('webgl2');
    return gl?.getParameter(gl.RENDERER) || 'unknown';
  });

  return {
    os: os.platform(),      // 'linux', 'darwin', 'win32'
    arch: os.arch(),        // 'x64', 'arm64'
    glRenderer,             // 'SwiftShader', 'NVIDIA GeForce...', etc.
  };
}

function getSnapshotPath(
  baseName: string,
  platform: PlatformInfo,
  snapshotsDir: string
): string {
  // For CI (SwiftShader), use generic path
  if (platform.glRenderer.includes('SwiftShader')) {
    return path.join(snapshotsDir, `${baseName}.png`);
  }

  // For local development with real GPU, use platform-specific
  const platformKey = `${platform.os}-${platform.arch}`;
  return path.join(snapshotsDir, platformKey, `${baseName}.png`);
}
```

### Step 8: Performance Testing

Beyond visual correctness, test render performance:

```typescript
test('maintains 60fps during pan', async ({ page }) => {
  await page.goto('/kitchen-scene');
  await waitForRenderer(page);

  // Collect frame times during interaction
  const frameTimes = await page.evaluate(async () => {
    const times: number[] = [];
    let lastTime = performance.now();

    return new Promise<number[]>((resolve) => {
      let frames = 0;
      const targetFrames = 120;  // 2 seconds at 60fps

      function measureFrame() {
        const now = performance.now();
        times.push(now - lastTime);
        lastTime = now;
        frames++;

        // Simulate camera pan
        window.__RENDERER__.rotateCamera(0.01, 0);

        if (frames < targetFrames) {
          requestAnimationFrame(measureFrame);
        } else {
          resolve(times);
        }
      }

      requestAnimationFrame(measureFrame);
    });
  });

  // Analyze frame times
  const avgFrameTime = frameTimes.reduce((a, b) => a + b) / frameTimes.length;
  const maxFrameTime = Math.max(...frameTimes);
  const fps = 1000 / avgFrameTime;

  console.log(`Average FPS: ${fps.toFixed(1)}, Max frame: ${maxFrameTime.toFixed(1)}ms`);

  expect(fps).toBeGreaterThan(55);           // Target 60fps
  expect(maxFrameTime).toBeLessThan(50);     // No major stutters
});
```

### Step 9: Testing Domain Randomization

If your renderer is for [[Sim2Real]] training with [[Domain Randomization]], test that randomization works:

```typescript
test('domain randomization produces diverse scenes', async ({ page }) => {
  await page.goto('/kitchen-scene');
  await waitForRenderer(page);

  const screenshots: Buffer[] = [];
  const seeds = [1, 2, 3, 4, 5];

  for (const seed of seeds) {
    await page.evaluate((s) => {
      window.__RENDERER__.loadScene('kitchen', {
        seed: s,
        randomizeLighting: true,
        randomizeTextures: true,
        randomizeObjectPositions: true,
      });
    }, seed);

    await stabilizeScene(page);
    const canvas = page.locator('canvas');
    screenshots.push(await canvas.screenshot());
  }

  // Verify scenes are all different from each other
  for (let i = 0; i < screenshots.length; i++) {
    for (let j = i + 1; j < screenshots.length; j++) {
      const result = await compareImages(screenshots[i], screenshots[j]);

      // Different seeds should produce visually different scenes
      expect(result.ssimScore).toBeLessThan(0.85);
    }
  }
});
```

### Quick Reference: Comparison Thresholds

| SSIM Score | Interpretation | Typical Use |
|------------|----------------|-------------|
| **0.99+** | Near identical | Pixel-perfect requirements |
| **0.95-0.99** | Minor differences | Antialiasing, compression artifacts |
| **0.90-0.95** | Noticeable differences | Acceptable for 3D with tolerances |
| **0.80-0.90** | Significant differences | Major lighting/position changes |
| **< 0.80** | Very different | Completely different scenes |

### Debugging Failed Visual Tests

When tests fail, Playwright generates artifacts:

```bash
# View trace for failed test
npx playwright show-trace test-results/kitchen-renders-xyz/trace.zip

# View diff images
open test-results/__diffs__/kitchen-front-diff.png
```

**Common failure patterns:**

| Diff Pattern | Likely Cause | Fix |
|--------------|--------------|-----|
| Edges only | Antialiasing difference | Lower SSIM threshold to 0.93 |
| Random pixels | Floating point / GPU | Use SwiftShader |
| Entire regions | Animation not paused | Add stabilization waits |
| Consistent offset | Camera position drift | Reset camera explicitly |
| Gradient bands | Color quantization | Use 32-bit color depth |

### Summary: The Complete Workflow

1. **Configure SwiftShader** in Playwright config for CI consistency
2. **Emit ready events** from your WASM app
3. **Stabilize scenes** before capture (pause animations, set camera)
4. **Capture multiple angles** for 3D coverage
5. **Use SSIM comparison** with 0.95 threshold (not pixel-perfect)
6. **Generate visual diffs** for debugging failures
7. **Store platform-specific baselines** if needed
8. **Test performance** alongside correctness
9. **Upload artifacts** in CI for debugging

This workflow handles the fundamental non-determinism of GPU rendering while providing meaningful regression detection for WebAssembly 3D applications.

---

## 🔗 Compatible Items

- [[JavaScript]]
- [[TypeScript]]
- [[Python]]
- [[Automation]]
- [[CI-CD]]
- [[Web Scraping]]
- [[Web Testing]]
- [[Selenium]]
- [[Puppeteer]]
- [[WebAssembly]]
- [[WebGL]]

---

## 🧭 Related Concepts

- [[Headless Browsers]]
- [[Network Interception]]
- [[Selectors]]
- [[Test Automation]]
- [[Browser Engines]]
- [[E2E Testing]]
- [[TDD]]
- [[Domain Randomization]]
- [[Sim2Real]]

---

## 📚 External Resources

- Official website: https://playwright.dev/
- API reference: https://playwright.dev/docs/api/class-playwright
- Python docs: https://playwright.dev/python/docs/intro
- Java integration: https://playwright.dev/java/docs/intro
- .NET integration: https://playwright.dev/dotnet/docs/intro
- Examples repo: https://github.com/microsoft/playwright
- SSIM.js (image comparison): https://github.com/nicoschwarz/ssim.js

---

## 📝 Summary

Playwright is an advanced browser automation framework offering reliability, speed, and full cross-browser coverage. Its deterministic auto-waiting model dramatically reduces flakiness, and its tooling improves debugging and CI workflows. Whether used for testing, scraping, automation, or RL agents learning to perform tasks in real browsers, Playwright provides a powerful, modern foundation.

---
