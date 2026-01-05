# Phaser

**Phaser** is a mature, feature-rich 2D game framework for [[JavaScript]] and [[TypeScript]]. It provides a complete toolkit for building browser-based games with WebGL and Canvas rendering, physics engines, audio, input handling, and asset management. Phaser is one of the most widely used open-source HTML5 game frameworks.

---

## 📚 Overview

Created by Richard Davey (Photon Storm) in 2013, Phaser has evolved through major versions with Phaser 3 being the current stable release. It's designed for both beginners and professional developers, powering thousands of games from prototypes to commercial releases.

Key highlights:
- WebGL with Canvas fallback for broad compatibility
- Multiple physics systems (Arcade, Matter.js, Impact)
- Comprehensive plugin ecosystem
- Strong documentation and community
- Battle-tested in production games

---

## 🧠 Core Concepts

- **Scenes**
  Container for game logic, replacing Phaser 2's states: `class GameScene extends Phaser.Scene`

- **Game Objects**
  Sprites, text, graphics, tilemaps, particles, and more

- **Physics Bodies**
  Arcade (simple/fast), Matter.js (realistic), or custom

- **Loaders**
  Asset pipeline for images, spritesheets, audio, tilemaps, JSON

- **Tweens & Timers**
  Animation and timing utilities built-in

- **Input**
  Keyboard, mouse, touch, and gamepad support

- **Cameras**
  Multiple cameras, follow, zoom, effects, and viewports

---

## 📊 Comparison Chart

| Engine | Language | Physics | Complexity | Community | Best For |
|--------|----------|---------|------------|-----------|----------|
| **Phaser** | JS/TS | Arcade, Matter.js | Moderate | Very large | Production 2D games |
| **Kaplay** | JS/TS | Built-in | Simple | Growing | Prototypes, jams |
| **PixiJS** | JS/TS | None (add-on) | Moderate | Large | Rendering, custom engines |
| **Godot** | GDScript/C# | Built-in | Moderate | Large | Full game engine |
| **Construct** | Visual/JS | Built-in | Simple | Moderate | No-code/low-code |
| **GameMaker** | GML | Built-in | Moderate | Large | Indie 2D games |
| **Cocos2d-x** | C++/JS | Box2D | Complex | Large | Mobile games |

---

## 🔧 Use Cases

- Commercial browser and mobile games
- Game jams with polished output
- Educational games and simulations
- Advergames and promotional content
- Porting retro games to web
- [[PWA]] game distribution

---

## ✅ Pros

- Mature, stable, production-ready
- Excellent documentation and examples
- Large plugin ecosystem
- Multiple physics engine options
- Active development and community
- Works with modern tooling (Webpack, Vite, [[Bun]])

---

## ❌ Cons

- Larger bundle size than minimal frameworks
- Learning curve for Scene lifecycle
- Phaser 2 to 3 migration was breaking
- WebGL-first can complicate some Canvas-only needs
- No built-in UI system (community solutions exist)

---

## 🔧 Quick Start

```typescript
import Phaser from "phaser";

class GameScene extends Phaser.Scene {
  constructor() {
    super("game");
  }

  preload() {
    this.load.image("logo", "assets/logo.png");
  }

  create() {
    const logo = this.add.image(400, 300, "logo");
    this.tweens.add({
      targets: logo,
      y: 450,
      duration: 2000,
      ease: "Power2",
      yoyo: true,
      loop: -1
    });
  }
}

new Phaser.Game({
  type: Phaser.AUTO,
  width: 800,
  height: 600,
  scene: GameScene
});
```

---

## 🔩 Compatible Items

- [[TypeScript]] - Full type definitions
- [[Webpack]] - Common bundler choice
- [[Bun]] - Fast dev server
- [[Tiled]] - Tilemap editor integration
- [[Texture Packer]] - Spritesheet generation
- [[Matter.js]] - Advanced physics
- [[Kenney Assets]] - CC0 game art

---

## 🔗 Related Concepts

- [[Kaplay]] (Simpler alternative)
- [[JavaScript]] (Base language)
- [[TypeScript]] (Typed development)
- [[Game Development]] (Parent topic)
- [[WebGL]] (Rendering backend)
- [[PWA]] (Distribution method)

---

## 📚 External Resources

- [Phaser Official Site](https://phaser.io/)
- [Phaser 3 Documentation](https://photonstorm.github.io/phaser3-docs/)
- [Phaser 3 Examples](https://phaser.io/examples)
- [Phaser GitHub](https://github.com/photonstorm/phaser)
- [Phaser Discord](https://discord.gg/phaser)
