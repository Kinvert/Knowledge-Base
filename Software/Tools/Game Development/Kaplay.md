# Kaplay

**Kaplay** is a [[JavaScript]]/[[TypeScript]] 2D game library focused on simplicity and rapid prototyping. It's the successor to Kaboom.js, offering a clean API for creating browser-based games with minimal boilerplate. Kaplay handles sprites, animations, physics, input, audio, and scenes out of the box.

---

## 📚 Overview

Kaplay was forked from Kaboom.js after Replit stopped active development. The community continued the project under the Kaplay name, maintaining the same philosophy of beginner-friendly game development with a fun, expressive API.

Key highlights:
- Simple, declarative API for game objects
- Built-in scene management and state handling
- Physics and collision detection included
- Works with [[Bun]], [[Node.js]], or browser directly
- First-class TypeScript support

---

## 🧠 Core Concepts

- **Game Objects**
  Everything is a game object with components: `add([sprite("player"), pos(100, 100), area()])`

- **Components**
  Modular behaviors attached to objects: `sprite()`, `pos()`, `area()`, `body()`, `health()`

- **Scenes**
  Organize game states: `scene("game", () => {...})`, `go("game")`

- **Tags**
  Label objects for collision and queries: `add([sprite("enemy"), "enemy"])`

- **Events**
  React to collisions, input, timers: `onCollide("player", "enemy", () => {...})`

- **Layers**
  Control rendering order for backgrounds, entities, UI

---

## 📊 Comparison Chart

| Engine | Language | 2D/3D | Complexity | Best For |
|--------|----------|-------|------------|----------|
| **Kaplay** | JS/TS | 2D | Simple | Prototypes, jam games, learning |
| **Phaser** | JS/TS | 2D | Moderate | Polished 2D games, production |
| **PixiJS** | JS/TS | 2D | Moderate | Rendering-focused, custom engines |
| **Godot** | GDScript/C# | 2D/3D | Moderate | Indie games, full features |
| **Unity** | C# | 2D/3D | Complex | Production games, cross-platform |
| **LÖVE** | Lua | 2D | Simple | Lightweight 2D games |
| **Pygame** | Python | 2D | Simple | Learning, prototypes |

---

## 🔧 Use Cases

- Game jams and rapid prototyping
- Learning game development concepts
- Browser-based casual games
- [[PWA]] (Progressive Web App) games
- Educational projects and tutorials
- Small indie releases

---

## ✅ Pros

- Extremely beginner-friendly API
- Zero configuration to start
- Native TypeScript support
- Fast iteration and hot reloading
- Active community (post-Kaboom fork)
- Comprehensive built-in features

---

## ❌ Cons

- Limited to 2D games
- Less suited for large, complex productions
- Smaller ecosystem than Phaser or Unity
- Browser-only (no native builds)
- Performance ceiling for graphics-heavy games

---

## 🔧 Quick Start

```typescript
import kaplay from "kaplay";

kaplay();

loadSprite("bean", "sprites/bean.png");

scene("main", () => {
  const player = add([
    sprite("bean"),
    pos(center()),
    area(),
    body(),
  ]);

  onKeyPress("space", () => {
    player.jump();
  });
});

go("main");
```

---

## 🔩 Compatible Items

- [[TypeScript]] - First-class support
- [[Bun]] - Fast runtime for dev server
- [[Node.js]] - Alternative runtime
- [[PWA]] - Offline-capable game distribution
- [[GitHub Pages]] - Free hosting for web games
- [[Kenney Assets]] - CC0 sprites and art

---

## 🔗 Related Concepts

- [[Phaser]] (More feature-rich 2D engine)
- [[JavaScript]] (Base language)
- [[TypeScript]] (Typed development)
- [[Game Development]] (Parent topic)
- [[PWA]] (Distribution method)

---

## 📚 External Resources

- [Kaplay Official Site](https://kaplayjs.com/)
- [Kaplay Documentation](https://kaplayjs.com/doc/)
- [Kaplay GitHub](https://github.com/kaplayjs/kaplay)
- [Kaboom.js (predecessor)](https://kaboomjs.com/)
- [Kaplay Playground](https://kaplayjs.com/play/)
