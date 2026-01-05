# TypeScript

**TypeScript** is a statically typed superset of [[JavaScript]] developed by Microsoft. It adds optional type annotations, interfaces, and compile-time checking to JavaScript, catching errors before runtime. TypeScript compiles down to plain JavaScript, making it compatible with any browser or runtime that supports JS.

---

## 📚 Overview

TypeScript addresses JavaScript's lack of static typing, which becomes problematic in large codebases. By adding types, developers get better IDE autocomplete, refactoring support, and early error detection. The language has become the default choice for large-scale frontend and backend JavaScript projects.

Key highlights:
- Compiles to JavaScript (runs anywhere JS runs)
- Gradual adoption (valid JS is valid TS)
- Strong tooling and IDE integration
- Backed by Microsoft with active development

---

## 🧠 Core Concepts

- **Type Annotations**
  Explicit types on variables, parameters, and return values: `function add(a: number, b: number): number`

- **Interfaces**
  Define object shapes and contracts for type checking

- **Generics**
  Reusable components that work with multiple types: `Array<T>`, `Promise<T>`

- **Union & Intersection Types**
  Combine types flexibly: `string | number`, `TypeA & TypeB`

- **Type Inference**
  Compiler deduces types when not explicitly annotated

- **Enums**
  Named constants for improved readability

- **Decorators**
  Metadata annotations for classes and methods (experimental)

---

## 📊 Comparison Chart

| Language | Typing | Compiles To | Runtime | Use Case |
|----------|--------|-------------|---------|----------|
| **TypeScript** | Static (optional) | JavaScript | Browser/Node/Bun/Deno | Large JS projects, full-stack |
| **JavaScript** | Dynamic | N/A (interpreted) | Browser/Node | Quick scripts, small projects |
| **Flow** | Static (optional) | JavaScript | Browser/Node | Facebook ecosystem |
| **Dart** | Static | JS or native | Browser/Flutter | Flutter apps, web |
| **CoffeeScript** | Dynamic | JavaScript | Browser/Node | Syntactic sugar (legacy) |
| **ReScript** | Static | JavaScript | Browser/Node | OCaml-like syntax for JS |
| **Elm** | Static | JavaScript | Browser | Functional frontend apps |

---

## 🔧 Use Cases

- Large-scale frontend applications (React, Angular, Vue)
- [[Node.js]] backend services and APIs
- Full-stack frameworks like Next.js, Remix
- Library and SDK development
- Game development with engines like [[Phaser]] or Kaplay
- CLI tools and build scripts

---

## ✅ Pros

- Catches type errors at compile time, not runtime
- Excellent IDE support (autocomplete, refactoring, go-to-definition)
- Self-documenting code through type annotations
- Gradual adoption - can migrate JS projects incrementally
- Large ecosystem with typed definitions (@types packages)
- Better maintainability for large codebases

---

## ❌ Cons

- Build step required (compilation)
- Learning curve for advanced type features
- Type definitions for third-party libs can lag or be incomplete
- Can lead to over-engineering with complex type gymnastics
- Slightly slower development iteration vs plain JS

---

## 🔧 Configuration

TypeScript uses `tsconfig.json` for project settings:

```json
{
  "compilerOptions": {
    "target": "ES2022",
    "module": "ESNext",
    "strict": true,
    "outDir": "./dist"
  },
  "include": ["src/**/*"]
}
```

Common compiler options:
- `strict` - Enable all strict type checks
- `target` - ECMAScript version to compile to
- `module` - Module system (CommonJS, ESNext)
- `noImplicitAny` - Error on implicit `any` types

---

## 🔩 Compatible Items

- [[Node.js]] - Server-side runtime
- [[Bun]] - Fast TypeScript-native runtime
- [[Deno]] - TypeScript-first runtime
- [[npm]] - Package manager
- [[Webpack]] - Bundler with TS support
- [[ESLint]] - Linting with @typescript-eslint
- [[Jest]] - Testing with ts-jest

---

## 🔗 Related Concepts

- [[JavaScript]] (Base language)
- [[Node.js]] (Server runtime)
- [[Elm]] (Alternative typed compile-to-JS)
- [[WebAssembly]] (Alternative compilation target)
- [[JSDoc]] (Type hints without TypeScript)

---

## 📚 External Resources

- [TypeScript Official Site](https://www.typescriptlang.org/)
- [TypeScript Handbook](https://www.typescriptlang.org/docs/handbook/)
- [DefinitelyTyped](https://github.com/DefinitelyTyped/DefinitelyTyped) - Type definitions repo
- [TypeScript Playground](https://www.typescriptlang.org/play)
