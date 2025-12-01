# HEEx (HTML-Enhanced Elixir Templates)

HEEx (HTML-EEx) is Elixir’s next-generation templating format used in Phoenix LiveView and Phoenix components. It extends traditional `~E` / EEx templates with *compile-time safety*, *HTML validation*, and *LiveView-aware ergonomics*. HEEx ensures your UI layer is both expressive and type-safe (as much as Elixir can be), catching many errors before runtime.

---

## 🚀 Core Ideas

### **1. Compile-Time HTML Validation**
HEEx parses templates as real HTML at compile time. This means:
- Invalid nesting, mismatched tags, or stray attributes = compile error  
- Missing assigns = compile error  
- Unknown component slots = compile error  

This is one of the largest improvements over older EEx templates.

---

## 🧱 Basic Syntax

HEEx templates look like HTML with embedded Elixir expressions using `<%= %>` or `<% %>`.

Examples of useful inline expressions:
- Read a value: `<%= @user.name %>`
- Run a loop: `<%= for item <- @items do %> ... <% end %>`
- Call helpers: `<%= link "Home", to: "/" %>`

HEEx has *no* multiline code blocks outside `<% %>`—the template is still fundamentally HTML-first.

---

## 🧩 LiveView Integration

HEEx is designed for LiveView, enabling:
- Automatic diff tracking and efficient patch updates  
- Reactive assigns (`assign/3`, `assigns` map)  
- Event bindings (e.g., `phx-click`, `phx-submit`, `phx-change`)  

Example reactive binding:  
`<button phx-click="increment">+</button>`

HEEx + LiveView = reactive UI without writing any JS.

---

## 🧱 Component System

HEEx comes with a robust component system:

### **Function Components**
Lightweight, stateless components:  
`<.button label="Save" />`

A function component is just:
- a regular function  
- with `attr` declarations  
- optionally slots

### **Live Components**
Stateful components that handle events and manage internal updates.

Slots allow structured content injection:  
`<.modal><:title>My Title</:title>Inner content</.modal>`

---

## 📦 Assigns & Safety

HEEx enforces strict assign checking.  
Forgot to assign something? The compiler fails.

Example of a compile error:
- Template references `@user`
- But your LiveView or component never assigned `:user`

This eliminates whole categories of bugs common in older templates.

---

## 🎨 HEEx-Specific Enhancements

### **Attributes API**
Attributes can be:
- interpolated (`class={@dynamic_class}`)
- merged (`class="text-xl #{@something}"`)
- conditionally present (`disabled={@loading}`)

### **JS Commands (LiveView.JS)**
Native JS composable commands with syntax like:  
`phx-click={JS.toggle(to: "#menu")}`

These unify common UI actions with LiveView's DOM patches.

---

## 🧠 Why HEEx Over EEx?

| Feature | HEEx | EEx |
|--------|------|------|
| Compile-time HTML validation | ✔ | ✘ |
| Compile-time assign checking | ✔ | ✘ |
| LiveView-aware diffing | ✔ | ✘ |
| Components with attrs/slots | ✔ | ✘ |
| Safer and more ergonomic | ✔ | ~ |
| Designed for old server-render templates | ✘ | ✔ |

HEEx is the modern standard.  
EEx is mostly legacy unless generating arbitrary text.

---

## 📚 Patterns & Best Practices

### 🔍 Keep Logic Minimal
Use HEEx templates for display logic, not business logic.  
Heavy logic belongs in LiveView modules or components.

### 🧩 Prefer Function Components
Use Live Components only when you need state.  
Stateless function components scale better.

### 🧼 Use `attr` and `slot` Declarations
This unlocks:
- better error messages
- autocomplete in editors
- cleaner contracts

### 🧪 Test with `Phoenix.ComponentTest`
HEEx templates are fully testable.

---

## ⚡ Zig & HEEx?

While HEEx itself is Phoenix-specific, engineers mixing Elixir + Zig often use HEEx for:
- web interfaces  
- control panels for robotics, RL systems, or agents  
- dashboards showing `NIF`/port/Zig data  

HEEx can embed:
- binary data representation  
- JSON from Zig  
- structured state passed from BEAM processes  

Even though the templating has nothing to do with Zig directly, the workflow of “strong runtime with safe UI templating” is common in Elixir–Zig projects.

---

## 🏁 Summary

**HEEx is:**
- HTML-validated
- safer than EEx
- component-oriented
- LiveView-native
- the standard for Phoenix development

Use it anytime you're building UI in Elixir—especially LiveView apps.
