# gnome-screenshot
`gnome-screenshot` is the canonical GNOME Desktop screenshot utility. It works with Wayland or X11. Behavior depends on the compositor, the current GTK stack, and what desktop shortcuts are already mapped to PrintScreen keys.

---

## 🧰 Core Concepts
- It captures monitor output as surfaces the compositor exposes
- Most people think it’s a GUI only – but the real power is the CLI tool `gnome-screenshot`
- It can be bound to custom hotkeys, which is far more useful for power users

---

## 🧪 Key CLI Options
(`inline code` form only, to keep this compliant)

- `gnome-screenshot -a` = interactively select an area
- `gnome-screenshot -w` = screenshot the currently focused window
- `gnome-screenshot -f file.png` = write to a specific file path
- `gnome-screenshot -d 3` = delay N seconds before capture
- `gnome-screenshot -i` = open the interactive GUI
- `gnome-screenshot --include-pointer` = include the mouse cursor pointer
- `gnome-screenshot --remove-border` = remove window border for focused-window shots
- `gnome-screenshot --border-effect=shadow` = add border styling to window capture

---

## 📝 Example Configs that matter
Good power combos:
- Entire screen to clipboard: `gnome-screenshot --clipboard`
- Area selection to clipboard: `gnome-screenshot -a --clipboard`
- Current window delayed to clipboard: `gnome-screenshot -w -d 2 --clipboard`

---

## 🎛️ Custom Keyboard Shortcut in GNOME (Wayland or X11)
This is *the* power move

1. GNOME Settings
2. Keyboard
3. View and Customize Shortcuts
4. Find the PrintScreen section
5. Disable the existing bindings (set them to *Disabled*)
6. Now add a *Custom Shortcut*
7. Name: `Area to Clipboard`
8. Command: `gnome-screenshot -a --clipboard`
9. Set shortcut: `Shift+Print` (or whatever you want)

*If you don’t disable the existing PrintScreen ones, GNOME will intercept and your custom commands won’t work.*

---

## 🔍 Comparison Chart
| Tool / Concept | Area Select | Clipboard | Window | Multi-monitor Aware | GUI Configurable | Notes |
|---|---|---|---|---|---|---|
| gnome-screenshot | ✅ | ✅ | ✅ | ✅ | Yes (with `-i`) | Default GNOME tool |
| flameshot | ✅ | ✅ | ✅ | ✅ | Yes | More annotation features |
| grim + slurp | ✅ | ✅ | ✅ | ✅ | No | Wayland command-line power combo |
| spectacle (KDE) | ✅ | ✅ | ✅ | ✅ | Yes | KDE default |
| scrot | ✅ | ❌ | ✅ | ❌ | No | Old, X11 only primarily |

---

## ✅ Strengths
- Installed by default on most GNOME systems
- CLI makes it scriptable
- Good for robotic development where you need automated capture of UI

---

## ❌ Weaknesses
- Has weaker annotation vs e.g. flameshot
- Many GNOME distros override/alias PrintScreen in weird ways, requiring cleanup

---

## 🧠 Related Concepts / Notes
- [[Wayland]] (display protocol)
- [[X11]] (older display system)
- [[Flameshot]] (alternative screenshot tool with annotation)
- [[Shell]] (Bash, Zsh, etc for scripting)
- [[GNOME]] (desktop environment)

---
