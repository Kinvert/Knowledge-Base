---
title: GNOME Settings Commands
tags: [linux, gnome, ubuntu, desktop, cli, settings]
aliases: [gsettings, dconf, GNOME CLI Settings, Ubuntu Dock Commands, Dash to Dock Commands]
---

# GNOME Settings Commands

**GNOME settings commands** are CLI tools for reading and changing desktop preferences that are usually hidden behind GNOME Settings, GNOME Tweaks, GNOME Extensions, or distro-specific panels like the Ubuntu Dock settings. The most important tools are `gsettings`, `dconf`, and `gnome-extensions`.

---

## 🧠 Summary

- `gsettings` is the safest first tool: schema-aware, validates keys, and knows allowed values.
- `dconf` is lower-level: path-based, useful for backup/restore and bulk edits.
- `gnome-extensions` manages installed [[GNOME]] Shell extensions.
- Most changes are per-user and take effect immediately.
- These commands usually require a graphical login session because they write through the user session bus.

---

## 🔧 Driving Example: Ubuntu Dock Click Behavior

On Ubuntu 24.04, the dock is based on Dash-to-Dock/Ubuntu Dock settings. If clicking a dock icon no longer previews multiple windows, set the dock click action:

```bash
gsettings set org.gnome.shell.extensions.dash-to-dock click-action 'focus-or-previews'
```

Useful alternatives:

| Value | Behavior |
|-------|----------|
| `focus-or-previews` | Focuses the app, or shows window previews when multiple windows exist |
| `cycle-windows` | Cycles through windows on repeated clicks |
| `minimize` | Minimizes the focused window on click |
| `minimize-or-previews` | Minimizes a single focused window, previews when multiple windows exist |
| `focus-minimize-or-previews` | Focus, minimize, or preview depending on current state |
| `focus-or-appspread` | Focuses the app or shows an app spread-style view |

Discover valid values on your system:

```bash
gsettings range org.gnome.shell.extensions.dash-to-dock click-action
```

Reset to distro default:

```bash
gsettings reset org.gnome.shell.extensions.dash-to-dock click-action
```

---

## ⚙️ Command Breakdown

```bash
gsettings set org.gnome.shell.extensions.dash-to-dock click-action 'focus-or-previews'
```

| Part | Meaning |
|------|---------|
| `gsettings` | GNOME command-line settings tool |
| `set` | Write a new value |
| `org.gnome.shell.extensions.dash-to-dock` | Schema ID for Ubuntu Dock / Dash-to-Dock settings |
| `click-action` | Key inside that schema |
| `'focus-or-previews'` | New value, written as a GVariant string |

Strings need quotes. Booleans use `true` or `false`. Arrays look like `['item1', 'item2']`.

---

## 🧰 Common `gsettings` Commands

| Task | Command |
|------|---------|
| Read one key | `gsettings get SCHEMA KEY` |
| Set one key | `gsettings set SCHEMA KEY VALUE` |
| Reset one key | `gsettings reset SCHEMA KEY` |
| Show allowed values | `gsettings range SCHEMA KEY` |
| Describe a key | `gsettings describe SCHEMA KEY` |
| List schemas | `gsettings list-schemas` |
| List keys in a schema | `gsettings list-keys SCHEMA` |
| Dump a schema | `gsettings list-recursively SCHEMA` |
| Watch live changes | `gsettings monitor SCHEMA` |
| Check if writable | `gsettings writable SCHEMA KEY` |

Good discovery pattern:

```bash
gsettings list-schemas | grep -i dock
gsettings list-recursively org.gnome.shell.extensions.dash-to-dock
gsettings describe org.gnome.shell.extensions.dash-to-dock click-action
```

---

## 🧱 `dconf` vs `gsettings`

| Tool | Level | Best For | Example |
|------|-------|----------|---------|
| `gsettings` | Schema-aware | Normal setting changes | `gsettings set org.gnome.desktop.interface color-scheme 'prefer-dark'` |
| `dconf` | Raw settings database | Backup, restore, watching paths | `dconf dump /org/gnome/` |
| `dconf-editor` | GUI over dconf | Browsing settings visually | Open app, search for key |
| GNOME Settings | Official GUI | Supported common settings | Display, keyboard, mouse |
| GNOME Tweaks | Extra GUI | Fonts, themes, window behavior | Shell/user interface tweaks |

Backup GNOME settings:

```bash
dconf dump / > gnome-settings.dconf
```

Restore them:

```bash
dconf load / < gnome-settings.dconf
```

Watch what changes while using the GUI:

```bash
dconf watch /
```

---

## 🧩 GNOME Extension Commands

| Task | Command |
|------|---------|
| List extensions | `gnome-extensions list` |
| List enabled extensions | `gnome-extensions list --enabled` |
| Show extension info | `gnome-extensions info EXTENSION_UUID` |
| Enable extension | `gnome-extensions enable EXTENSION_UUID` |
| Disable extension | `gnome-extensions disable EXTENSION_UUID` |
| Open extension preferences | `gnome-extensions prefs EXTENSION_UUID` |

Example:

```bash
gnome-extensions list | grep -i dock
gnome-extensions prefs ubuntu-dock@ubuntu.com
```

Extension UUIDs vary by distro and install method. Use `gnome-extensions list` instead of guessing.

---

## 🧪 Useful Settings You May See Later

| Area | Example Command | Notes |
|------|-----------------|-------|
| Dark mode | `gsettings set org.gnome.desktop.interface color-scheme 'prefer-dark'` | Common GNOME 42+ setting |
| Text scaling | `gsettings set org.gnome.desktop.interface text-scaling-factor 1.25` | Useful on high-DPI displays |
| Clock seconds | `gsettings set org.gnome.desktop.interface clock-show-seconds true` | Top bar clock |
| Show weekday | `gsettings set org.gnome.desktop.interface clock-show-weekday true` | Top bar clock |
| Disable hot corners | `gsettings set org.gnome.desktop.interface enable-hot-corners false` | Avoid accidental overview |
| Workspaces on primary only | `gsettings set org.gnome.mutter workspaces-only-on-primary true` | Multi-monitor behavior |
| App switcher current workspace | `gsettings set org.gnome.shell.app-switcher current-workspace-only true` | Alt-Tab scope |
| Show hidden files | `gsettings set org.gtk.gtk4.Settings.FileChooser show-hidden true` | GTK4 file chooser |
| Legacy file chooser hidden files | `gsettings set org.gtk.Settings.FileChooser show-hidden true` | Older GTK apps |
| Reset interface schema | `gsettings reset-recursively org.gnome.desktop.interface` | Broad reset; use carefully |

---

## ⚠️ Pitfalls

- Some keys do not exist unless the matching app or extension is installed.
- Ubuntu Dock settings live under `org.gnome.shell.extensions.dash-to-dock`, even though the visible UI says Ubuntu Dock.
- `gsettings` values use GVariant syntax, so shell quoting matters.
- `sudo gsettings ...` usually edits root's settings, not your desktop user's settings.
- Bulk `reset-recursively` and `dconf load /` can wipe custom desktop settings.
- GNOME Shell extensions can break after major GNOME upgrades; disable them first when debugging desktop glitches.

---

## 🔗 Related Notes

- [[Ubuntu]]
- [[Linux]]
- [[gnome-screenshot]]
- [[dbus-x11]]
- [[Wayland]]
- [[X11]]
- [[Shell]]

---

## 🌐 External Resources

- [gsettings man page](https://man.archlinux.org/man/gsettings.1.en)
- [GNOME dconf overview](https://wiki.gnome.org/Projects/dconf)
- [Red Hat: Working with GSettings keys on the command line](https://docs.redhat.com/en/documentation/red_hat_enterprise_linux/8/html/using_the_desktop_environment_in_rhel_8/working-with-gsettings-keys-on-command-line_using-the-desktop-environment-in-rhel-8)
- [Red Hat: Managing GNOME Shell extensions by command line](https://docs.redhat.com/en/documentation/red_hat_enterprise_linux/9/html/customizing_the_gnome_desktop_environment/managing-gnome-shell-extensions-via-command-line_customizing-the-gnome-desktop-environment)
- [Ask Ubuntu: enable minimize on click in Ubuntu Dock](https://askubuntu.com/questions/960074/how-do-i-enable-minimize-on-click-on-ubuntu-dock-in-17-10-and-later)
- [Launchpad: Ubuntu Dock click-action preview bug](https://bugs.launchpad.net/ubuntu/+source/gnome-shell-extension-ubuntu-dock/+bug/1947445)
