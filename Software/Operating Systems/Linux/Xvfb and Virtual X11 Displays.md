# Xvfb and Virtual X11 Displays

**Xvfb** is the **X virtual framebuffer**: an X11 display server that runs without a real monitor, GPU display output, keyboard, or mouse. It gives programs a fake X11 display backed by memory.

**`xvfb-run`** is a convenience wrapper that starts Xvfb, sets `DISPLAY`, runs your command, then cleans the temporary X server up afterward.

Use it when a GUI program, test suite, browser, screenshot tool, CAD utility, or legacy app refuses to run because there is no X display.

---

## Mental Model

Normal desktop:

```text
GUI app -> DISPLAY=:0 -> real Xorg/Wayland/Xwayland session -> monitor
```

Xvfb:

```text
GUI app -> DISPLAY=:99 -> Xvfb -> framebuffer in RAM -> no visible monitor
```

The app thinks it is drawing windows. Xvfb accepts the X11 protocol calls and renders into memory instead of a physical display.

---

## Fast Start

Install:

```bash
sudo apt install xvfb xauth
sudo dnf install xorg-x11-server-Xvfb xorg-x11-xauth
sudo pacman -S xorg-server-xvfb xorg-xauth
```

Run one command in a fake display:

```bash
xvfb-run my-gui-command
```

Run with a specific screen size and color depth:

```bash
xvfb-run -a -s "-screen 0 1920x1080x24" my-gui-command
```

Run browser tests:

```bash
xvfb-run -a npm test
xvfb-run -a pytest
xvfb-run -a playwright test
```

Run a single X client:

```bash
xvfb-run -a xclock
```

---

## `xvfb-run`

`xvfb-run` is usually what you want for scripts and CI.

It does this:

1. Creates a temporary Xauthority file.
2. Starts `Xvfb` in the background.
3. Sets `DISPLAY` for the child command.
4. Runs your command.
5. Kills Xvfb when the command exits.
6. Exits with the wrapped command's exit status.

Useful flags:

| Flag | Meaning |
|---|---|
| `-a`, `--auto-servernum` | Pick a free display number automatically. Use this in CI. |
| `-n N`, `--server-num=N` | Start at display number `N`; default is usually `99`. |
| `-s ARGS`, `--server-args=ARGS` | Pass arguments to `Xvfb`. |
| `-e FILE`, `--error-file=FILE` | Write Xvfb/xauth errors somewhere visible. |
| `-f FILE`, `--auth-file=FILE` | Use a specific Xauthority file. |
| `-l`, `--listen-tcp` | Allow TCP listening. Usually avoid this. |

Most common reliable form:

```bash
xvfb-run -a -s "-screen 0 1280x1024x24" command args...
```

Debug stderr:

```bash
xvfb-run -a -e /dev/stderr -s "-screen 0 1280x1024x24" command args...
```

---

## Raw Xvfb

Use raw `Xvfb` when you want to keep the fake display alive across multiple commands.

Start server:

```bash
Xvfb :99 -screen 0 1920x1080x24 -nolisten tcp &
export DISPLAY=:99
```

Run apps against it:

```bash
my-gui-command
xdpyinfo
```

Stop it:

```bash
pkill -f "Xvfb :99"
```

Safer script pattern:

```bash
Xvfb :99 -screen 0 1920x1080x24 -nolisten tcp &
XVFB_PID=$!
export DISPLAY=:99

command-that-needs-x11

kill "$XVFB_PID"
```

---

## What It Is Good For

- Running GUI tests in CI.
- Running old GUI apps on servers.
- Running browser tests that still need X11.
- Generating screenshots or PDFs from GUI renderers.
- Running tools like `matplotlib`, Qt, GTK, Tk, SWT, Electron, or Java AWT in headless environments.
- Testing X11 apps against unusual screen sizes or color depths.
- Running commands over SSH without X forwarding.

---

## What It Is Not

- It is not a full remote desktop by itself.
- It does not make windows visible unless paired with another tool.
- It is usually not hardware accelerated.
- It is not a Wayland compositor.
- It is not the same thing as browser-native headless mode.
- It does not fix apps that require real GPU/OpenGL features.

If you need to see or interact with the session, use Xpra, VNC, Xephyr, or a real Xorg session instead.

---

## Related Tools

| Tool | What it does | When to use |
|---|---|---|
| `Xvfb` | Headless in-memory X11 server | CI, tests, fake display |
| `xvfb-run` | Wrapper around Xvfb | One-shot commands |
| `Xdummy` | Xorg with dummy video driver | More complete virtual Xorg; sometimes better than Xvfb |
| `Xephyr` | Nested X server inside another X display | Test desktop sessions visibly |
| `Xnest` | Older nested X server | Legacy nested X11 testing |
| `Xpra` | Persistent/attachable remote X sessions | Headless apps you may later view |
| `x11vnc` | Expose an existing X display over VNC | Remote viewing/control |
| `TigerVNC` / `Xvnc` | Virtual X desktop served over VNC | Full remote desktop |
| `weston --backend=headless` | Headless Wayland compositor | Wayland-native headless tests |
| `chromium --headless` | Browser without X display | Modern browser automation |

---

## Xvfb vs Browser Headless

For modern browser testing, prefer native headless mode when it works:

```bash
chromium --headless=new --screenshot https://example.com
playwright test
```

Use Xvfb when:

- The browser or test stack needs a real-ish X11 display.
- The app uses Electron, GTK, Qt, Java AWT, or native file dialogs.
- You need to test window behavior.
- A library checks `DISPLAY` and crashes without it.

---

## Xvfb vs Xpra vs VNC

Use `xvfb-run` for disposable command execution:

```bash
xvfb-run -a command
```

Use raw `Xvfb` for a persistent invisible display:

```bash
Xvfb :99 -screen 0 1920x1080x24 &
export DISPLAY=:99
```

Use `xpra` when you want to detach and later attach to GUI apps:

```bash
xpra start --start=firefox
xpra attach
```

Use VNC when you want a full remote desktop:

```bash
vncserver :1 -geometry 1920x1080 -depth 24
```

---

## Common Problems

| Error | Meaning | Fix |
|---|---|---|
| `cannot open display` | `DISPLAY` not set or X server not running | Use `xvfb-run -a command` |
| `xvfb-run: error: xauth command not found` | Missing `xauth` | Install `xauth` |
| `Xvfb failed to start` | Display number conflict or bad server args | Use `-a -e /dev/stderr` |
| `Server is already active for display` | Another X server owns that display number | Use `xvfb-run -a` or another `:N` |
| Tiny or weird screenshots | Default screen too small or 8-bit depth | Set `-screen 0 1920x1080x24` |
| OpenGL/GLX failures | No hardware acceleration or missing Mesa libs | Use software rendering or real/virtual GPU |
| Fonts look wrong | Missing font packages | Install distro font packages |
| Program hangs forever | GUI app opened a dialog you cannot see | Use VNC/Xpra or logs/screenshots |

---

## Docker Pattern

Install dependencies in the image:

```dockerfile
RUN apt-get update && apt-get install -y xvfb xauth
```

Run:

```bash
xvfb-run -a -s "-screen 0 1920x1080x24" npm test
```

For Chromium/Electron in containers, you may also need:

```bash
--disable-dev-shm-usage
```

or a larger shared memory mount:

```bash
docker run --shm-size=2g ...
```

---

## Security Notes

- Prefer `-nolisten tcp`; `xvfb-run` disables TCP listening by default.
- Do not expose X11 displays to untrusted users. X11 has a weak security model once a client can connect.
- Avoid `xhost +`; it allows broad access to your X server.
- Use temporary Xauthority cookies, which `xvfb-run` handles automatically.
- For CI, use `xvfb-run -a` to avoid display-number collisions between jobs.

---

## Quick Recipes

Run a flaky GUI test suite:

```bash
xvfb-run -a -e /dev/stderr -s "-screen 0 1920x1080x24" pytest
```

Generate a screenshot with a GUI browser/tool:

```bash
xvfb-run -a -s "-screen 0 1366x768x24" cutycapt --url=https://example.com --out=page.png
```

Run a Java GUI app headlessly:

```bash
xvfb-run -a java -jar app.jar
```

Run an Electron app in CI:

```bash
xvfb-run -a npm run test:e2e
```

Keep a fake display alive:

```bash
Xvfb :99 -screen 0 1920x1080x24 -nolisten tcp &
export DISPLAY=:99
```

Confirm the display:

```bash
xdpyinfo | head
```

---

## Related Concepts

- [[Linux]]
- [[TUI]]
- [[Docker Container]]
- [[CI-CD]]
- [[Wayland]]
- [[X11]]
- [[VNC]]
- [[Chromium]]
- [[Playwright]]

---

## Further Reading

- [X.Org Xvfb manual](https://www.x.org/archive/X11R7.5/doc/man/man1/Xvfb.1.html)
- [Debian xvfb-run man page](https://manpages.debian.org/testing/xvfb/xvfb-run.1.en.html)
- [Xpra manual](https://xpra.org/manual)
- [Arch Wiki: Xvfb](https://wiki.archlinux.org/title/Xvfb)
- [X.Org Xserver manual](https://www.x.org/releases/X11R7.7/doc/man/man1/Xserver.1.xhtml)
