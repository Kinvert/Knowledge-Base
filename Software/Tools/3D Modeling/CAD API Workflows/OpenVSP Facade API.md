# OpenVSP Facade API

The **Facade API** is an OpenVSP execution mode for the Python API that routes calls through a local socket server to a separate OpenVSP process. It exists to support GUI+Python co-running workflows and multiple isolated VSP instances from Python.

It does not change your geometry API names; it changes **how** those APIs are hosted and called.

---

## Why it matters

In normal API mode, calls are in-process and fast. In Facade mode, calls cross a local process boundary.

That is useful when:

- You want to keep a custom Python GUI app responsive while OpenVSP is active.
- You need non-blocking OpenVSP GUI + scripted control in the same process.
- You want multiple independent VSP instances in one Python job.

Tradeoff: facade adds process/sockets overhead and startup latency compared with in-process API.

---

## Control flags (`openvsp_config`)

The Python import behavior is controlled through `openvsp_config` because `openvsp` options are not passable directly at import time.

| Flag | Default | What it does |
|---|---|---|
| `LOAD_GRAPHICS` | `False` | Load OpenVSP graphics subsystem. |
| `LOAD_FACADE` | `False` | Route API through Facade IPC process. |
| `LOAD_MULTI_FACADE` | `False` | Enables `vsp_servers` helpers for multiple VSP instances. |

If you do not import `openvsp_config`, defaults are applied (`False` for both primary flags in current docs).

### 1) Normal API (default)

```python
import openvsp_config
openvsp_config.LOAD_GRAPHICS = False
openvsp_config.LOAD_FACADE = False
import openvsp as vsp
```

### 2) OpenVSP API with GUI

```python
import openvsp_config
openvsp_config.LOAD_GRAPHICS = True
openvsp_config.LOAD_FACADE = False
import openvsp as vsp

vsp.ReadVSPFile("aircraft.vsp3")
vsp.InitGUI()
vsp.StartGUI()
```

`StartGUI()` blocks program flow while the OpenVSP UI is running.

### 3) Facade mode (headless API in separate process)

```python
import openvsp_config
openvsp_config.LOAD_GRAPHICS = False
openvsp_config.LOAD_FACADE = True
import openvsp as vsp
```

Use this for batch/headless automation where you want the Python process isolated from OpenVSP process internals.

### 4) GUI + Facade (non-blocking GUI + Python control)

```python
import openvsp_config
openvsp_config.LOAD_GRAPHICS = True
openvsp_config.LOAD_FACADE = True
import openvsp as vsp

vsp.ReadVSPFile("aircraft.vsp3")
vsp.InitGUI()
vsp.StartGUI()  # GUI stays interactive while Python continues
```

This is the key mode when a user-facing GUI and scripted parameter sweeps should coexist.

---

## MultiFacade API

Because a single process can host one OpenVSP instance, Facade mode is also the path to multiple independent instances:

```python
import openvsp_config
openvsp_config.LOAD_MULTI_FACADE = True
import openvsp as vsp_multi

vsp_a = vsp_multi.vsp_servers.start_vsp_instance()
vsp_b = vsp_multi.vsp_servers.start_vsp_instance()

vsp_a.ReadVSPFile(r"model_a.vsp3")
vsp_b.ReadVSPFile(r"model_b.vsp3")

print("A geoms:", vsp_a.FindGeoms())
print("B geoms:", vsp_b.FindGeoms())
```

From the docs, `vsp_servers` exposes instance control helpers:

- `start_vsp_instance(name=None, port=-1)` to start an additional server-backed OpenVSP instance.
- `get_vsp_instance(name)` for name-based lookup in that server set.
- `get_vsp_instance(name=None, port=None)` to stop a specific instance by name or port (as documented).

Name and port controls are useful for long-running optimization farms.

---

## Additional Facade options

When using facade communication, these options are available on `openvsp_config`:

- `FACADE_PORT` (default `-1`): fixed TCP/IPC port or dynamic.
- `FACADE_SERVER_TIMEOUT` (default `0.1`): socket connect timeout in seconds.
- `FACADE_SERVER_ATTEMPTS` (default `10`): retry count for server connection.
- `FACADE_PRINT_LEVEL` (`0` none, `1` info, `2` debug).

---

## Introspection checks you can script

Use these to make startup behavior explicit in logs:

- `vsp.IsFacade()` → `True` when facade transport is active.
- `vsp.IsGUIBuild()` → whether GUI-capable build is available.
- `vsp.IsGUIRunning()` and `vsp.IsEventLoopRunning()` → GUI control state.

---

## Common gotchas

- Set `openvsp_config` **before** importing `openvsp`; changing it after import has no effect.
- Keep port settings stable for strict CI or multi-tenant nodes.
- Facade is usually slower per call than in-process mode; use it for orchestration, not micro-burst API loops.
- For automation stacks, ensure Python version compatibility with your OpenVSP binary build (official docs stress this in installation guidance).

---

## Practical workflow decision (OpenVSP)

| Situation | Recommended mode | Why |
|---|---|---|
| Batch DOE loop / optimization script | `LOAD_FACADE=True`, `LOAD_GRAPHICS=False` | Deterministic headless process with isolated OpenVSP instance |
| Human-in-loop GUI + scripted edits | `LOAD_GRAPHICS=True`, `LOAD_FACADE=True` | Keeps OpenVSP responsive while Python control continues |
| Pure local script, no GUI | both flags `False` | Lowest overhead |
| Comparing design families in parallel | `LOAD_MULTI_FACADE=True` + `vsp_servers` | Independent model state per instance |

---

## Related notes

- [[OpenVSP CFD Workflow]]
- [[CAD APIs for CFD-Driven Parametric Design Loops]]

## Further reading

- OpenVSP config/docs: https://openvsp.org/pyapi_docs/latest/openvsp_config.html
- OpenVSP Python API docs: https://openvsp.org/pyapi_docs/latest/index.html
- OpenVSP python package docs: https://openvsp.org/pyapi_docs/latest/openvsp.html
- OpenVSP API docs: https://openvsp.org/api_docs/latest/index.html
