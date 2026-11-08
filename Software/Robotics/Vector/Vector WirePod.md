# Vector WirePod

**Vector WirePod** (`wire-pod`) is community server software providing a local backend for [[Anki Vector]], particularly voice-command services. It normally runs on an external host, not as a replacement robot operating system.

The upstream project documents support for production and developer-accessible Vector 1.0/2.0 units, with different robot preparation/authentication paths. Ordinary local-backend use is not itself a reason to perform a developer unlock.

---

## 🧩 Distinguish the Ecosystem Pieces

| Piece | Runs where / what it represents | Not equivalent to |
|---|---|---|
| WirePod backend | External computer/device | Robot OS image |
| Escape Pod-compatible firmware | On the robot | Root/developer access |
| Official Escape Pod product | Historical local-backend product/workflow | Every compatible community backend |
| WireOS | Robot Linux/application image | WirePod server |
| Python SDK client | Workstation API consumer | Backend or bootloader unlock |
| OSKR/generic developer unlock | Robot access/security state | Required prerequisite for every voice backend |

A setup working at one layer does not establish all the others.

---

## 🔄 Backend Relationship

After onboard wake-word handling, relevant robot communications reach the configured backend. WirePod provides server-side voice/intent functionality and related configuration/authentication services.

It is not a raw IMU acquisition service. Existing state access belongs to the SDK path, while richer internal telemetry requires the corresponding onboard producer/export work.

A local voice backend also does not prove the complete robot/host setup has zero Internet dependencies. Optional knowledge services, model downloads, updates and integrations have their own behavior and privacy implications.

---

## 🛠️ Robot Preparation Versus Host Installation

The installation wiki separates production robots from OSKR/developer-unlocked robots.

For the documented production path, it describes special DDL firmware compatible with Escape Pod and says an ordinary `2.0.*` version string alone is insufficient; the compatible build carries an `ep` suffix.

It distinguishes entering recovery from clearing user data, and includes user-data reset in its preparation workflow. That is a persistent/destructive setup action, not merely starting an external server.

Consult the applicable current instructions before migration. Do not copy historical commands, erase user data or re-pair a working SDK setup just to improve telemetry recording.

---

## 🖥️ Host and Network Requirements

Upstream installation options include Linux, Windows, macOS, Android and container workflows. Supported architecture/package details and permissions differ between them; use the current platform-specific guide rather than assuming one binary fits every ARM system.

The wiki highlights practical network issues:

- Avoid multiple active WirePod instances during setup.
- Avoid conflicting devices advertising the `escapepod` name.
- Ensure the robot and intended host can discover/reach one another.
- Container networking and mDNS need explicit consideration.
- Android background/battery restrictions can affect service lifetime.
- Privileged ports may be unavailable to an unprivileged host application.

A container starting successfully does not prove robot discovery, authentication or backend readiness.

---

## 🔐 Authentication and SDK Boundaries

The wiki separately authenticates production and unlocked bots with WirePod. Backend configuration, SDK application access and root SSH remain distinct capabilities.

Keep device identifiers, certificates, account material and tokens private. Treat shared configuration exports and diagnostic logs as potentially sensitive.

No router port forwarding or blanket firewall changes follow from the goal of local robot telemetry. Diagnose the actual local connection path instead.

---

## ⚖️ Strengths and Limits

**Strengths:** a community-maintained local voice backend, support for production units through compatible firmware, multiple host choices and separation from arbitrary onboard code execution.

**Limits:** host availability and network discovery matter; setup can involve firmware/user-data changes; speech models and optional integrations have separate requirements; backend compatibility is not evidence of developer/root access or new sensor APIs.

The project credits DDL's open-sourced Chipper code and Escape Pod work. That backend provenance should not be confused with licensing rights to an entire inherited robot OS image.

---

## 🔗 Related Notes

- [[Vector Escape Pod]] — compatibility terminology.
- [[Vector SDK]] — direct client-facing robot state and actions.
- [[Vector WireOS]] — onboard community firmware.
- [[Vector OSKR]] — developer access, separate from local voice service.
- [[Docker]] and [[systemd]] — host deployment and service lifetime.

---

## 📚 External Resources

- [WirePod source and README](https://github.com/kercre123/wire-pod).
- [Current installation wiki](https://github.com/kercre123/wire-pod/wiki/Installation) — production preparation, host options and distinct authentication paths.
- [Project wiki](https://github.com/kercre123/wire-pod/wiki) — troubleshooting and development.

These sources document the project; no host installation, robot migration or live backend test is asserted here.
