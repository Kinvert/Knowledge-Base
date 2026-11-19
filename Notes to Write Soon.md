# Notes to Write Soon

## Robotics fundamentals backlog

Focus on worked examples, implementation decisions, and failure diagnosis alongside the existing concept comparisons. Choose locations before creating new notes; expand existing notes where indicated.

### First: foundational notes to add

- [[Robot Coordinate Frames]] — transform composition/inversion, source/target conventions, quaternion ordering, camera optical frames, and REP-103/105; work through a camera-to-base point transformation.
- [[Forward Kinematics]] and [[Jacobian]] — derive a two-link arm, distinguish geometric and analytic Jacobians, explain singularities with rank/SVD, and verify a Jacobian with finite differences.
- [[TF2]] — timestamped transforms, static versus dynamic publishers, map → odom → base_link, buffer/extrapolation errors, and simulated-time debugging.
- [[ROS2 Executors and Callback Groups]] — mutually exclusive/reentrant groups, futures, service-call deadlocks, composition, and shared-state synchronization.
- [[Navigation2]] — connect localization, planners, controllers, lifecycle nodes, behavior-tree recovery, and costmaps; include marking/clearing, footprint/inflation, and stale-obstacle diagnosis.
- [[robot_localization]] and [[AMCL]] — compare continuous sensor fusion with map-based localization; cover frame ownership, covariance, initialization, and failure recovery.
- [[rosbag2]] — recording and replay, QoS overrides, simulated time, TF capture, storage choices, and reproducing an intermittent robot failure.
- [[C++ for Robotics]] — ownership/RAII, callback lifetimes, bounded queues, allocation in control loops, thread safety, and a sensor-to-controller pipeline with a latency budget.

### Next: expand existing notes with worked examples

- Expand [[Inverse Kinematics]] — damped least squares, pose-error conventions, joint limits, null-space motion, convergence, and failure near singularities.
- Expand [[Camera Calibration]] — replace the link-only stub with intrinsics/distortion, capture procedure, reprojection diagnostics, held-out validation, stereo depth uncertainty, and hand-eye/temporal calibration connections.
- Expand [[Extended Kalman Filter]] — encoder-plus-IMU state and measurement models, Jacobians, bias/observability, Q/R tuning, innovation gating, consistency checks, and delayed measurements.
- Expand [[PID Controller]] and [[Trajectory Optimization and MPC]] — discrete PID with anti-windup/filtering; a concrete constrained MPC objective, warm start, deadline miss, and fallback.
- Expand [[Robot Dynamics and Spatial Algebra]] and [[Operational Space Control]] — manipulator equation, gravity compensation, inverse dynamics, Jacobian-transpose wrench mapping, and impedance versus admittance.
- Expand [[Factor Graph]], [[Bundle Adjustment]], and [[Pose Graph Optimization]] — weighted residuals, gauge freedom, robust losses, sparsity, marginalization, and trajectory evaluation; connect to a future [[IMU Preintegration]] note.
- Expand [[ros2_control]] and [[MoveIt]] — read/update/write, resource claiming, lifecycle/controller switching, planning scene, attached objects, time parameterization, and execution failures.
- Expand [[Drive-Train Modeling for Balancing Robots]], [[Encoder]], and [[IMU]] — count-to-velocity conversion, wrap/quantization, odometry integration, slip, specific force, bias, and calibration.
- Expand [[Robot Policy Deployment]] and [[Safe RL for Robotics]] — one complete robot case study with measured latency, acceptance metrics, held-out conditions, fault injection, and recovery behavior.

### Corrections and refreshes to queue before relying on the notes

- Correct [[Lie Group]] / [[Lie Algebra]] definitions, KDL classification in [[Inverse Kinematics]], and motor/yaw equations in [[Drive-Train Modeling for Balancing Robots]] / [[Two-Wheeled Inverted-Pendulum Dynamics]].
- Correct [[ROS2 Node]], [[ROS2 Services]], and [[QoS]] — node versus process, async service semantics, and version-specific profile defaults; revise the admissibility/closed-set assumption in [[A Star Algorithm]].
- Review [[ORB-SLAM]], [[ICP]], [[GICP]], [[URDF]], and [[Atomic]] for the audit's misleading algorithm, API, and lock-free claims; distinguish integral augmentation from base LQR/MPC behavior in [[PID Controller]].
- Refresh [[ROS2]], [[Gazebo]], [[Ignition]], [[Gazebo Garden]], [[MJX]], [[MuJoCo]], [[Isaac Lab]], and [[Robot Policy Deployment]] against official versioned sources; reconcile older summaries with [[MuJoCo Warp]] and distinguish stable releases from previews.
- Fix vision links that resolve `FAST` to the action-tokenizer note; review duplicate A* / ICP / Open X-Embodiment entry points and the `ph2sys` typo in [[PTP]].

### Later: role-specific depth

- Robot learning: expand [[ACT Action Chunking Transformer]] with temporal ensembling; correct [[OpenVLA]] fine-tuning advice; refresh [[LeRobot]] and [[Robot Dataset Formats]]; add [[DAgger]] and compare flow-matching/action-chunk inference approaches using closed-loop metrics.
- Embedded and aerial: expand [[EtherCAT]], [[CAN]], [[MAVLink]], [[PX4]], and [[ArduPilot]] with drive/bus state machines, command acknowledgments, offboard lifecycle, watchdogs, and tested link-loss recovery.
- Manipulation and locomotion: expand [[Whole-Body Control]] with contact constraints, friction cones, task hierarchy, and a small constrained-control example.

## Deep Dives

Planned walkthroughs that follow one concrete operation across the existing topic notes. Start with the six priorities below, then choose follow-ups by interest. Reuse and expand existing explanations where they already cover a stage; choose folder locations before creating new files.

### First priorities

- [[How an LLM Generates One Token]] — trace a prompt through chat formatting, token IDs, embedding lookup, positional information, Q/K/V attention, residuals, normalization, MLPs, vocabulary logits, sampling, and KV-cache updates. Use a small decoder-only model with explicit tensor shapes and a numerical attention example; finish with detokenization and streamed output.
- [[How an LLM Learns]] — follow one batch through shifted labels, causal and loss masks, forward pass, cross-entropy, backpropagation, gradient accumulation, and an optimizer update. Distinguish parameters, activations, gradients, and optimizer state; connect pretraining, supervised fine-tuning, preference optimization, and held-out evaluation.
- [[From URL to Pixels]] — follow entering `google.com` through browser input/navigation, caches, DNS, route lookup, ARP or IPv6 neighbor discovery, packet forwarding, TCP/TLS or QUIC, HTTP, server response, HTML/CSS/JavaScript, layout, paint, rasterization, and compositing. Compare cold and warm visits using a browser trace; distinguish opening the homepage from submitting a search and mark Google's private internals as unknown.
- [[From SQL Query to Durable Commit]] — follow a PostgreSQL request through parsing, planning, index/table access, buffer cache, execution, MVCC snapshots, locks, WAL, commit, and crash recovery. Use `EXPLAIN` and a two-session transaction example to connect query speed, isolation, and durability.
- [[From Shell Command to Running Process]] — trace shell parsing, quoting, expansion, PATH lookup, redirection/pipes, process creation, `exec`, ELF loading, dynamic linking, runtime initialization, and `main()`. Show builtins versus external programs and inspect a tiny example with process maps and syscall tracing.
- [[From Memory Access to Physical RAM]] — follow allocation and first access through allocator bookkeeping, virtual mappings, demand paging, page faults, page tables, TLB, cache lines, and RAM. Explain copy-on-write, NUMA, false sharing, and why allocated address space is not the same as resident memory.

### AI and model execution

- [[How Image Generation Turns a Prompt into Pixels]] — trace one prompt through a named open model with inspectable weights/code, from text conditioning and initial noise to a saved image. Include actual tensor shapes, intermediate visualizations, and a small reproducible inference loop; separate architecture-specific mechanisms from the general workflow.
  - **How the model learns:** follow training images/captions, image encoding, sampled noise/timesteps, prediction targets, loss, and weight updates. Compare diffusion noise prediction with flow-matching velocity prediction; distinguish training from the iterative process used to generate a new image.
  - **How words influence the image:** trace tokenization, text embeddings, pooled conditioning, and cross-attention or joint attention in the chosen architecture. Explain spatial relationships, prompt adherence, negative prompts where supported, and why writing readable text inside an image is a separate challenge from understanding the prompt.
  - **What happens at each generation step:** inspect random initialization, latent dimensions, timestep conditioning, U-Net or transformer predictions, guidance, and sampler/scheduler updates. Visualize selected intermediate states; distinguish a model prediction from the numerical update, and compare step count, guidance strength, seed, and resolution while holding other settings fixed.
  - **How latent values become visible pixels:** follow the image decoder, scaling, color channels, output range, and image encoding. Distinguish latent-space and pixel-space generation, continuous patch representations and discrete image tokens, and iterative diffusion/flow generation versus autoregressive image-token generation; avoid assuming every generator follows the same pipeline.
  - **How editing and reference images work:** trace image encoding, noise strength or the model's editing-specific conditioning, masks, inpainting/outpainting, reference-image conditioning, and structural controls. Explain why preserving an identity, changing one object, and keeping every unmasked pixel identical are different requirements; inspect where compositing is used.
  - **Where time, memory, and failure modes come from:** profile text encoding, repeated model evaluations, attention, and image decoding. Compare precision, batching, offload, resolution, distilled models, and upscaling; investigate repeated objects, incorrect counts, anatomy, text artifacts, and seed reproducibility across software/hardware changes. Record the exact model revision, settings, timings, VRAM use, and observed outputs.
- [[LLM Inference Memory and Performance]] — account for weights, KV cache, activations, workspace, and runtime overhead; contrast prefill with decode, first-token latency with token throughput, and single requests with batching. Connect GQA/MQA, prefix reuse, eviction, and CPU offload to a reproducible profile.
- [[How Quantization Changes a Model]] — work through floating-point values becoming integer codes, scales, zero points, grouping, and reconstructed values; compare weight-only and activation quantization, calibration, outliers, accuracy loss, and the kernels that actually execute the model. Connect to the existing GGUF and KV-cache quantization backlog.
- [[How LoRA Changes a Model]] — derive the low-rank weight update, identify frozen/trainable tensors, follow gradients through adapters, and explain rank/scaling, target modules, checkpointing, merging, and QLoRA. Keep model adaptation distinct from the LoRa radio protocol.
- [[From Documents to a Grounded RAG Answer]] — trace ingestion, parsing, chunking, metadata, embeddings, indexing, query retrieval, filtering, reranking, prompt assembly, generation, and citation checks. Include stale documents, missed evidence, conflicting sources, and retrieval-versus-answer evaluation; build on [[RAG]] rather than repeat its catalog.
- [[What Happens During an Agent Tool Call]] — trace model output through structured arguments, validation, permission boundaries, actual tool execution, returned observations, context updates, and another model step. Cover malformed calls, timeouts, cancellation, retries, idempotency, and why generating a command is different from executing it.
- [[How Coding Agent Harnesses Work]] — follow one bug-fix request through the software surrounding the model: session creation, repository discovery, instruction loading, context assembly, model requests, streamed responses, tool dispatch, file edits, test feedback, and completion. Build on [[Agentic Coding Loops]] and the tool-call deep dive, with these concrete stages:
  - **What goes into a model call:** system/developer/user messages, repository instructions, tool schemas, selected files, search results, conversation history, token budgets, and provider-specific serialization. Show what the model can see versus what remains on disk or in harness state.
  - **Who controls the loop:** distinguish model-proposed actions from harness-enforced transitions; trace tool-call IDs, argument validation, dispatch, returned observations, and the next inference request. Include a minimal event-loop implementation and an annotated transcript without assuming access to private model reasoning.
  - **How actions become real changes:** trace shell process creation, working directory/environment, stdout/stderr, exit codes, persistent process sessions, patch application, and diff inspection. Explain tool adapters, MCP integration, approval policy, sandbox enforcement, and the trust boundary between repository/tool content and instructions.
  - **How long tasks retain context:** compare full transcripts, output truncation, retrieval, summaries/compaction, persistent notes, and prompt caching. Inspect what survives a restart, what must be re-read, and how stale context or a summary can lose an important constraint.
  - **How execution stays coordinated:** cover asynchronous tools, cancellation, timeouts, retries after partial side effects, user messages arriving mid-task, and optional subagents/worktrees. Track ownership of files and running processes, shared versus isolated state, and recovery after a crash or interrupted edit.
  - **How the harness decides to continue or stop:** separate the model's completion claim from test evidence, policy gates, iteration/cost limits, and human review. Reproduce a failing test, a successful fix, a misleading passing check, and a stalled loop; explain why the same model can perform differently with different tools and context policies.
  - **Worked implementation:** use a tiny disposable repository and a pinned open-source harness revision to trace prompt → search → read → patch → failing test → revised patch → passing test → final diff. Record messages, tool events, latency, token usage, and persisted state; connect implementation findings to [[oh-my-pi]] and the existing local coding-agent backlog.

### Operating systems and developer tools

- [[What Happens When You Save a File]] — trace application buffers, write calls, page cache, filesystem metadata, journaling, block I/O, device caches, and persistence. Compare overwrite with temporary-file-plus-rename patterns; show what file and directory synchronization protect against during a crash.
- [[From Power Button to Linux Login]] — follow firmware initialization, boot selection, bootloader, kernel decompression/startup, initramfs, root filesystem mounting, init/service startup, device discovery, and login. Contrast a PC UEFI boot with an embedded boot chain and locate useful logs at each stage.
- [[From Source Code to CPU Instructions]] — follow preprocessing, parsing, type checking, intermediate representation, optimization, assembly, object files, relocations, linking, and execution. Compile one tiny function at different optimization levels and inspect symbols, disassembly, and debug information.
- [[What Happens When You Start a Container]] — trace image resolution/pull, layers, writable filesystem, runtime setup, namespaces, cgroups, mounts, networking, and process startup. Explain shared-kernel isolation, PID 1/signals, resource limits, volumes, shutdown, and how this differs from a virtual machine.
- [[What Happens During Git Commit and Push]] — trace working tree, index, blobs, trees, commits, refs, object transfer, and remote ref updates. Explain what a commit records, how branches differ from folders, what rebasing changes, and why local success does not imply a successful push.
- [[How a Concurrent Request Is Scheduled]] — follow one request across threads, event loops, readiness notifications, queues, and workers. Contrast blocking with async execution and parallelism; demonstrate cancellation, backpressure, races, deadlocks, and queueing latency using a small service.
- [[From PyTorch Matmul to a GPU Result]] — trace tensor layout/dtype, framework dispatch, device memory, transfers, library/kernel selection, stream submission, GPU execution, synchronization, and result access. Use a profiler to separate host launch time, device work, and copies; connect to [[CUDA Streams]], [[cuBLAS]], and [[PTX]].

### Networks, search, and distributed services

- [[How Email Travels through SMTP IMAP and POP3]] — follow a message with an attachment from clicking Send to delivery, reading it on two devices, replying, and deleting it. Separate message submission/relay from mailbox access; compare SMTP, IMAP, and POP3 using actual protocol transcripts and a diagram of clients, submission servers, relays, delivery agents, and message stores.
  - **Sending and routing with SMTP:** trace authenticated submission, DNS MX lookup, server-to-server delivery, `EHLO`, `MAIL FROM`, `RCPT TO`, `DATA`, and reply codes. Distinguish envelope addresses from visible headers, explain Bcc handling, queues, temporary/permanent failures, retries, and bounces, and show why acceptance by one server does not prove inbox delivery or that a person read the message. [SMTP specification](https://www.rfc-editor.org/rfc/rfc5321.html).
  - **What the message bytes contain:** annotate headers, the blank-line separator, body, MIME multipart boundaries, text/HTML alternatives, attachment encoding, character sets, Message-ID, threading headers, and Received hops. Separate transport framing from message format, and explain why attachments grow during encoding and why an email address's display name is not proof of identity.
  - **Keeping devices synchronized with IMAP:** trace capability negotiation, authentication, mailbox selection, message fetching/searching, flags, and change notifications such as IDLE. Explain sequence numbers versus UIDs/UIDVALIDITY, partial body downloads, local caches, concurrent clients, and the difference between marking a message deleted and expunging it. Identify the protocol revision/extensions used. [IMAP4rev2 specification](https://www.rfc-editor.org/rfc/rfc9051.html).
  - **Downloading mail with POP3:** walk through authorization, transaction, and update states using `STAT`, `LIST`, `UIDL`, `RETR`, `DELE`, `RSET`, and `QUIT` as supported. Show when deletions become effective, what happens on an interrupted session, how clients avoid repeat downloads, and why leaving copies on the server does not provide IMAP-style folder/read-state synchronization. [POP3 specification](https://www.rfc-editor.org/rfc/rfc1939.html).
  - **Ports, encryption, and login:** map SMTP relay/submission and IMAP/POP3 to their conventional ports; compare implicit TLS with STARTTLS and distinguish authentication from encryption. Trace certificate checks and a named SASL/OAuth mechanism; separate client-to-server security, relay-to-relay transport security, and end-to-end message encryption. [Email submission and access TLS guidance](https://www.rfc-editor.org/rfc/rfc8314.html).
  - **Why mail gets rejected or lands in spam:** scope SPF, DKIM, and DMARC checks and alignment, forwarding/mailing-list complications, spam filtering, and reputation. Read delivery logs, authentication results, and bounce status codes; distinguish domain authentication from content encryption, mailbox login, and a guarantee that a message is trustworthy.
  - **Worked lab and comparison:** use disposable local accounts and a test mail server to follow one message through submission, storage, IMAP synchronization, and POP3 retrieval. Capture server logs and sanitized application-level transcripts, reproduce a rejected recipient and a disconnect during retrieval, and compare offline behavior, folders, deletion, multi-device use, and server/client storage. Explain where webmail's browser HTTP connection sits in this architecture without assuming every provider uses IMAP internally.
- [[How TLS Establishes Trust]] — inspect one TLS 1.3 handshake: negotiation, ephemeral key agreement, certificate chain and hostname checks, transcript authentication, derived traffic keys, and encrypted records. Compare a fresh connection with resumption; connect to HTTPS and QUIC while keeping their transport details distinct.
- [[How a Search Query Becomes Ranked Results]] — separate prior crawling/indexing from query-time parsing, candidate retrieval, ranking/reranking, snippets, and result serving. Build a tiny inverted-index example and compare lexical with vector retrieval; label public Google documentation separately from illustrative backend design.
- [[From Background Job to Reliable Completion]] — follow enqueue, transaction commit, worker claim, execution, acknowledgment, retry, timeout, and recovery. Use [[Oban]] as a concrete example; show duplicate delivery, idempotent side effects, backoff, and what happens if a worker crashes after performing an action.
- [[What Happens When a Distributed Write Fails Halfway]] — trace a client write across replicas and acknowledgments, then inject a lost response, leader failure, or network partition. Compare committed, visible, durable, and acknowledged states; explain retries, deduplication, quorum assumptions, and the limits of exactly-once claims.

### Cameras, devices, and physical systems

- [[How Global Shutter Pixels Store an Exposure]] — deep dive into electronic CMOS global shutter, building on [[Global Shutter]] and comparing it with [[Rolling Shutter]]. Follow photons through charge accumulation, simultaneous exposure capture, temporary pixel storage, sequential readout, digitization, and delivery of a frame; explain exactly what is stored, where, and for how long.
  - **Inside one pixel:** draw a representative photodiode, transfer gates, shielded storage node, floating diffusion, reset transistor, and readout circuitry. Follow electrons during reset, integration, global transfer, storage, and readout; distinguish the storage node from the sense node and identify which details are specific to the chosen sensor architecture. [Sony Pregius pixel technology](https://www.sony-semicon.com/en/technology/industry/pregius.html).
  - **What sensor memory actually means:** compare charge-domain storage with voltage-domain sample-and-hold and digital storage after conversion. Explain where capacitors/charge wells retain an analog signal, when ADC conversion happens, and how this differs from SRAM/DRAM frame buffers in a sensor stack, camera, or host. Avoid treating every global-shutter design as the same circuit or assuming it needs a conventional digital memory cell at every pixel.
  - **Global exposure versus sequential readout:** draw row-by-row timing diagrams for rolling shutter, global shutter, and global reset release. Track exposure start/end, global transfer, stored-signal waiting time, and row readout; show why reading a global-shutter frame sequentially does not recreate rolling-shutter geometry. Distinguish exposure duration, row interval, full-frame readout time, and frame period. [Basler electronic shutter types](https://docs.baslerweb.com/electronic-shutter-types).
  - **What happens while the stored frame waits:** investigate light shielding, parasitic light sensitivity, leakage/dark signal, storage capacity, transfer efficiency, reset/read noise, and correlated double sampling where supported. Explain why a storage node can change pixel area, full-well capacity, dynamic range, and sensitivity; compare front-side, back-side, and stacked implementations without assuming universal performance penalties.
  - **Whether the next exposure can overlap readout:** trace two consecutive frames and ownership of the photodiode/storage node at each stage. Use a named sensor's timing limits to explain overlapping acquisition, minimum frame periods, dead time, trigger acceptance, and overrun; account separately for sensor readout and downstream link bandwidth.
  - **What global shutter fixes and what it does not:** compare a moving edge and rotating object under matched exposure times; calculate rolling-shutter row-time skew separately from motion blur during integration. Cover pulsed lighting, global-reset flash windows, trigger latency/jitter, and multi-camera synchronization; explicitly correct the existing overview's implications that global shutter eliminates all motion distortion or automatically synchronizes separate cameras.
  - **Worked trace:** use a small pixel-array schematic, a charge/voltage timeline, and a moving-target simulation, then map the model to a documented real sensor. Annotate reset → exposure → transfer → storage → ADC → output; calculate frame payload and required output bandwidth, label conceptual versus measured timings, and show which internal signals cannot be observed from camera timestamps alone.
- [[From Photons to a Camera Frame]] — trace exposure, sensor readout, analog-to-digital conversion, raw pixel format, optional ISP processing, transport, driver buffers, timestamps, and application access. Compare raw monochrome/Bayer paths with processed images; connect rolling/global shutter, bandwidth, dropped frames, and calibration.
- [[From Camera Frame to Displayed Video]] — trace pixel formats, color conversion, codec prediction/transforms/quantization, packetization or container muxing, buffering, decoding, and display. Measure where latency accumulates and distinguish codec, container, and transport using [[ffmpeg]] examples.
- [[What Happens When You Plug In a USB Device]] — follow attachment detection, enumeration, descriptors, addressing/configuration, driver binding, endpoint transfers, DMA/completion handling, and device-node creation. Use kernel logs and udev inspection; include resets, disconnects, permissions, and why successful enumeration does not prove usable data transfer.

### Remote access, VPNs, and file transfer

- [[How Tailscale Connects Two Machines]] — follow login/device identity, key distribution, peer discovery, route installation, NAT traversal, WireGuard encryption, and direct versus DERP/peer-relay paths. Trace one packet with `tailscale ping`, `status`, and `netcheck`; build on [[Tailscale]] and reconcile its direct-only summary with relay behavior. [Connection types](https://tailscale.com/docs/reference/connection-types).
- [[How WireGuard Carries an IP Packet]] — trace interface routing, peer selection, allowed IPs, handshake/session keys, encrypted UDP encapsulation, replay protection, endpoint roaming, and decapsulation. Explain what WireGuard supplies versus the coordination added by Tailscale.
- [[How NAT Traversal and Hole Punching Work]] — follow two machines behind separate routers through endpoint discovery, mapping/filtering behavior, simultaneous probes, keepalives, and relay fallback. Draw the changing address/port tuples and explain why some networks prevent a direct path.
- [[How MagicDNS and Split DNS Resolve Names]] — trace a short tailnet name and a public domain through search suffixes, OS resolver selection, per-domain routing, caches, and upstream resolvers. Diagnose name resolution separately from reachability and authorization.
- [[How Exit Nodes and Subnet Routers Route Traffic]] — compare a private-subnet request with full-tunnel internet traffic; show route advertisements/approval, longest-prefix selection, forwarding, NAT, return paths, DNS, and overlapping subnets using a small routing diagram.
- [[How Tailscale SSH Differs from SSH over Tailscale]] — separate network reachability, tailnet identity/policy, SSH server implementation, user authorization, and session establishment. Compare ordinary OpenSSH on a tailnet address with Tailscale SSH, including policy changes and session failures.
- [[How SSH Establishes a Remote Session]] — trace TCP connection, version negotiation, key exchange, host-key verification, user authentication, encrypted channels, PTY allocation, remote shell, and exit status. Annotate `ssh -vvv`; distinguish host identity from user identity. [OpenSSH manual](https://man.openbsd.org/ssh).
- [[How SSH Keys and Agents Authenticate You]] — follow public-key enrollment, a signature request, private-key protection, agent sockets, key selection, and server verification. Compare passphrases, hardware-backed keys, certificates, agent forwarding, and why a forwarded agent is not a copied private key.
- [[How SSH Port Forwarding Works]] — trace `-L`, `-R`, and `-D` from listening socket through an SSH channel to the destination connection. Identify which host resolves/connects to each destination, bind addresses, access controls, tunnel lifetime, and failure symptoms.
- [[How SSH Jump Hosts and Connection Multiplexing Work]] — follow `ProxyJump` through a bastion and `ControlMaster` through a shared connection. Explain separate host verification, connection/channel reuse, control-socket permissions, stale sockets, and what survives a disconnect.
- [[How SCP Actually Copies a File]] — trace path parsing, SSH authentication, subsystem startup, reads/writes, metadata, progress, and exit status. Distinguish the `scp` command from legacy SCP: OpenSSH 9.0 onward defaults to SFTP; cover `-O`, quoting, wildcards, remote-to-remote copies, and partial transfers. [SCP manual](https://man.openbsd.org/scp).
- [[How Rsync Avoids Copying Unchanged Data]] — follow file-list construction, quick checks, rolling/strong checksums, delta reconstruction, temporary files, and final replacement. Contrast local copying, SSH transport, and daemon mode; explain resume behavior, metadata, trailing slashes, and deletion scope.

### Remote graphics and display systems

- [[How X11 Forwarding Works over SSH]] — follow a remote GUI application's X11 connection through `DISPLAY`, SSH's proxy display, `xauth` cookies, encrypted channels, and the local X server. Explain the client/server naming reversal, `-X` versus `-Y`, latency, and why forwarding is not ordinary screen-video streaming. [X11 forwarding reference](https://man.openbsd.org/ssh#X11_FORWARDING).
- [[How an X11 Application Draws a Window]] — trace display connection, resource IDs, window creation, drawing requests, input events, window-manager interaction, and compositing. Separate application, X server, window manager, and compositor; explain local Unix sockets versus network transport.
- [[How Wayland and Xwayland Display Applications]] — follow client buffer creation, compositor communication, input delivery, frame callbacks, and presentation. Show where Xwayland translates compatibility boundaries and why X11 remote-display assumptions do not transfer unchanged.
- [[How Xvfb Runs a GUI without a Monitor]] — build on [[Xvfb and Virtual X11 Displays]] by tracing server startup, authentication, display selection, framebuffer storage, rendering, screenshot capture, and teardown. Distinguish a virtual display from a remote desktop and from GPU acceleration.
- [[How VNC and RDP Deliver a Remote Desktop]] — compare framebuffer updates, drawing/video encoding, input return paths, session ownership, clipboard/device redirection, and reconnect behavior. Explain why bandwidth, latency, resolution, and host rendering affect the experience differently.
- [[How Headless GPU Rendering Works]] — trace application context creation, EGL/GLX or another chosen graphics API, device selection, render targets, synchronization, and readback. Diagnose missing display versus missing driver/context support; compare hardware rendering with software fallback.

### Packets, links, and network diagnosis

- [[How a DNS Query Is Resolved]] — follow a cache miss through stub and recursive resolvers, root/TLD/authoritative answers, aliases, TTLs, and negative caching. Compare UDP/TCP with encrypted DNS and use `dig` to separate resolver behavior from browser caching.
- [[How a Device Gets an IP Address]] — trace DHCPv4 discovery/lease/renewal and compare IPv6 router advertisements, SLAAC, and DHCPv6 roles. Explain gateway/DNS selection, duplicate-address checks, lease loss, and why link connectivity does not imply internet access.
- [[How ARP and IPv6 Neighbor Discovery Find the Next Hop]] — map a routed destination to a local-link neighbor, resolve its link-layer address, update caches, and transmit a frame. Include on-link versus gateway traffic, stale entries, and neighbor-unreachable diagnosis.
- [[How TCP Recovers from Loss and Controls Speed]] — follow sequence numbers, ACKs, receive windows, congestion windows, retransmissions, RTT estimates, and connection teardown. Use a packet trace to distinguish application slowness, receiver backpressure, packet loss, and limited bandwidth.
- [[How QUIC Multiplexes Streams]] — trace connection establishment, packet numbers, independent streams, loss recovery, flow control, and connection migration. Compare transport head-of-line behavior with HTTP/2 over TCP and explain what HTTP/3 adds above QUIC.
- [[How a Packet Crosses the Linux Network Stack]] — follow a socket send through routing, firewall/connection tracking, queues, driver, and NIC, then trace receive processing back to a socket. Mark network namespaces and capture points to explain apparently contradictory packet traces.
- [[Why Small Network Requests Work but Large Ones Hang]] — investigate MTU, encapsulation overhead, fragmentation rules, path-MTU discovery, filtered control messages, and TCP MSS. Build a controlled size-sweep experiment across a VPN instead of diagnosing every stall as packet loss.
- [[How Wi-Fi Connects and Roams]] — trace scanning, association, authentication/key establishment, contention, retransmissions, power saving, and roaming between access points. Connect signal quality and airtime to latency; separate roaming from obtaining a new IP address.
- [[How WebRTC Establishes a Peer Connection]] — trace application signaling, SDP, ICE candidate gathering/checks, STUN/TURN, transport security, and media/data-channel setup. Explain direct versus relayed paths, renegotiation, bandwidth adaptation, and useful connection statistics.
- [[How Local Service Discovery Works]] — follow mDNS/DNS-SD announcements, multicast queries, service records, address resolution, and cache expiry. Explain `.local`, interface selection, discovery across VLAN/VPN boundaries, and why seeing a service does not prove it is reachable.

### Filesystems, storage, and synchronization

- [[How SFTP Operates over SSH]] — follow subsystem negotiation, request IDs, file handles, pipelined reads/writes, attributes, errors, and close operations. Distinguish protocol capabilities from client features such as resume; compare subsystem restrictions with shell access.
- [[How NFS and SMB Turn Remote Files into Local Paths]] — trace mount/session setup, pathname lookup, permissions, reads/writes, caching, locking, and reconnects. Compare local filesystem expectations with remote consistency, identity mapping, and outage behavior.
- [[How FUSE and SSHFS Implement a Filesystem]] — trace a file operation from the kernel VFS to a user-space handler and onward over SSH. Explain caching, permissions, blocking, connection loss, and why a mounted path can hang applications.
- [[How an NVMe SSD Services a Read and Write]] — trace filesystem/block requests, submission/completion queues, DMA, controller mapping, flash pages, garbage collection, and wear leveling. Explain queue depth, TRIM, sustained versus burst performance, and power-loss assumptions.
- [[How Snapshots and Backups Preserve Data]] — follow copy-on-write snapshots, incremental change tracking, retention, verification, and restore. Distinguish a snapshot on the same device from an independent backup; include database consistency and a demonstrated restore.
- [[How Compression and Archives Store Files]] — separate archive metadata from compression streams; work through dictionary/entropy coding, checksums, seekability, and streaming. Compare tar plus gzip/zstd with ZIP using concrete files and corruption/recovery behavior.

### Linux sessions, services, and resource management

- [[How Terminals and PTYs Carry Input and Output]] — trace keystrokes through terminal emulation, PTY master/slave, line discipline, shell, escape sequences, and screen updates. Connect SSH terminal allocation and tmux persistence; explain raw/canonical mode, echo, and terminal size.
- [[How Unix Signals and Job Control Work]] — follow process groups, controlling terminals, foreground/background jobs, signal delivery, handlers, and child reaping. Explain Ctrl-C/Ctrl-Z, SIGHUP, graceful shutdown, zombies, and the distinction between stopping and terminating a process.
- [[How systemd Starts and Supervises a Service]] — trace unit dependencies/order, environment, credentials, execution, readiness, restart policies, logging, and shutdown. Compare process existence with service readiness and diagnose startup races using a small service.
- [[How File Descriptors and epoll Connect Programs]] — follow `open`, `dup`, `fork`, `exec`, pipes, sockets, readiness notifications, and close/reference lifetimes. Show descriptor leaks, pipe EOF, nonblocking reads, and level-triggered versus edge-triggered event handling.
- [[How Shared Memory and mmap Move Data between Processes]] — trace mappings, shared backing storage, page faults, visibility, synchronization, and cleanup. Compare shared memory with pipes/sockets; explain why sharing bytes does not supply a message protocol or race protection.
- [[How Linux Decides Whether Access Is Allowed]] — follow user/group credentials, ownership/mode bits, ACLs, capabilities, namespaces, and security-module policy through one file or device access. Explain permission-denied diagnostics and privilege changes without treating root as a universal bypass.
- [[How the Linux Scheduler Allocates CPU Time]] — trace runnable tasks, wakeups, scheduling classes, priorities, affinity, context switches, and cgroup quotas. Measure latency under load and connect priority inversion, throttling, and real-time policy assumptions.

### Containers, deployment, and build infrastructure

- [[How Docker Networking Connects a Container]] — follow network namespaces, virtual Ethernet pairs, bridges, address assignment, DNS, published ports, NAT, and return traffic. Compare bridge and host networking and trace a request from host to container to internet.
- [[How Kubernetes Turns a Manifest into a Running Pod]] — follow API validation/admission, stored desired state, reconciliation, scheduling, kubelet, runtime, network/storage setup, and readiness. Explain why accepted configuration, scheduled workload, and serving application are distinct states.
- [[How Kubernetes Routes Traffic to a Service]] — trace DNS, Service addresses, endpoint selection, the installed data plane, ingress/load balancing, and pod readiness changes. Cover external versus in-cluster paths without assuming every cluster uses the same proxy implementation.
- [[How Container Image Builds and Layer Caches Work]] — follow build context, instructions, dependency layers, content addressing, cache invalidation, registry storage, and image pulling. Explain multi-stage builds, reproducibility, architecture selection, and why deleting a file in a later layer does not erase earlier content.
- [[What Happens When a Package Manager Installs Software]] — follow metadata resolution, version constraints, downloads, integrity verification, installation/build steps, environment paths, and lockfiles. Use a Python wheel/source package as the worked example and contrast it with a system package.
- [[How a CI-CD Pipeline Produces and Deploys an Artifact]] — trace a source event through runner provisioning, dependency caches, build/test stages, artifact identity, credentials, deployment, health checks, and rollback. Explain reproducibility, promotion of the same artifact, and failure between stages.

### Web services, authentication, and caches

- [[How a Reverse Proxy Handles a Request]] — follow listener acceptance, optional TLS termination, host/path routing, upstream selection, connection reuse, buffering, timeouts, and response streaming. Diagnose 502/504 errors and distinguish proxy latency from application latency.
- [[How ACME Issues and Renews a Certificate]] — trace account registration, domain-control challenges, certificate request, issuance, installation, and renewal. Compare HTTP and DNS challenges and show why successful issuance does not prove the server is presenting the new certificate.
- [[How Web Login Creates a Session]] — trace credential verification, password hashing, cookies, session storage, expiration, logout, and request authentication. Compare a local login with an OAuth/OIDC authorization-code flow, explicitly separating authorization from identity assertions.
- [[How Browsers Enforce Origins and Cross-Site Boundaries]] — trace origin checks, CORS preflights, credential handling, cookie SameSite rules, and CSRF defenses. Use two local origins to show why CORS controls response access rather than authenticating callers or universally preventing requests.
- [[How WebSockets and Server-Sent Events Keep Updates Flowing]] — follow setup, framing/event parsing, buffering, heartbeats, reconnects, ordering, and application backpressure. Compare bidirectional sockets with server-to-client streams through a reverse proxy and a dropped connection.
- [[How CDN and HTTP Caches Decide What to Serve]] — trace cache keys, freshness, validators, revalidation, variants, stale responses, purge/invalidation, and origin fetches. Explain private versus shared caches and reproduce a stale-content or incorrect-cache-key bug.

### Language runtimes and execution boundaries

- [[What Happens When Python Imports a Module]] — trace search paths, finders/loaders, package initialization, bytecode caches, module caching, extension loading, and circular imports. Explain why changing the working directory or environment can load a different module.
- [[How Garbage Collectors Find and Reclaim Objects]] — follow roots, reachability, reference counting or tracing, generations, cycles, pauses, and finalization in a chosen runtime. Distinguish live objects, reusable heap space, and memory returned to the OS.
- [[How BEAM Processes Exchange Messages and Recover]] — trace lightweight process creation, scheduler work, mailboxes, selective receive, links/monitors, exits, and supervisor restart decisions. Use [[BEAM]] and [[Oban]] connections while separating process recovery from restoring application state.
- [[What Happens Across an FFI Boundary]] — trace calling convention, argument layout, ownership, lifetime, error propagation, callbacks, and thread/runtime constraints. Compare a Python native extension with an Elixir NIF and show how a small ABI mismatch causes corruption.
- [[How a JIT Turns Runtime Code into Machine Code]] — follow bytecode/IR execution, profiling, compilation, specialization, guards, deoptimization, and code caching. Compare one JavaScript or WebAssembly runtime with ahead-of-time compilation using a measurable warmup example.

### Embedded communication, timing, and control

- [[How Interrupts and DMA Deliver Sensor Data]] — trace peripheral events, DMA descriptors/buffers, completion interrupts, deferred work, cache visibility, and application wakeups. Compare polling and interrupts; measure latency, overruns, and ownership races.
- [[How UART Turns Bytes into a Serial Stream]] — follow framing bits, baud clocks, sampling, FIFOs, interrupts/DMA, and receive buffers. Distinguish logic-level UART from USB serial and RS-232/RS-485 electrical layers; diagnose garbled data and missing message boundaries.
- [[How I2C and SPI Complete a Sensor Read]] — trace bus selection/addressing, clock/data edges, register commands, repeated starts or chip-select boundaries, and returned bytes. Use logic-analyzer captures to explain pull-ups, mode mismatch, clock stretching, and bus contention.
- [[How a Debug Probe Flashes and Stops a Microcontroller]] — follow host tools through SWD/JTAG, target discovery, reset/halt, flash algorithms, breakpoints, register/memory access, and resume. Explain bootloader flashing versus debug access and what changes when the target resets.
- [[How a CAN Frame Wins Arbitration and Reaches an ECU]] — trace bit timing, dominant/recessive arbitration, identifier filtering, acknowledgment, errors, retries, and bus-off. Separate CAN framing from higher-level message meaning and compare classic CAN with CAN FD where relevant.
- [[How EtherCAT Drives a Synchronized Motor Cycle]] — trace the master's cycle, frame processing through devices, process-data mapping, distributed clocks, controller/drive state, and feedback. Diagnose working-counter mismatches, missed cycles, and drive watchdogs using a timing diagram.
- [[How NTP and PTP Synchronize Clocks]] — follow timestamps, offset/delay estimates, clock adjustment, hardware timestamping, and holdover. Explain asymmetry, drift, wall versus monotonic clocks, and why timestamp precision does not guarantee synchronization accuracy.

### AI serving, indexing, and training infrastructure

- [[How an LLM Server Schedules Many Requests]] — follow admission, tokenization, prefill/decode scheduling, continuous batching, KV allocation, prefix reuse, cancellation, and eviction. Build on the inference-performance topic with a multi-client trace and explicit latency/fairness tradeoffs.
- [[How HNSW Finds Similar Vectors]] — trace graph construction, levels, neighbor selection, insertion, approximate search, and candidate queues. Measure recall/latency/memory and explain how filtering, deleted entries, and embedding-model changes affect results.
- [[How Multi-GPU Training Synchronizes Gradients]] — follow batch partitioning, forward/backward work, gradient buckets, collectives, optimizer steps, and synchronization. Compare data/tensor/pipeline parallelism and expose communication costs with a small distributed trace.
- [[How a Training Checkpoint Resumes an Experiment]] — follow saving/loading of weights, optimizer/scheduler state, random generators, data position, and precision/distributed state. Distinguish inference weights from resumable training state and test recovery after an interrupted write or changed world size.
- [[How a Vision-Language Model Turns an Image into Tokens]] — follow decoding, resizing/cropping, patch or feature extraction, visual encoding, projection/fusion, positional information, and language generation. Compare image and video inputs, token budgets, spatial detail loss, and model-specific preprocessing.

### SSH key generation and authentication troubleshooting

- [[What ssh-keygen Actually Generates]] — trace OS randomness, algorithm-specific key-pair generation, public-key derivation, serialization, private-key passphrase protection, and fingerprints. Inspect a disposable Ed25519 example and explain `-t`, `-f`, `-C`, `-a`, `-l`, `-y`, and `-p`; distinguish comments, fingerprints, user keys, host keys, `authorized_keys`, and `known_hosts`. Connect generation to enrollment, rotation, and removal without overwriting an existing key. [ssh-keygen manual](https://man.openbsd.org/ssh-keygen).
- [[Why an SSH Key Is Rejected]] — trace configuration precedence, identity selection, agent offers, server authorization, file ownership/modes, algorithm compatibility, and account restrictions. Correlate client verbose output with server logs; distinguish a rejected user key from a changed host key, wrong username, inaccessible agent, or authentication-attempt limit.

### HTTP requests from simple examples to complete workflows

- [[An HTTP GET Request Byte by Byte]] — send a small request to a local test server and annotate method, target, headers, blank line, status, response headers, and body. Connect URL parsing to the bytes sent; compare `curl` with browser requests, redirects, compressed bodies, and connection reuse. Use HTTP/1.1 for the readable example and identify what changes with HTTP/2 or HTTP/3. [HTTP semantics](https://www.rfc-editor.org/rfc/rfc9110.html).
- [[How an HTTP JSON Request Is Built and Parsed]] — follow a structured object through JSON serialization, UTF-8 bytes, content type/length, server body reading, decoding, schema validation, application handling, and response parsing. Compare POST, PUT, and PATCH semantics; reproduce malformed JSON, wrong content type, empty bodies, and validation errors.
- [[How an HTTP File Upload Works]] — construct a multipart form with text and binary parts, inspect boundaries and metadata, and follow streaming ingestion, temporary storage, size limits, validation, and final persistence. Compare multipart with a raw binary body and a signed object-storage upload; show cancellation and partial-upload cleanup.
- [[How a Multi-Step Authenticated HTTP Workflow Works]] — trace login/token acquisition, cookies or authorization headers, resource creation, asynchronous job polling, and result download. Include redirects, pagination, token expiry, rate limits, timeouts, retries, and idempotency; contrast a browser's CORS/CSRF rules with a command-line client.
- [[How HTTP Range Requests Resume a Download]] — trace `Range`, `206`, `Content-Range`, validators, `If-Range`, and reassembly of partial content. Explain what happens when the resource changes, a server ignores ranges, or compression changes the representation; compare application byte ranges with transport packets.
- [[How HTTP Frames and Streams Delimit a Message]] — compare HTTP/1.1 content lengths/chunked transfer with HTTP/2 frames/streams and HTTP/3 over QUIC. Follow a large response through buffering, flow control, cancellation, and connection reuse; distinguish network reads, protocol frames, application chunks, and complete messages.

### M3U8 playlists and adaptive media playback

- [[How an M3U8 Playlist Becomes Playing Video]] — start with an actual UTF-8 playlist and follow a variant/master playlist to a media playlist, segment URLs, initialization data, downloads, demuxing, decoding, and presentation. Annotate core HLS tags, relative URLs, durations, sequence numbers, and end-of-stream; distinguish a playlist from media bytes and a codec. Connect [[M3U8]] and [[HLS]]. [HLS specification](https://www.rfc-editor.org/rfc/rfc8216.html).
- [[How Live HLS Chooses Quality and Stays Near the Live Edge]] — trace playlist reloads, sliding segment windows, bandwidth estimates, buffer occupancy, rendition switches, stalls, and recovery. Compare live with on-demand playback; investigate segment duration, keyframe alignment, CDN caching, and version-specific low-latency extensions using a request timeline.
- [[How Audio and Video Stay Synchronized]] — follow container timestamps, decode versus presentation order, audio/video clocks, buffering, clock drift, seeking, and discontinuities. Explain why frame rate, sample rate, and wall-clock time are different quantities; diagnose growing lip-sync error with a short media trace.

### CPU-GPU data movement and bandwidth

- [[How Data Moves from CPU RAM to GPU Memory and Back]] — follow allocation, CPU writes, staging/pinning, driver submission, DMA/copy engines, PCIe transfer, device storage, synchronization, and readback. Track ownership and when each side may use a buffer; compare a discrete GPU with shared-memory hardware and measure bytes transferred, elapsed time, and effective bandwidth. [CUDA host-device transfers](https://docs.nvidia.com/cuda/cuda-c-best-practices-guide/index.html#data-transfer-between-host-and-device).
- [[How Pinned Memory and Asynchronous GPU Copies Work]] — compare pageable and page-locked host buffers, staging costs, copy submission, streams/events, and buffer lifetime. Build a double-buffered transfer/compute pipeline and verify overlap in a profiler; explain why an asynchronous API name alone does not guarantee overlap or zero CPU overhead.
- [[How Unified Memory and Zero-Copy GPU Access Differ]] — distinguish unified addressing, managed-memory migration, mapped host access, and physically shared memory. Trace page faults, prefetching, residency, coherence/synchronization, and oversubscription; identify which mechanisms depend on the actual CPU/GPU platform.
- [[Why Memory Bandwidth Can Matter More than GPU FLOPS]] — count bytes and arithmetic operations for a vector operation and a matrix multiplication, estimate transfer/compute lower bounds, and plot arithmetic intensity against a roofline. Distinguish RAM, PCIe, and VRAM/HBM bandwidth, small-transfer latency, cache reuse, and batch-size effects; compare measured throughput with advertised peaks.
- [[How GPU Threads Access Memory Efficiently]] — trace warp memory requests through coalescing, cache lines, device memory, shared-memory tiles, bank conflicts, and register spills. Compare contiguous and strided access in one small kernel; connect layout choices to bytes moved, occupancy, and achieved bandwidth.
- [[How Multiple GPUs Exchange Data]] — trace a peer copy through the actual PCIe/NVLink topology, peer-access support, staging fallbacks, and synchronization. Compare point-to-point transfers with collective communication and explain NUMA placement, shared-link contention, and why adding GPUs need not increase throughput.

### Keyboards, mice, input events, and visible feedback

- [[From Keyboard Switch to Character on Screen]] — follow switch actuation, matrix scanning, debounce, rollover/ghosting, firmware mapping, HID reports, USB or wireless transport, kernel input events, layout/modifier processing, focused application handling, text insertion, and repaint. Trace press, hold, and release separately; distinguish physical keys, report usages, OS keycodes, and characters.
- [[From Mouse Movement to Cursor Movement]] — follow optical sensing and motion estimation, counts/deltas, button/wheel state, firmware reports, transport, OS input processing, acceleration/scaling, compositor cursor updates, and application hit testing. Explain CPI/DPI, polling, raw input, high-DPI displays, and why moving the cursor differs from clicking a widget.
- [[How HID Reports Describe Keyboards and Mice]] — decode a report descriptor and actual input/output/feature reports; follow usage pages, report IDs, bit fields, relative/absolute values, and driver event conversion. Explain host-scheduled USB interrupt transfers, transport differences, and output reports such as keyboard LEDs. [Linux HID introduction](https://docs.kernel.org/hid/hidintro.html).
- [[How Keyboard Layouts and Input Methods Produce Unicode]] — trace a hardware key through modifiers, layout maps, dead keys, compose sequences, IME preedit/commit, and application text events. Compare shortcuts with text input and explain why one keypress may produce no character, several characters, or different text across applications.
- [[Where Input-to-Photon Latency Comes From]] — budget sensing/scanning, debounce, report intervals, transport, OS queues, application/game sampling, simulation, rendering, presentation queues, scanout, and pixel response. Compare cursor and game-action paths; measure distributions and explain why device polling rate and display refresh rate do not determine total latency alone.
- [[How a Touchpad Turns Contacts into Gestures]] — trace capacitive sensing, contact tracking, palm rejection, device reports, gesture recognition, scrolling, acceleration, and application delivery. Separate hardware measurements from OS policy; explain multi-touch IDs, rejected contacts, and differences between pointer motion and gesture events.
- [[How Copy and Paste Moves Data between Applications]] — trace selection/clipboard ownership, advertised formats, on-demand conversion, data transfer, and insertion into another application. Compare plain text, rich text, images, large transfers, and remote-desktop clipboard forwarding; explain why clipboard contents may depend on the source application's lifetime.
- [[From Microphone Sound to Recorded Bytes and Back to Speakers]] — follow transduction, analog gain/filtering, ADC sampling, device transport, driver buffers, audio-server mixing/resampling, application processing, playback queues, DAC, and speaker output. Distinguish sample rate, bit depth, channels, buffer size, clipping, clock drift, and end-to-end audio latency.

### Drive discovery, mounting, and safe removal

- [[How an Operating System Discovers a New Drive]] — trace connection or boot-time detection through bus enumeration, controller/driver binding, device identification, capacity discovery, block-device creation, partition scanning, and filesystem probing. Compare Linux kernel events/udev with Windows Plug and Play; show why appearing in firmware, Device Manager, a disk utility, and a file browser are different milestones.
- [[What Linux mount Actually Does]] — follow a block device and filesystem driver into the VFS and a mountpoint in the directory tree. Explain superblocks, mount options, read-only mounts, bind mounts, mount namespaces, and why mounting over a populated directory hides its previous contents. Use `lsblk --fs`, `blkid`, `findmnt`, `mount`, and `/etc/fstab` to connect visible paths to underlying storage. [mount manual](https://man7.org/linux/man-pages/man8/mount.8.html).
- [[How Windows Gives a Volume a Drive Letter]] — trace disks, partitions, recognized filesystems, volume devices, the Mount Manager, volume GUID paths, drive letters, and mounted folders. Compare `C:\` with Linux's root tree; inspect Disk Management, `Get-Disk`, `Get-Partition`, `Get-Volume`, and `mountvol`, including a healthy volume with no drive letter. [Windows volume naming](https://learn.microsoft.com/en-us/windows/win32/fileio/naming-a-volume).
- [[How Automount Works on Linux and Windows]] — follow detection into policy: Linux udev events, UDisks, authorization, and desktop mounting versus `/etc/fstab` and systemd mount/automount units; compare Windows automatic volume mounting. Cover login versus boot, on-demand mounts, missing drives, permissions, and why discovering a disk does not automatically make its files available.
- [[What Safely Remove Hardware Actually Does]] — trace a Windows removal request through applications holding files open, outstanding I/O, cached writes, filesystem teardown, and device removal. Compare Quick removal and Better performance policies, OS versus device caches, refusal because a device is busy, and surprise unplugging during a write. Explain what the safe-to-remove notification confirms. [Windows removal policies](https://learn.microsoft.com/en-us/windows/client-management/client-tools/change-default-removal-policy-external-storage-media).
- [[How sync umount Eject and Drive Power-Off Differ]] — compare flushing writes, unmounting one filesystem, ejecting media, and powering off an entire external device. Trace a multi-partition USB drive; explain open files/current directories, `fuser` or `lsof` diagnosis, and `udisksctl` workflows. Cover why `sync` alone or lazy unmount does not establish that a drive is ready to unplug.
- [[Why a Drive Appears but Its Files Do Not]] — diagnose one layer at a time: bus/controller, block device, partition table, encryption, filesystem support/health, mount state, and access permissions. Compare Linux logs and block-device tools with Windows Device Manager and Disk Management; distinguish a missing drive letter, unsupported filesystem, locked volume, failing link, and an actually empty disk.
- [[How UUIDs Labels and Device Names Identify Storage]] — distinguish `/dev/sdX`, NVMe controller/namespace names, hardware serials, `/dev/disk/by-id`, filesystem UUIDs, partition UUIDs, labels, and Windows volume identifiers. Follow a reboot, port change, and cloned disk; explain stable mount configuration, duplicate identifiers, and why a name alone does not prove which physical drive is selected.

### Storage protocols and volume layers

- [[How SATA and NVMe Carry a Storage Request]] — trace the same read through a SATA device using ATA commands and an AHCI controller, then through an NVMe device over PCIe. Compare command queues, NCQ, submission/completion handling, DMA, interrupts, latency, parallelism, and bandwidth limits. Separate protocol, host-controller interface, and media type; connect to the existing NVMe read/write deep dive for flash-controller internals.
- [[Why M.2 Does Not Tell You a Drive's Protocol]] — separate connector/form factor, keying, SATA versus PCIe wiring, NVMe commands, lane count, and platform support. Compare a 2.5-inch SATA SSD, M.2 SATA SSD, M.2 NVMe SSD, PCIe adapter, and external enclosure; trace compatibility checks, shared motherboard lanes, negotiated link speed, and a drive that physically fits but is not detected.
- [[How a USB Enclosure Talks to a SATA or NVMe Drive]] — follow USB mass-storage requests through Bulk-Only Transport or UAS, SCSI commands, bridge translation, and the enclosed drive's native interface. Investigate queueing, flush/TRIM/health-information pass-through, firmware quirks, resets, power delivery, and the slowest link; distinguish a USB bridge from PCIe tunneling through a compatible external connection.
- [[How Partition Tables Divide a Drive into Volumes]] — decode MBR and GPT from a disposable disk image, following logical block addresses, partition boundaries/types, protective MBR, and GPT backup metadata. Separate physical disk, partition, logical volume, and filesystem; explain sector sizes, alignment, partition-table rereads, and why changing a partition boundary does not resize the filesystem inside it.
- [[What Formatting a Drive Actually Writes]] — follow filesystem creation through superblocks or boot sectors, allocation structures, root directories, and optional journals. Compare ext4, XFS, Btrfs, NTFS, and exFAT; separate partitioning, formatting, mounting, repair, and secure erasure. Use disposable images to compare quick/full format behavior on a named platform and inspect what metadata changes.
- [[How Disk Encryption Fits between a Drive and Its Files]] — trace Linux LUKS/dm-crypt and Windows BitLocker from locked storage through key acquisition, unlocked block access, filesystem mounting, and ordinary reads/writes. Distinguish unlocking from mounting, recovery keys from login passwords, and encryption boundaries; show which metadata remains visible and what happens when a device disappears while open.
- [[How LVM RAID and Storage Spaces Build Logical Disks]] — map filesystems to logical volumes, pools, extents, stripes, mirrors, and physical devices. Compare Linux device-mapper/LVM and software RAID with Windows Storage Spaces; trace one read, one write, a failed disk, and expansion. Explain redundancy, usable capacity, thin provisioning, rebuilds, and which layer must grow before more filesystem space appears.
- [[What Happens after You Delete a File from an SSD]] — trace unlinking and free-space accounting into discard/TRIM, controller mapping changes, garbage collection, and eventual flash erasure. Compare immediate discard, scheduled trimming, snapshots retaining blocks, and encrypted or USB-bridged paths; distinguish reclaimed filesystem space from physical erasure and connect to the existing NVMe controller deep dive.

### USB, USB-C, and the direction of signals

- [[How Two Wires Carry One Differential Signal]] — draw transmitter voltages, receiver thresholds, differential/common-mode voltage, termination, and return paths. Explain why the plus/minus wires of a pair are not TX/RX, why removing one conductor does not select a direction, and how noise, skew, impedance discontinuities, and cable length affect reception. Compare differential signaling with single-ended UART and VGA signals.
- [[How USB 2.0 Sends and Receives on the Same Pair]] — trace host tokens, device responses, bus turnaround, driver enable/high-impedance states, packet framing, checksums, and handshakes on D+/D−. Draw who drives the pair at each instant for an IN and an OUT transaction; explain host-relative direction, half-duplex operation, speed-dependent signaling, and why an interrupt endpoint still needs host scheduling.
- [[How USB 3.x Uses Separate Transmit and Receive Pairs]] — map SuperSpeed TX and RX differential pairs at both ends, then trace link startup, packet exchange, acknowledgments, flow control, and recovery. Compare full-duplex links with the separate USB 2.0 path in the same connector; show why cutting the return pair breaks protocol dependencies even when the application only sends data one way. [TI USB interface explanation](https://e2e.ti.com/support/interface-group/interface/f/interface-forum/1210274/usb-c-vs-usb3-0).
- [[How USB-C Works in Either Orientation]] — annotate receptacle versus cable wiring: VBUS, ground, D+/D−, high-speed pairs, CC1/CC2, and SBU. Trace attachment, orientation detection, mux routing, and the selected operating mode; distinguish the connector from USB speed, display support, and power capability. Follow a plug flip through an actual lane map. [USB-IF Type-C specification, revision 2.0](https://www.usb.org/sites/default/files/USB%20Type-C%20Spec%20R2.0%20-%20August%202019.pdf).
- [[How USB-C Negotiates Power and Data Roles]] — follow CC pull-ups/pull-downs, attachment, initial power availability, USB Power Delivery messages, cable identification where applicable, and contract changes. Separate source/sink power roles from host/device data roles; trace a laptop charging while controlling a peripheral, role swaps, disconnects, and why a charging connection can still exchange control messages.
- [[Why One-Way USB Takes More than Removing a Wire]] — distinguish a one-way application payload, a read-only device, a charging-only adapter, and a physically one-way information path. Map setup, requests, handshakes, and flow-control traffic needed for different transfer types, including isochronous exceptions; explain why D+/D− cannot be split by direction and why dedicated TX/RX pairs alone do not make a working data diode. Sketch where an active gateway must terminate protocols and define exactly which paths it permits.
- [[How USB Hubs Schedule Several Devices on One Port]] — follow a keyboard, storage device, and camera through hub topology, host-controller scheduling, endpoints, buffers, and upstream bandwidth. Compare USB 2.0 split transactions/transaction translators with SuperSpeed hub behavior; connect power budgeting, suspend/resume, resets, and a slow device or saturated camera to observable symptoms.
- [[How USB4 Carries USB Data and Display Traffic]] — trace router discovery, negotiated links, protocol adapters, tunnels, bandwidth allocation, and delivery at the far end. Compare USB3 and DisplayPort tunneling, platform-dependent PCIe support, and native DisplayPort Alt Mode; use a dock diagram to distinguish sharing a packet transport from assigning physical lanes to a different protocol.
- [[Why Identical USB-C Cables Behave Differently]] — inspect cable wiring, supported data modes, power ratings, electronic markers where required, passive versus active construction, and redriver/retimer roles. Trace examples that charge but lack video, fall back to USB 2.0, or fail at higher rates; separate a cable's advertised capability from the negotiated behavior of both ports and the entire path.

### VGA, HDMI, DisplayPort, and monitor feedback

- [[How VGA Turns Pixel Values into Analog Signals]] — follow framebuffer scanout, pixel clock, DAC output, analog RGB voltage, horizontal/vertical sync, blanking, and monitor sampling. Relate a waveform to positions in one scanline; compare CRT beam timing with LCD digitization, explain clock/phase adjustment and blurry text, and include the separate DDC path used to query monitor capabilities.
- [[How HDMI Sends Pixels Audio and Control]] — trace pixel/audio preparation, encoding, serialization, differential lanes, receiver recovery, and display processing. Compare a named TMDS mode with an FRL mode, including clocking and link setup; separately map DDC, hot-plug detect, CEC, and ARC/eARC so the direction of the main video stream is not confused with every signal in the cable. [HDMI specification overview](https://www.hdmi.org/spec/index).
- [[How DisplayPort Trains a Link and Sends a Frame]] — follow hot-plug detection, AUX capability reads, lane/rate selection, training, main-link transport, and sink reconstruction. Diagram the source-to-sink main link separately from bidirectional AUX and sink-to-source hot-plug signaling; compare legacy and UHBR encoding on named versions and diagnose training failure versus unsupported display timing.
- [[How a Monitor Tells the Computer What It Supports]] — read and decode EDID/DisplayID data, trace DDC over I2C for VGA/HDMI and I2C-over-AUX for DisplayPort, and show how the OS selects a mode. Separate advertised capabilities from physical link status; investigate hot-plug events, sleeping monitors, KVMs, stale capability data, and display arrangements changing after reconnects.
- [[How USB-C Carries DisplayPort and USB at Once]] — follow mode discovery/entry, orientation-aware switching, two- versus four-lane DisplayPort allocation, AUX over SBU, and hot-plug signaling over the Type-C control path. Explain the consequences for simultaneous SuperSpeed USB and the separate USB 2.0 pair; contrast this physical lane assignment with DisplayPort tunneled over USB4. [VESA DisplayPort over USB-C](https://www.displayport.org/displayport-over-usb-c/).
- [[Why Display Adapters and Cables Can Be Directional]] — trace DisplayPort-to-HDMI, HDMI-to-DisplayPort, digital-to-VGA conversion, and an active optical cable. Distinguish compatible signaling, protocol conversion, DACs, power requirements, and source/sink placement; account for capability reads and hot-plug feedback. Explain why reversing the connectors or removing a feedback wire does not necessarily preserve video, and what an EDID emulator actually replaces.
- [[Where Display Bandwidth Goes between GPU and Monitor]] — calculate pixel rate and payload from resolution, refresh rate, timing overhead, color depth, and chroma format, then account for link encoding and optional DSC. Compare VGA, HDMI, DisplayPort, and USB-C display paths; trace MST/dock bandwidth sharing and distinguish insufficient bandwidth, link errors, compression, and panel refresh limits.

### What each deep dive should include

- One named platform/version and a concrete input, with the data representation and ownership at each stage.
- A sequence diagram plus a small worked example, trace, packet capture, or profiler output that makes the mechanism observable.
- Cold versus warm paths, reuse/caching, concurrency, failure modes, and a way to distinguish similar symptoms.
- Links to the existing component notes, primary sources for technical claims, and explicit boundaries between observed behavior, simplified examples, and implementation-specific details.

## Local coding-agent track backlog (RTX 5090 + Qwen 27B family)

- Qwen 27B lineage notes (models that fit in 32 GB):
  - [[Qwen3.8-27B Local]] — MTP head, context length, agentic-coding tuning, and local inference requirements
  - [[Qwen 27B Family Comparison]] — Qwen3.5-27B → Qwen3.6-27B → Qwen3.8-27B: architecture (Gated Delta Networks, MTP), benchmarks, which quant to run on a 5090
  - [[Gemma 3 27B]] — main non-Qwen 5090 coding alternative
- Inference speed techniques (why a 27B flies on a 5090):
  - [[Speculative Decoding]] — draft-verify mechanics, acceptance rates, when it pays off
  - [[Multi-Token Prediction]] — the Qwen3.6/3.8 MTP head; enabling it in llama.cpp and vLLM
  - [[DFlash]] — block-diffusion speculative decoding; DFlash 2 for Qwen3-27B; contrast with MTP
  - [[KV Cache Quantization]] — Q8_0/Q4_0 KV cache; context-vs-VRAM tradeoff on 32 GB
  - [[GGUF Quantization Levels]] — Q4_K_M vs Q5_K_M vs Q6_K for a 27B on the 5090
  - [[Flash Attention]] — what it does, where the flags show up in llama.cpp/vLLM
- Context management (the small-model killer):
  - [[Context Window Management for Coding Agents]] — long-context degradation, session structuring, when to start a new session
  - [[Context Compaction]] — text-summarization compaction: how and when the agent rewrites its own history
  - [[SnapCompact]] — oh-my-pi bitmap-frame context compression; renders old history as dense PNG frames a vision model reads back
  - [[LLM Prompt Caching]] — prefix caching in llama.cpp/vLLM/Ollama; latency and cost
- Serving engines:
  - [[llama.cpp]] — the GGUF + MTP path; flags that matter on a 5090
  - [[Ollama]] — the easy path; what you give up vs llama.cpp
  - [[vLLM]] — OpenAI-compatible serving; when it beats llama.cpp
- Evaluation:
  - [[SWE-bench]] — what it measures, where the Qwen family sits
- Harness and workflow (oh-my-pi):
  - Expand [[oh-my-pi]]: permissions/approval model, agent config, local-model setup
  - [[Effective Coding with Small Local Models]] — practical playbook: task decomposition, subagents, context hygiene, prompt patterns that fit a 27B's working set

## Multimodal VLM comparison backlog

- Write/expand a note comparing drone-vision and aerial-style alternatives to Miril-Drone-2B-1:
  - [[Miril-Drone-2B-1]] as baseline
  - [[Qwen2.5-VL-7B-Instruct]]
  - [[InternVL2_5-8B]]
  - [[MiniCPM-V-2]]
  - [[MiniCPM-o-2_6]]
  - [[deepseek-vl2]]
  - [[llava-onevision-qwen2-7b-ov-hf]]
  - [[Idefics3-8B-Llama3]]
  - [[kosmos-2.5]]
  - [[paligemma2-28b-pt-896]]
  - Include capability comparison (latency, context, multi-image, structured output, edge feasibility)

## RF / drone telemetry backlog (ExpressLRS LoRa transcript follow-up)

- Create deep-dive notes from the long-range RC receiver video:
  - [[LoRa Decoding Below Noise Floor]]
  - [[Chirp Dechirp Processing Gain]]
  - [[LoRa Dechirp-and-FFT Pipeline]]
  - [[QAM Constellation to SNR Margin]]
  - [[2.4 GHz vs 900 MHz for Drones]]
  - [[Forest and Foliage Effects on 2.4 GHz Links]]
  - [[ExpressLRS Receiver Hardware Architecture]]
  - [[ExpressLRS RX Diversity and Dual-Front-end Layout]]
  - [[ExpressLRS Antenna Types: Chip vs External SMA]]
  - [[PCB RF Design Rules for 2.4 GHz Modules]]
  - [[50 Ohm RF Impedance Matching at PCB Trace Speed]]
  - [[Ground Return Current and Via Placement in RF Layouts]]
  - [[What Does “Vias in RF” Mean: A Practical Guide]]
  - [[ELRS Range-Test Methodology for Vehicles]]
  - [[RX Noise Figure and Link Margin in RC Links]]
  - [[ESP32 Roles in Open-Source RC Modules]]

- Update existing notes with hard data from these concepts:
  - [[ExpressLRS]] should include explicit PHY explanation (FSK/LoRa tradeoffs, why CSS still wins for range)
  - [[Chirp Spread Spectrum (CSS)]] should include dechirp math intuition, bin concentration, and processing gain examples
  - [[Frequency Hopping Spread Spectrum]] and [[Adaptive Frequency Hopping]] should include ELRS-specific hop-map behavior
  - [[Interference-Hardened Radio Design for UAVs]] should include explicit transmission-line/impedance checks and ground-plane loop arguments

## CFD / linear-algebra backlog (AMGX-aligned)

- Prioritize fast single-phase incompressible notes for practical external-flow workflows:
  - [[simpleFoam]] + [[SIMPLE]] + [[Pressure-velocity coupling]] as steady baseline and pseudo-transient warm start
  - [[pisoFoam]] and [[PISO]] as default transient incompressible reference
  - [[pimpleFoam]] for transient single-phase external flows with stronger coupling and larger time steps
  - [[icoFoam]] for laminar educational transients and low-Re sanity checks
  - External single-phase incompressible comparison matrix: `simpleFoam` vs `pisoFoam` vs `pimpleFoam` with recommendations by mesh/Co/time-scale
- Add solver-type notes for reuse from hub taxonomy:
  - Pressure-coupling families (steady/transient/mixed)
  - [[SIMPLE]], [[Incompressible]], [[Compressible]], [[Low-Mach]], [[High-Mach]], [[Density-based]]
  - [[multiphase]], `interFoam`, `compressibleInterFoam`
  - `rhoSimpleFoam`, `rhoPimpleFoam`, `rhoCentralFoam`, `sonicFoam`
  - `buoyantSimpleFoam`, `buoyantPimpleFoam`, `potentialFoam`
  - `chtMultiRegionFoam`, `reactingFoam`, `fireFoam`
  - `swirlFlowRateInletVelocity`, turbulence startup BCs, rotating mesh/`MRF`
- Create dedicated notes comparing key CFD equation groups:
  - Pressure solvers (continuity-pressure coupling workflows, Poisson/pressure equation)
  - Velocity solvers (momentum transport and coupling sensitivity)
  - Density solvers (compressible frameworks and EOS coupling)
  - Scalar/energy solvers (temperature/species/turbulence transport)
  - Swirl and rotational source setups (BC/operator-level handling, not separate solver family)
- Add AMGX follow-up note in this folder:
  - Comparative accuracy/speed behavior of AMGX vs FOAM-GAMG and PETSc paths
  - Integration pitfalls for external solver stacks in practical OpenFOAM runs
- Add dedicated LBM/GPU-structured notes in this folder:
  - [[FluidX3D]]
  - [[WaLBerla]]
  - [[OpenLB]]
  - [[TCLB]]
  - [[XLB]]
  - Add a short comparison matrix: [[FluidX3D]] vs [[WaLBerla]] vs [[OpenLB]] vs [[TCLB]] vs [[XLB]]
  - [[Voxel CFD]]
  - [[Signed Distance Field (SDF) Geometry Pipeline]]
  - [[Voxelization for CFD]]
  - [[Lattice Boltzmann Method]]
  - [[Sparse Voxel Data Structures]]
  - [[Staircase Error]]
  - [[Volume Fraction (VOF)]]
  - [[Level Set Method]]
  - [[Narrow-Band SDF]]
  - [[Inside-Outside Test]]
  - [[Triangle Voxelization]]
  - [[Marching Cubes]]
  - [[Surface Voxelization]]
  - [[Signed Distance Transform]]
  - [[Scanline Fill]]
  - [[Ray Casting]]
  - [[Fast Geometry Preprocessing in CFD]]
  - [[AMR for Voxel CFD]]
  - [[SDF-to-Mesh Boundary Reconstruction]]
  - Airfoil validation against publicly available coordinates and lift/drag data
  - [[Reynolds Scaling with Voxel LBM]]
  - [[Airfoil camber fidelity under voxel masking]]
  - [[BGK vs TRT ceiling in custom LBM pipelines]]
  - [[Bouzidi vs halfway bounce-back]]
  - [[Cylinder drag/cd residual budget decomposition]]

- Add immediate follow-up notes for AI-accelerated geometry-to-CFD optimization loops:
  - [[Geometry-to-CFD Pipeline]]
  - [[Rule-based Geometry Generation]]
  - [[CAD-to-STEP Export Pipelines]]
  - [[Parasolid Import and Conversion]]
  - [[CFD Surrogate Modeling]]
  - [[Differentiable CFD Optimization Loop]]
  - [[Multi-fidelity CFD Workflows]]
  - [[Bayesian Optimization for CFD]]
  - [[Evolution Strategies for Geometry Search]]
  - [[Gradient-based Geometry Optimization]]
  - [[Implicit Geometry Parameterization]]
  - [[Signed Distance Field (SDF) Geometry Representations]]
  - [[Voxelization for LBM and AI CFD]]
  - [[Boundary Tagging Automation for CFD]]
  - [[Objective Aggregation and Tabulation for Simulation Campaigns]]
  - [[Design of Experiments for CFD]]
  - [[CFD Batch Scheduling for Fast Iteration]]
  - [[Shape Optimization with Constraints]]
  - [[Meshing Failure Detection and Recovery]]
  - [[Numerical Noise Filtering for CFD Objectives]]
  - [[Early Stopping Criteria in Iterative CFD Solvers]]
  - Add notes for geometry front-end and API handoff:
    - [[OpenVSP]]
    - [[OpenVSP File I/O]]
    - [[OpenVSP Python API]]
    - [[openvsp.py]]
    - [[vspscript]]
    - [[VSP3 file format]]
    - [[DES design variable files]]
    - [[OpenVSP ComputeCFDMesh]]
    - [[custom_vsp3 in DAFoam]]
    - [[STL export + snappyHexMesh chain]]
    - [[OpenVSP-to-OpenFOAM conversion]]

## RTS map / unit representation backlog

- Add dedicated note set on RTS map state modeling and map data storage, starting with:
  - [[StarCraft map file formats]]
  - [[Command and Conquer map formats]]
- Add notes for core RTS map data terminology and data structures:
  - Build grid (32×32 pixel build tiles) vs walk grid (8×8 pixel walk tiles)
  - Walkability / buildability masks
  - Pathing grid and pathability checks
  - Placement grid and placement validity checks
  - Terrain height map / height fields
  - Fog-of-war layers and visibility states
  - Unit list storage (x,y + owner + hp/shields/energy/resource attributes)
  - Region decomposition and choke-point graphs
  - Choke-point detection and tactical region boundaries
  - Map properties and map locations (regions, starts, markers)
- Add practical data-structure keywords for future RTS notes:
  - walkability mask
  - placement/buildability mask
  - visibility layer and visibility-state transitions
  - pathing cache + region graph + choke-point graph
  - connected components and contour tracing
  - Voronoi and medial-axis corridor models
  - influence/density style maps
  - CHK chunk IDs and section stacking (`TYPE`, `VER `, `MTXM`, `MASK`, `MRGN`, `UNIT`, `TRIG`)
  - CHK chunk header structure: `name (4-byte tag) + size + payload`
  - C&C map grid encoding and INI section semantics (`[Triggers]`, `[TeamTypes]`, `[Waypoints]`)
  - SC2 map protocol data model terms: `GameInfo`, `StartRaw`, `ObservationRaw`, `ImageData`, `LocalMap`, `local_map_path`
  - SC2 binary overlay planes: `pathing_grid`, `placement_grid`, `terrain_height`, `playable_area`
  - SC2 feature layers: `pathing`, `buildable`, `visibility_map`, `height`, `creep`, `power`
  - unit-world-state fields for fast joins: `tag`, `unit_type`, `alliance`, `health`, `energy`, `shield`, `ownership`, `order`, `display type`
  - SC2 runtime semantics: `DisplayType` (visible/snapshot/hidden/placeholder), `visibility` states, `pathability`
  - map-file containers and workflows: `.SC2Map`, `.SC2Components`, `scenario.chk`, `staredit`, unpacked component trees
  - common RTS map-search terms: A* node index, bucketed frontier, Manhattan/Euclidean heuristic, jump-point pruning, corridor width, bottleneck
- Add notes for higher-level RTS spatial abstractions in AI pipelines:
  - [[Influence maps]]
  - [[Potential fields]]
  - [[Region graph]]
  - [[Voronoi diagrams for game terrain analysis]]
  - [[Connected components for walkable terrain]]
- Suggested source anchors for first pass:
  - Blizzard SC2API: GameInfo struct (pathing_grid, terrain_height, placement_grid)
  - Blizzard SC2API: ObservationInterface (GetUnits, visibility/pathability/placement APIs)
  - CHK format spec (UNIT, MTXM, TILE, MASK, MRGN sections)
  - AIIDE papers on RTS terrain analysis (perimeter/region/chokepoint decomposition)
