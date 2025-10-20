# Ethereum Virtual Machine (EVM)

The **Ethereum Virtual Machine (EVM)** is the decentralized computation engine that runs smart contracts on the Ethereum blockchain. It provides a deterministic, sandboxed environment for executing bytecode instructions, ensuring that all nodes in the network reach consensus on computation results. The EVM is sometimes described as a "world computer" — a globally distributed virtual CPU and memory system governed by consensus.

---

## ⚙️ Overview

The EVM is a **state machine** that processes transactions and updates the Ethereum state accordingly. Each transaction is executed as bytecode on the EVM, which operates independently of the underlying hardware or operating system of participating nodes.  
It ensures **consistency, security, and determinism** across all Ethereum clients, regardless of programming language or machine architecture.

---

## 🧠 Core Concepts

- **Bytecode**: Smart contracts are compiled to EVM bytecode, a low-level instruction set similar to assembly.
- **Opcodes**: The EVM has about 140 operation codes, covering arithmetic, logic, stack management, and blockchain-specific operations.
- **Gas**: Each instruction costs a certain amount of `gas`, preventing infinite loops and allocating computational resources fairly.
- **Accounts**: Two types exist — externally owned accounts (EOAs) controlled by private keys, and contract accounts controlled by EVM code.
- **Determinism**: Every node must produce identical results for the same input.
- **Storage and Memory**: Persistent `storage` is expensive and written to the blockchain, while transient `memory` and `stack` are cheaper but temporary.

---

## ⚖️ Comparison Chart

| Feature / Platform | **EVM (Ethereum)** | **WASM (WebAssembly)** | **Solana VM (Sealevel)** | **Bitcoin Script** | **Hyperledger Fabric** |
|--------------------|--------------------|-------------------------|---------------------------|--------------------|-------------------------|
| Type | Stack-based VM | Stack-based binary format | Parallel execution engine | Non-Turing complete | Modular chain framework |
| Deterministic | Yes | Yes (when sandboxed) | Yes | Yes | Yes |
| Parallelism | Limited | Moderate | High | N/A | Configurable |
| Smart Contracts | Yes | Yes | Yes | Limited | Yes |
| Language Support | Solidity, Vyper | Rust, C, C++ | Rust, C | Custom scripts | Go, Java, Node.js |
| Consensus | Proof-of-Stake | N/A | Proof-of-History | Proof-of-Work | Pluggable |
| Execution Cost | Gas model | Depends on host | Compute units | Fixed | Varies |

---

## 🧩 Use Cases

- Execution of decentralized applications (DApps)
- Deployment of **DeFi** protocols and smart contracts
- Token standards like **[[ERC-20]]** and **[[ERC-721]]**
- On-chain automation via smart contracts
- Cross-chain interoperability (EVM-compatible networks)

---

## 💪 Strengths

- Widely adopted across multiple chains (Ethereum, Polygon, Avalanche, BSC)
- Mature developer ecosystem and tooling
- Deterministic and isolated execution model
- Rich library and contract standard support
- Gas-based resource control for network safety

---

## ⚠️ Weaknesses

- Limited computational performance due to global consensus
- High gas costs during network congestion
- Sequential execution model limits scalability
- Difficult to formally verify complex contracts
- Prone to security risks if smart contracts are poorly written

---

## 🧰 Developer Tools

- **Solidity** (primary language for smart contracts)
- **Remix IDE** (browser-based development environment)
- **Hardhat**, **Truffle**, **Foundry** (testing and deployment frameworks)
- **Ganache** (local blockchain simulator)
- **Ethers.js** and **Web3.js** (interaction libraries)

---

## 🧾 Variants and Compatible Networks

- **Ethereum Mainnet** — canonical EVM environment  
- **Polygon**, **Binance Smart Chain**, **Avalanche**, **Fantom**, **Arbitrum**, **Optimism** — all EVM-compatible chains  
- **eWASM** — planned future replacement for the EVM using WebAssembly for better performance and language support

---

## 🔍 Related Concepts and Notes

- [[Blockchain]] (Distributed ledger technology)
- [[Smart Contract]] (Automated on-chain programs)
- [[Solidity]] (EVM smart contract language)
- [[Gas]] (Execution cost unit)
- [[Consensus Mechanisms]] (Validation processes like PoS)
- [[Layer 2 Scaling]] (e.g., Optimistic and ZK Rollups)
- [[Decentralized Applications]] (DApps)
- [[WASM]] (WebAssembly, next-gen virtual machine)

---

## 📚 External Resources

- [Ethereum Yellow Paper](https://ethereum.github.io/yellowpaper/paper.pdf)
- [EVM Opcodes Reference](https://www.ethervm.io/)
- [Solidity Documentation](https://docs.soliditylang.org/)
- [Ethereum.org Developer Portal](https://ethereum.org/en/developers/)
- [Etherscan Bytecode Viewer](https://etherscan.io/)

---

## 🧭 Summary

The **EVM** is the computational heart of Ethereum, enforcing deterministic smart contract execution and enabling trustless automation across a global network. Despite performance limitations, its widespread adoption and compatibility have made it the **de facto standard for blockchain-based computation**, influencing nearly every modern blockchain project.

---
