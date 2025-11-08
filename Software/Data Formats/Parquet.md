# Parquet

Parquet is a columnar data storage format optimized for analytical workloads. It is widely used in finance, ML, robotics telemetry mining, and any environment where you need efficient column scans and partial reads. It is extremely common in backtesting engines because price/time data is columnar.

---

## 📘 Overview

Parquet is a binary columnar file format invented for big-data analytics. It stores columns independently with type info, compression, and encodings. Engines like Spark, DuckDB, Polars, Pandas, vectorbt, backtesting engines, and RL pipelines all use Parquet because it is fast, compact, and supports selective column read.

---

## 📝 Summary

- Binary columnar  
- Very good compression  
- Very fast partial read  
- Excellent for multi-column numerical/financial data  
- Supports schema + metadata inside file

---

## ⚙ Core Concepts

- **Columnar** (read only columns you need)  
- **Encodings** (RLE, dictionary encoding, etc)  
- **Predicate pushdown** (query engine only reads relevant chunks)  
- **Schema** (each column has a type)

---

## 📊 Comparison Chart (relative to similar formats)

| Format | Columnar | Compression | Schema | Read Speed (analytics) | Best For |
|---|:---:|:---:|:---:|---:|---|
| Parquet | ✅ | Excellent | ✅ | Very Fast | Analytics workloads |
| CSV | ❌ | None | ❌ | Slow | Interchange, human readable |
| HDF5 | sorta | good | yes | good | scientific computing |
| Feather/Arrow IPC | ✅ | good | ✅ | blazing | columnar interchange |
| SQLite | row-based | depends | schema | moderate | relational queries |
| JSON | ❌ | none | implicit | very slow | interchange/human readable |

---

## 🧪 Use Cases

- Finance backtesting engines storing per-symbol OHLCV  
- Tick/L2 storage with partitioning by date  
- RL market replay storing “frames” of market state  
- Large robotics telemetry logs (columnar is much faster to query later for analysis)  
- Feature stores

---

## ✅ Pros

- Best general purpose storage format for **large numeric tables**  
- Efficient with time-series (as long as columnar)  
- Works with S3 / MinIO / object stores extremely well  
- Very good with DuckDB / Polars (finance workflows)

---

## ❌ Cons

- Not human readable  
- Single-file writes are not append-friendly (need partition layout)  
- Metadata schema changes require care (versioning, schema evolution)

---

## 🔧 Key Features

- Compression per column  
- Nested data support (not just flat tables)  
- Supports predicate pushdown (query engine does less work)  
- Often used with partitioned directories not single gigantic file

---

## 🔗 Related Concepts / Notes

- - [[OHLCV Data]]  
- - [[Backtesting Engines]]
- - [[Finance Backtesting Engines]]
- - [[Market Replay Engine]]  
- - [[vectorbt]]  
- - [[Reinforcement Learning]]

---

## 🌐 External Resources

- Apache Parquet official docs  
- DuckDB / Polars documentation on Parquet I/O

---

## 🔍 Capabilities

- Very fast analytical scans  
- Can skip irrelevant columns  
- Can skip entire row-groups via metadata predicates

---

## 🧠 Key Highlights

- Parquet is one of the few formats that scales finance research from laptop → cluster easily.  
- Most modern “quant data lakes” are Parquet in S3.

