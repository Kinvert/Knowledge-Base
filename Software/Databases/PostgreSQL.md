---
title: PostgreSQL
aliases:
  - Postgres
  - PostgreSQL Database
  - Postgres DB
tags:
  - databases
  - sql
  - storage
  - backends
---

# PostgreSQL

**PostgreSQL** is a production-grade relational database with strong SQL compliance, ACID transactions, extensible indexing, and a large ecosystem of extensions.

It is often the backbone for metadata-heavy robotics platforms, experiment tracking, annotation storage, and fleet telemetry pipelines.

---

## Core strengths

- ACID-compliant transactions for reliable consistency.
- Rich extension ecosystem (`PostGIS`, `TimescaleDB`, custom types).
- Powerful SQL with complex joins and constraint logic.
- Mature ecosystem for managed and self-hosted deployments.
- Strong community with long-term stability.

---

## Comparison table

| Engine | Best For | Consistency Model | Extensibility | Typical Deployment |
|---|---|---|---|---|
| PostgreSQL | Relational transactional workloads | Strong | Very high | Core service DB |
| MySQL/MariaDB | OLTP web workloads | Strong (default) | Medium | Standard app stacks |
| ClickHouse | Analytical workloads | Weaker for OLTP | Medium | Telemetry analytics |
| TimescaleDB | Time-series on SQL | Strong + TS extensions | Medium | Sensor/time-series pipelines |
| SQLite | Embedded/local single-file apps | Transactional single-process | Low | Edge apps, prototypes |

---

## Common strengths in robotics

- Structured metadata for robot runs and experiments.
- Strong joins across calibration tables, software versions, and logs.
- Reliable transaction semantics for mission state and job scheduling.
- `JSONB` support for semi-structured payloads.

---

## Common caveats

- OLAP-heavy workloads may require scaling or a companion columnar engine.
- Operational tuning is nontrivial for high-ingest systems.
- Large unbounded telemetry tables need lifecycle policies.

---

## Tooling and commands

- `psql` for interactive SQL inspection.
- `pg_dump` / `pg_restore` for backups.
- `VACUUM` / `ANALYZE` for maintenance.
- `pg_stat_statements` for slow-query visibility.

---

## Related notes

- [[Databases]]
- [[TimescaleDB]]
- [[ClickHouse]]
- [[SQL]]
- [[ORM]]

