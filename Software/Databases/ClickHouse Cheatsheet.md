# ClickHouse Cheatsheet

ClickHouse is a **column-oriented, high-performance analytical database** used for processing large datasets in real time. It excels at analytical queries (OLAP), streaming data ingestion, and log or telemetry analysis. This cheatsheet covers essential CLI usage, connection options, query patterns, export/import commands, and safe operation tips.

---

## ⚙️ Overview

ClickHouse is designed for **speed, compression, and scalability**, optimized for read-heavy analytics rather than transactional workloads.  
It’s widely used in telemetry, finance, ad tech, and robotics data aggregation where fast aggregation on massive datasets is required.

---

## 🧭 Connecting to ClickHouse

`clickhouse-client` is the standard command-line interface for interacting with the database.

- Connect locally: `clickhouse-client`
- Connect to remote host: `clickhouse-client --host my-server`
- Specify user and password: `clickhouse-client --user myuser --password mypass`
- Select database: `clickhouse-client --database mydb`
- Combine options: `clickhouse-client --host my-server --user myuser --password mypass --database mydb`
- Run a single query: `clickhouse-client --query "SELECT * FROM table LIMIT 10"`
- Export query result to CSV: `clickhouse-client --query "SELECT * FROM table" --format CSVWithNames > output.csv`
- Execute multi-line queries interactively: `clickhouse-client --multiline`

---

## 📊 Basic Query Examples

- Show databases: `clickhouse-client --query "SHOW DATABASES"`
- Show tables in a DB: `clickhouse-client --query "SHOW TABLES FROM mydb"`
- Describe table: `clickhouse-client --query "DESCRIBE TABLE mydb.mytable"`
- Select first rows: `clickhouse-client --query "SELECT * FROM mydb.mytable LIMIT 10"`
- Filter data: `clickhouse-client --query "SELECT * FROM mytable WHERE value > 10"`
- Count rows: `clickhouse-client --query "SELECT count() FROM mytable"`
- Distinct values: `clickhouse-client --query "SELECT DISTINCT column FROM mytable"`
- Aggregate data: `clickhouse-client --query "SELECT column, count() FROM mytable GROUP BY column"`
- Order results: `clickhouse-client --query "SELECT * FROM mytable ORDER BY timestamp DESC LIMIT 20"`

---

## 🧱 Database and Table Management

⚠️ **Caution:** Be extremely careful with DROP, ALTER, and TRUNCATE — they can permanently delete data.

- Create database: `clickhouse-client --query "CREATE DATABASE mydb"`
- Create table: `clickhouse-client --query "CREATE TABLE mydb.mytable (id UInt32, name String) ENGINE = MergeTree() ORDER BY id"`
- Show create statement: `clickhouse-client --query "SHOW CREATE TABLE mydb.mytable"`
- Drop table (⚠️ permanent): `clickhouse-client --query "DROP TABLE mydb.mytable"`
- Drop database (⚠️ permanent): `clickhouse-client --query "DROP DATABASE mydb"`
- Truncate table (⚠️ clears all data): `clickhouse-client --query "TRUNCATE TABLE mydb.mytable"`

---

## 💾 Data Ingestion and Export

- Insert inline data: `clickhouse-client --query "INSERT INTO mydb.mytable VALUES (1, 'Alice'), (2, 'Bob')"`
- Insert from file: `clickhouse-client --query "INSERT INTO mydb.mytable FORMAT CSV" < data.csv`
- Export as CSV: `clickhouse-client --query "SELECT * FROM mydb.mytable" --format CSVWithNames > data.csv`
- Export as TSV: `clickhouse-client --query "SELECT * FROM mydb.mytable" --format TabSeparatedWithNames > data.tsv`
- Export as JSON: `clickhouse-client --query "SELECT * FROM mydb.mytable" --format JSONEachRow > data.json`

---

## 📈 Analytical Query Examples

- Average: `clickhouse-client --query "SELECT avg(column) FROM mytable"`
- Min/Max: `clickhouse-client --query "SELECT min(column), max(column) FROM mytable"`
- Time grouping: `clickhouse-client --query "SELECT toStartOfHour(timestamp) AS hour, count() FROM mytable GROUP BY hour ORDER BY hour"`
- Sampling (fast overview): `clickhouse-client --query "SELECT * FROM mytable SAMPLE 0.1 LIMIT 100"`
- Top-N query: `clickhouse-client --query "SELECT item, count() AS c FROM mytable GROUP BY item ORDER BY c DESC LIMIT 10"`

---

## ⚡ Performance and Tuning

- Check query speed: `clickhouse-client --time --query "SELECT count() FROM mytable"`
- Limit memory usage: `clickhouse-client --max_memory_usage 1000000000 --query "SELECT * FROM mytable"`
- View current settings: `clickhouse-client --query "SHOW SETTINGS"`
- Set temporary parameter: `clickhouse-client --query "SET max_threads = 4; SELECT count() FROM mytable"`

---

## 🧮 Useful Functions

- Date parsing: `toDateTime('2025-01-01 00:00:00')`
- Hashing: `cityHash64(column)`
- Array aggregation: `groupArray(column)`
- Unique count: `uniq(column)`
- Conditional logic: `if(condition, then_value, else_value)`
- String split: `splitByChar(',', string_column)`
- Join example: `SELECT * FROM t1 JOIN t2 USING (id)`

---

## 🧰 Administration and Inspection

- Check server status: `systemctl status clickhouse-server`
- Restart service: `sudo systemctl restart clickhouse-server`
- View logs: `sudo tail -f /var/log/clickhouse-server/clickhouse-server.log`
- Show running queries: `clickhouse-client --query "SHOW PROCESSLIST"`
- Kill query: `clickhouse-client --query "KILL QUERY WHERE query_id='abcd1234'"`

---

## 🧨 Caution & Safety Tips

⚠️ **Never run DROP or TRUNCATE commands unless absolutely certain.**
- Always back up before major schema or data changes.  
- Test dangerous queries with `--query "EXPLAIN SELECT ..."` first.  
- Avoid running write commands (`INSERT`, `ALTER`, `DROP`) as root user.  
- Use `LIMIT` when testing SELECT queries on huge datasets.  
- Prefer `DETACH TABLE` instead of `DROP TABLE` for safe removal (keeps data on disk).  
- Set read-only mode temporarily: `clickhouse-client --readonly 1`  
- Regularly back up `/var/lib/clickhouse/` or use replicated tables.

---

## 🧩 Related Concepts

- [[ClickHouse]]
- [[SQL]]
- [[PostgreSQL]]
- [[InfluxDB]]
- [[TimescaleDB]]
- [[Grafana]]
- [[OLAP]]
- [[Data Warehouse]]
- [[Column-Oriented Storage]]
- [[ETL Pipelines]]

---

## 📚 External Resources

- Official Docs: [https://clickhouse.com/docs](https://clickhouse.com/docs)
- GitHub Repo: [https://github.com/ClickHouse/ClickHouse](https://github.com/ClickHouse/ClickHouse)
- Awesome ClickHouse: [https://github.com/ClickHouse/awesome-clickhouse](https://github.com/ClickHouse/awesome-clickhouse)
- ClickHouse Cloud: [https://clickhouse.com/cloud](https://clickhouse.com/cloud)

---

## 🧭 Summary

ClickHouse provides **extreme query speed** for analytical workloads through **columnar storage, compression, and vectorized execution**.  
Ideal for telemetry, metrics, and robotics logs where performance and scale matter.  
Use it carefully — while blazing fast, destructive operations are immediate and irreversible.
