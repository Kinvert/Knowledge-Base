# ORM (Object-Relational Mapping)

Object-Relational Mapping (ORM) is a technique that bridges the gap between object-oriented programming and relational databases. ORMs allow developers to work with database records as if they were native objects in their programming language, abstracting away SQL while enabling cleaner logic, faster iteration, and safer database interactions. In Reinforcement Learning and related engineering work, ORMs can help manage experiment metadata, logging, configuration, and model registries.

---

## 🧩 Overview

Object-Relational Mapping tools translate high-level language objects into database tables, rows, and relationships. Instead of writing SQL manually, developers define classes that correspond to database structures. Popular ORMs include Django ORM (Python), SQLAlchemy (Python), ActiveRecord (Ruby), Sequelize (JavaScript), and Ecto (Elixir). ORMs vary widely in philosophy, from “fully abstract SQL” (Django) to “SQL-first” (SQLAlchemy core).

---

## 🧠 Core Concepts

- **Models:** Classes that map to database tables  
- **Fields/Attributes:** Class attributes mapping to columns  
- **Relationships:** One-to-one, one-to-many, many-to-many associations  
- **Query Builders:** High-level APIs to build SQL queries  
- **Migrations:** Version-controlled schema changes  
- **Object Lifecycle:** Creating, reading, updating, and deleting entities  
- **Unit-of-Work Pattern:** Batching operations into transactions (e.g., SQLAlchemy)

---

## 📊 Comparison Chart

| ORM / Feature | Language | Query Style | Philosophy | Schema Migrations | Best For |
|---------------|----------|-------------|------------|-------------------|----------|
| Django ORM | Python | Fully abstracted, minimal SQL | Convention-driven | Built-in | Rapid backend dev |
| SQLAlchemy | Python | SQL or ORM | Explicit control | Alembic | Complex systems |
| Peewee | Python | Lightweight ORM | Simple | Built-in | Small apps, scripts |
| Sequelize | JavaScript | Builder pattern | Flexible | Third-party | Node services |
| Ecto | Elixir | Composable queries | Functional design | Strong migrations | Phoenix apps |
| Hibernate | Java | HQL + ORM | Enterprise-scale | Strong | Java enterprise |

---

## ⚙️ How It Works

ORMs maintain metadata that maps classes to database tables and translate object-oriented method calls into SQL queries. At runtime:
1. Definitions (e.g., models) register themselves with an ORM metadata registry  
2. Queries construct an abstract syntax tree  
3. The ORM compiles that AST into SQL  
4. The database returns rows  
5. Rows are converted into language objects  
6. Object changes can be persisted via transactions

This abstraction improves safety and developer speed but adds complexity and performance overhead in certain situations.

---

## 🛠️ Use Cases

- Experiment tracking for RL agents  
- Storing hyperparameters, metadata, and rollouts  
- Managing user access levels or dashboards in RL pipelines  
- Model registries and versioning  
- General full-stack development for web interfaces around RL tooling  
- CRUD-heavy applications and dashboards  
- Prototyping data-heavy backends  

---

## 🌟 Strengths

- Rapid development  
- Type-safe interactions with the database  
- Reduced boilerplate SQL  
- Built-in validation and relationships  
- Database-agnostic designs  
- Useful abstractions for RL experiment management  

---

## ⚠️ Weaknesses

- Performance overhead vs handwritten SQL  
- Complex queries can become unreadable or inefficient  
- Hard to debug generated SQL  
- Can hide essential database concepts  
- Schema migrations vary widely in stability and reliability  

---

## 🔧 Developer Tools

- Django Admin for fast data inspection  
- SQLAlchemy ORM with Alembic migration tooling  
- Ecto’s `mix ecto.migrate` workflow  
- Sequelize CLI and TypeScript support  
- ORM visualizers such as ERDs and schema explorers  

---

## 🧬 Variants & Styles

- **Active Record** (Django, Rails): Models contain business logic  
- **Data Mapper** (SQLAlchemy): Models are separated from persistence logic  
- **Functional Query Builders** (Ecto): Queries are composable expressions  
- **Lightweight Micro-ORMs** (Peewee, GORM): Minimal feature set, fast to adopt  

---

## 🔍 Related Concepts / Notes

- [[SQL]] (Structured Query Language)  
- [[PostgreSQL]] (Relational Database)  
- [[SQLite]] (Embedded Database)  
- [[DataFrames]] (Tabular Structures)  
- [[Experiment Tracking]] (Metadata for RL)  
- [[MLOps]] (Machine Learning Operations)  
- [[Serialization]] (Storing Objects)  
- [[Python]] (Programming Language)

---

## 🔌 Compatible Items

- PostgreSQL  
- MySQL  
- SQLite  
- MariaDB  
- Oracle DB  
- MS SQL Server  
- Cloud managed SQL services (RDS, Cloud SQL, etc)

---

## 📚 External Resources

- Django ORM Documentation  
- SQLAlchemy ORM User Guide  
- Ecto and Phoenix Framework docs  
- Sequelize Docs  
- PostgreSQL official documentation  

---

## 📝 Summary

ORMs create a unified way for engineers to interact with relational databases using objects instead of SQL. In RL systems, ORMs help maintain organized experiment data, log metadata, and build interfaces for deployment. Choosing the right ORM depends on control needs, scale, language, and project philosophy.
