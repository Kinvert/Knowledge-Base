# TPOT (Tree-based Pipeline Optimization Tool)

TPOT is an open-source AutoML (Automated Machine Learning) tool built on top of [[scikit-learn]]. It uses **genetic programming** to automatically search through possible machine learning pipelines and optimize them for predictive performance. Instead of manually selecting preprocessing steps, models, and hyperparameters, TPOT explores these combinations and evolves them over time to produce high-performing pipelines.  

---

## Key Concepts

- **Genetic Programming** – TPOT applies evolutionary algorithms (mutations, crossovers, selection) to optimize ML pipelines.
- **Pipeline Optimization** – A "pipeline" includes preprocessing, feature selection, model choice, and hyperparameter tuning.
- **Scikit-learn Backend** – All operators are based on [[scikit-learn]], making it easy to integrate with existing workflows.
- **Fitness Function**: By default, TPOT optimizes pipelines based on cross-validated accuracy, but other scoring metrics can be used.
- **Exportable Pipelines** – Once optimized, TPOT outputs clean Python code for direct reuse.
- **Focus on Tabular Data** – Works best for structured datasets (classification, regression). 

---

## Installation

`pip install tpot`

---

## Basic Usage

`from tpot import TPOTClassifier`

`tpot = TPOTClassifier(generations=5, population_size=20, verbosity=2)`  
`tpot.fit(X_train, y_train)`  
`tpot.score(X_test, y_test)`  
`tpot.export('best_pipeline.py')`

---

## Advantages

- **Automates tedious ML workflow** (model selection, preprocessing, hyperparameters).  
- **Reduces human bias** in choosing algorithms.  
- **Provides reproducible pipelines** in scikit-learn code.  
- **Supports classification, regression, and multilabel tasks**.  

---

## Limitations

- **Computationally expensive**: Evolutionary algorithms take time and resources.  
- **Not always interpretable**: The best pipeline may be complex.  
- **Limited to scikit-learn ecosystem**: It doesn’t natively handle deep learning frameworks.  

---

## Features

- Automated model + hyperparameter selection  
- Built-in cross-validation  
- Support for regression and classification tasks  
- Parallel processing for faster searches  
- Export pipelines as Python code  
- Configurable search space for fine-grained control  

---

## One-liner Examples

- Run TPOT on a dataset:  
  `TPOTClassifier(generations=5, population_size=20, verbosity=2).fit(X_train, y_train)`  

- Export the best pipeline:  
  `tpot.export('best_pipeline.py')`  

- Run TPOT for regression:  
  `TPOTRegressor(generations=10, population_size=50, cv=5).fit(X_train, y_train)`  

---

## Comparisons

| Tool        | Approach                           | Strengths | Weaknesses |
|-------------|------------------------------------|-----------|------------|
| **TPOT**    | Genetic programming (evolutionary search over sklearn pipelines) | Produces interpretable sklearn pipelines, good balance of exploration | Can be slow on very large datasets, limited to sklearn ecosystem |
| **[[Auto-sklearn]]** | Bayesian optimization + meta-learning | Strong model/hyperparameter tuning, ensembling | More complex outputs, less interpretable |
| **[[H2O AutoML]]** | Stacking/ensembling multiple algorithms | Scales well, strong performance, distributed | Pipelines not as transparent, more black-box |
| **[[FLAML]]**   | Cost-effective hyperparameter optimization | Lightweight, efficient for small/medium datasets | Fewer built-in preprocessing steps |
| **[[MLJAR AutoML]]** | Stacked models with interpretability options | Easy to use, human-readable reports | Less customizable search compared to TPOT |
| **[[CuPy]] (comparison for acceleration)** | GPU-accelerated NumPy computations | Extremely fast preprocessing/feature engineering if combined | Not an AutoML tool itself, but useful to pair with TPOT |

---

## Use Cases

- Rapid prototyping for ML problems where domain knowledge is limited  
- Automating feature preprocessing and model selection in tabular datasets  
- Benchmarking manual ML pipelines against automated ones  
- Teaching evolutionary algorithms applied to ML  
- Generating scikit-learn compatible pipelines for integration into production  

---

## Related Topics

- [[scikit-learn]]  
- [[AutoML]]
- [[Auto-sklearn]]  
- [[Hyperparameter Optimization]]  
- [[Genetic Algorithms]]  
- [[CuPy]]
- [[Machine Learning Pipelines]]  
