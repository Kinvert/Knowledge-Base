# 🥒 Pickle (Python Serialization)

`pickle` is a built-in Python module used for serializing and deserializing Python objects — the process of converting Python objects into a byte stream (`pickling`) and reconstructing them back into objects (`unpickling`). It’s a key tool for persisting data, caching computations, or transmitting Python objects between processes or systems, particularly when working with robotics applications that exchange structured data or model states.

---

## ⚙️ Overview

`pickle` enables saving almost any Python object (lists, dictionaries, classes, etc.) to a file or sending it over a network as a binary stream.  
The pickled byte stream can later be reloaded to restore the exact object state.

It’s widely used for:
- Storing trained models or intermediate computation results.
- Transferring objects between processes using `multiprocessing`.
- Saving robot states or configurations during runtime.

---

## 🧠 Core Concepts

- **Pickling**: The process of converting a Python object into a byte stream (`pickle.dump(obj, file)` or `pickle.dumps(obj)`).
- **Unpickling**: Reconstructing the object from the byte stream (`pickle.load(file)` or `pickle.loads(data)`).
- **Binary vs Text**: Pickle outputs binary data; files should be opened in binary mode (`'wb'` or `'rb'`).
- **Protocol Versions**: Pickle protocols define serialization formats; higher versions support more features and better efficiency.  
  Example: `protocol=pickle.HIGHEST_PROTOCOL`.

---

## 📊 Comparison Chart

| Feature / Library        | `pickle` | `json` | `marshal` | `dill` | `joblib` | `protobuf` |
|---------------------------|----------|---------|------------|---------|-----------|-------------|
| Language-Specific         | ✅ Yes (Python-only) | ❌ No | ✅ Yes | ✅ Yes | ✅ Yes | ❌ No |
| Handles Custom Classes    | ✅ | ❌ | ⚠️ Limited | ✅ | ✅ | ⚠️ With schema |
| Human-Readable            | ❌ Binary | ✅ Yes | ❌ Binary | ❌ Binary | ❌ Binary | ❌ Binary |
| Safe for Untrusted Input  | ❌ No | ✅ Yes | ❌ No | ❌ No | ❌ No | ✅ Yes |
| Cross-Language Compatible | ❌ No | ✅ Yes | ❌ No | ❌ No | ❌ No | ✅ Yes |
| Typical Use Case          | Python data persistence | Configs / Web | Python internals | Complex objects | ML models | Network data |

---

## 🧰 Use Cases

- Persisting robotic calibration parameters between sessions.
- Saving preprocessed data or neural network weights.
- Caching intermediate steps in motion planning or SLAM.
- Sharing Python objects between processes using [[Multiprocessing]].
- Quick checkpointing during long-running computations.

---

## ✅ Strengths

- Native to Python (no external dependencies).
- Can serialize nearly all Python objects.
- Supports recursive and complex structures.
- Fast binary format for in-memory use.

---

## ❌ Weaknesses

- **Not secure**: Loading pickled data from untrusted sources can execute arbitrary code.
- **Python-specific**: Incompatible with other programming languages.
- **Version fragility**: Pickles from different Python versions may break.
- **Larger output size** compared to structured formats like JSON or MessagePack.

---

## 🔧 Developer Tools and Alternatives

- `dill`: Extends `pickle` to support lambdas, closures, and more.
- `joblib`: Efficient for large NumPy arrays and scikit-learn models.
- `cloudpickle`: Used by distributed frameworks like Dask and Ray.
- `jsonpickle`: Serializes objects to JSON for better cross-language support.
- `protobuf`: For robust, language-independent serialization.

---

## 🔒 Security Warning

Never unpickle data from untrusted or unauthenticated sources.  
`pickle.load()` can execute arbitrary Python code, making it a potential attack vector.  
For safer alternatives, consider:
- `json` for text-based formats.
- `protobuf` or `msgpack` for binary-safe structured data.

---

## 📚 Related Concepts / Notes

- [[Multiprocessing]] (Inter-process communication)
- [[JSON]] (JavaScript Object Notation)
- [[Joblib]] (Efficient serialization for ML)
- [[Dill]] (Extended pickling)
- [[Protocol Buffers]] (Cross-language serialization)
- [[Data Serialization]] (General concept)

---

## 🌐 External Resources

- [Python Official Docs: pickle](https://docs.python.org/3/library/pickle.html)
- [Security Notice – Python Docs](https://docs.python.org/3/library/pickle.html#module-pickle)
- [Joblib Documentation](https://joblib.readthedocs.io/)
- [Dill GitHub Repository](https://github.com/uqfoundation/dill)

---
