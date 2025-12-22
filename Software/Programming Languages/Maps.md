# Maps

Maps (also called dictionaries, hash tables, associative arrays, or hash maps) are fundamental data structures that store key-value pairs, enabling efficient lookup, insertion, and deletion based on keys. They are essential for countless programming tasks, from caching and indexing to counting and grouping. Understanding how different languages implement maps—their hashing strategies, collision resolution, memory models, and performance characteristics—is critical for writing efficient code.

## 🎯 Overview

---

A map associates keys with values, allowing O(1) average-case lookup by key. Unlike lists which use integer indices, maps can use any hashable or comparable type as keys. The fundamental operations are insertion, lookup, and deletion, all of which are typically constant-time on average but can degrade to linear time in worst cases.

Maps can be implemented using hash tables (most common), balanced trees (for ordered maps), or other structures. Hash-based maps provide fast average-case performance but no ordering guarantees. Tree-based maps maintain sorted order but have O(log n) operations. The choice dramatically affects both performance and available operations.

## 🧠 Core Concepts

---

**Hash Functions**: Convert keys to integer hash codes that determine storage location. Good hash functions distribute keys uniformly to minimize collisions. Poor hash functions cause performance degradation.

**Collision Resolution**: Handles cases where different keys hash to the same location. Common strategies include chaining (linked lists at each bucket) and open addressing (probing for alternative locations).

**Load Factor**: Ratio of entries to buckets. Higher load factors save memory but increase collisions. Maps typically resize when load factor exceeds threshold (often 0.75).

**Resizing**: When load factor threshold exceeded, allocate larger table (typically 2x), rehash all entries into new table. Expensive operation but amortized over many insertions.

**Key Equality**: Maps need to determine when keys are equal. Hash collision doesn't guarantee equality—must check with equality function. Hash equal keys must have equal hashes, but equal hashes don't guarantee equal keys.

**Ordering**: Hash maps provide no ordering guarantees. Iteration order may be random, insertion-ordered, or arbitrary. Tree maps maintain sorted order by key.

**Mutability**: Like lists, maps can be mutable (allowing in-place changes) or immutable (creating new maps for modifications).

**Reference vs Value Semantics**: Whether assignment copies the map data or just the reference, fundamentally affecting behavior.

## 📊 Comparison Chart

---

| Language | Name | Mutable | Implementation | Ordered | Pass By | Copy on Assign | Lookup | Insert | Delete |
|----------|------|---------|----------------|---------|---------|----------------|--------|--------|--------|
| **Python** | dict | Yes | Hash table | Insertion (3.7+) | Reference | No (ref copy) | O(1) avg | O(1) avg | O(1) avg |
| **C** | N/A | Manual | Manual | Depends | Value* | Manual | Depends | Depends | Depends |
| **C++** | map | Yes | Red-black tree | Yes (sorted) | Value | Yes (deep) | O(log n) | O(log n) | O(log n) |
| **C++** | unordered_map | Yes | Hash table | No | Value | Yes (deep) | O(1) avg | O(1) avg | O(1) avg |
| **Elixir** | Map | No | HAMT | No | Value | Yes (sharing) | O(log n) | O(log n) | O(log n) |
| **Zig** | HashMap | Yes | Hash table | No | Value | Yes | O(1) avg | O(1) avg | O(1) avg |
| **JavaScript** | Map | Yes | Hash table | Insertion | Reference | No (ref copy) | O(1) avg | O(1) avg | O(1) avg |
| **JavaScript** | Object | Yes | Hash table** | No*** | Reference | No (ref copy) | O(1) avg | O(1) avg | O(1) avg |
| **Rust** | HashMap | Yes | Hash table | No | Move/Borrow | Move | O(1) avg | O(1) avg | O(1) avg |
| **Java** | HashMap | Yes | Hash table | No | Reference | No (ref copy) | O(1) avg | O(1) avg | O(1) avg |
| **Java** | TreeMap | Yes | Red-black tree | Yes (sorted) | Reference | No (ref copy) | O(log n) | O(log n) | O(log n) |
| **Go** | map | Yes | Hash table | No | Reference | Shallow copy | O(1) avg | O(1) avg | O(1) avg |
| **Haskell** | Map | No | Balanced tree | Yes (sorted) | Value | Yes (sharing) | O(log n) | O(log n) | O(log n) |

*Depends on implementation, **Engines may optimize, ***Mostly unordered but some special-case ordering

## 🐍 Python Dictionaries

---

Python's `dict` is a hash table implementation with reference semantics that has evolved significantly, particularly in Python 3.6+ where it maintains insertion order.

**Implementation Details**: Python dicts use open addressing with pseudo-random probing. The hash table stores entries (hash, key, value) in a dense array with a separate sparse indices array for lookup. This compact dict implementation (PEP 468) preserves insertion order as a side effect of memory layout optimization.

**Memory Model**: Assignment creates a new reference, not a copy. Multiple variables can reference the same dict. Modifications through any reference affect all. The dict object contains pointers to hash table, size, version (for iteration detection), and other metadata.

**Hash Requirements**: Keys must be hashable—immutable types with `__hash__()` and `__eq__()`. Integers, strings, tuples of hashables are hashable. Lists, dicts, sets are not hashable (mutable). Custom objects hashable by default (identity-based) unless `__eq__` defined without `__hash__`.

**Mutability**: Dicts are fully mutable. `d[key] = value` sets/updates entry, `del d[key]` removes entry, `d.pop(key)` removes and returns value, `d.clear()` removes all entries, `d.update(other)` merges another dict.

**Performance Characteristics**: Average O(1) for lookup, insertion, deletion. Worst case O(n) with many collisions (rare with good hash function). Memory overhead approximately 3x space needed for entries due to load factor and indices array.

**Insertion Order**: Python 3.7+ guarantees insertion order preservation. Iteration yields keys in order they were first inserted. `dict` is now ordered dict by default—`collections.OrderedDict` still exists for explicit ordering semantics.

**Dictionary Comprehensions**: `{key_expr: value_expr for item in iterable if condition}` creates dict from expression. Example: `{x: x**2 for x in range(10) if x % 2 == 0}` creates dict mapping even numbers to squares.

**Common Operations**: `d.get(key, default)` returns value or default if key absent (avoids KeyError), `d.setdefault(key, default)` returns value or sets and returns default, `d.keys()` returns view of keys, `d.values()` returns view of values, `d.items()` returns view of (key, value) pairs.

**Dictionary Views**: `keys()`, `values()`, `items()` return view objects that reflect dict changes. Views are set-like (for keys/items) and iterable. Changes to dict visible in existing views.

**Merging**: Python 3.9+ supports `d1 | d2` for merging (creates new dict), `d1 |= d2` for in-place update. Earlier versions use `d1.update(d2)` or `{**d1, **d2}`.

**Default Dicts**: `collections.defaultdict(factory)` provides default values for missing keys. Example: `defaultdict(list)` returns empty list for missing keys, useful for grouping: `d[key].append(value)` without checking if key exists.

**Counter**: `collections.Counter` is specialized dict for counting. Methods like `most_common(n)`, arithmetic operations, and automatic zero-value for missing keys.

**Hashing Strategy**: Python uses hash function based on SipHash for strings (randomized to prevent hash collision attacks). Integers hash to themselves (for small integers). Tuples combine hashes of elements.

**Internal Structure**: CPython dict has `ma_used` (entry count), `ma_version_tag` (incremented on modification for iterator invalidation), `ma_keys` (pointer to keys table), `ma_values` (pointer to values array or NULL).

**Growth Pattern**: Resizes at 2/3 load factor. New size is next power of 2 that gives load factor < 2/3. Small dicts start at 8 entries, then 16, 32, 64, etc.

**Common Pitfalls**: Modifying dict during iteration raises RuntimeError (dict changed size during iteration). Use `list(d.keys())` to iterate over snapshot. Default values in `get()` don't get inserted—use `setdefault()` for that.

## 🔧 C Hash Tables

---

C has no built-in hash table—must implement manually or use library. Common approaches include uthash (macro-based), glib GHashTable, and custom implementations.

**Manual Implementation**: Typical structure includes array of buckets (pointers to linked lists or entries), size, count, hash function, and equality function. Generic implementations use `void*` for keys/values.

**Chaining Implementation**: Each bucket is linked list of entries. Hash determines bucket, then linear search through list. Simple to implement but can have poor cache locality.
```c
typedef struct Entry {
    void *key;
    void *value;
    struct Entry *next;
} Entry;

typedef struct {
    Entry **buckets;
    size_t size;
    size_t count;
    unsigned (*hash)(void*);
    int (*equals)(void*, void*);
} HashTable;
```

**Hash Functions**: Must write or use library hash functions. FNV-1a, MurmurHash, and djb2 are popular. String hashing example:
```c
unsigned hash_string(const char *str) {
    unsigned hash = 5381;
    int c;
    while ((c = *str++))
        hash = ((hash << 5) + hash) + c;
    return hash;
}
```

**Open Addressing**: Alternative to chaining—store entries directly in array, probe for alternative locations on collision. Linear probing (check next slot), quadratic probing (quadratic sequence), or double hashing (second hash function for step size).

**Memory Management**: Must manually allocate and free. Each insertion may allocate entry node, keys/values if copied. Deletion must free entry and possibly key/value. Clear operation must iterate and free all entries. Failure to free causes memory leaks.

**Resizing**: Must implement manually. When load factor exceeds threshold (e.g., 0.75), allocate new larger table, rehash all entries, free old table. Typically double size to next prime or power of 2.

**No Generic Type Safety**: Using `void*` loses type safety. Inserting wrong type compiles but causes runtime errors. Must maintain discipline or use macro-based approaches.

**uthash Library**: Popular macro-based hash table for C. Adds hash table capability to any struct. Uses macros for type safety. Example:
```c
struct User {
    int id;
    char name[32];
    UT_hash_handle hh;
};

struct User *users = NULL;
HASH_ADD_INT(users, id, user_ptr);
HASH_FIND_INT(users, &id_to_find, found_user);
```

**Performance Considerations**: Cache locality important—chaining can be poor. Robin Hood hashing (open addressing variant) provides better worst-case. Power-of-2 sizes enable fast modulo with bitwise AND.

**Common Patterns**: Function pointers for hash and equality enable generic implementations. Wrapping in struct with function pointers provides object-oriented style. Using macros for type-safe wrappers around generic implementation.

**Pitfalls**: Forgetting to free memory, using freed keys/values, pointer issues (dangling pointers after resize), not handling hash collisions properly, poor hash functions causing clustering.

## ⚡ C++ Maps and Unordered Maps

---

C++ provides `std::map` (ordered, tree-based) and `std::unordered_map` (hash-based) with value semantics and strong type safety.

**std::map Implementation**: Red-black tree (self-balancing binary search tree). Each node contains key, value, parent pointer, left child, right child, and color bit. Maintains sorted order by key.

**std::unordered_map Implementation**: Hash table with chaining (buckets containing linked lists). Template parameters for key type, value type, hash function, equality predicate, and allocator.

**Memory Model**: Maps own their data with value semantics. Assignment performs deep copy. `map<int, string> b = a` creates independent copy. Move semantics (C++11+) enable efficient transfers: `map<int, string> b = std::move(a)` transfers ownership in O(1).

**Key Requirements for std::map**: Keys must be comparable with `operator<`. Standard types (int, string, etc.) work. Custom types need comparison operator. Order determined by `<`, not `==`.

**Key Requirements for std::unordered_map**: Keys must be hashable and equality-comparable. Standard types have specializations. Custom types need `std::hash` specialization and `operator==`.

**Basic Operations**: `m[key]` accesses/inserts (default-constructs if absent), `m.at(key)` accesses with bounds check (throws `std::out_of_range`), `m.insert({key, value})` inserts if absent, `m.emplace(key, value)` constructs in-place, `m.erase(key)` removes entry, `m.find(key)` returns iterator or `end()`.

**Insert vs Emplace**: `insert` takes constructed pair, `emplace` constructs in place. For complex value types, `emplace` avoids temporary object. Example: `m.emplace(std::piecewise_construct, std::forward_as_tuple(key), std::forward_as_tuple(args...))` for complex construction.

**Operator[] Behavior**: Creates entry with default-constructed value if key absent. This can be surprising—`m[key]` on const map doesn't compile (would modify). Use `find()` or `at()` for lookup-only.

**Iterators**: `map::iterator` yields `std::pair<const Key, Value>`. Note const key—can't modify key through iterator (would break invariant). Can modify value. Iteration over `std::map` yields sorted order.

**Iterator Invalidation**: For `std::map`, only iterators to erased elements invalidated. Insert doesn't invalidate. For `std::unordered_map`, rehashing invalidates all iterators. `reserve(n)` prevents rehashing.

**Count and Find**: `m.count(key)` returns 0 or 1 (multi-maps can return higher). `m.find(key)` returns iterator, check against `m.end()`. `contains(key)` (C++20) returns bool directly.

**Custom Hash Function**: For `std::unordered_map` with custom types, provide hash functor:
```cpp
struct MyHash {
    size_t operator()(const MyType& obj) const {
        return std::hash<int>()(obj.id) ^ (std::hash<string>()(obj.name) << 1);
    }
};

std::unordered_map<MyType, Value, MyHash> m;
```

**Load Factor**: `unordered_map` exposes `load_factor()` (count/buckets), `max_load_factor()` (threshold), `max_load_factor(float)` (set threshold), `rehash(n)` (set bucket count), `reserve(n)` (reserve capacity for n elements).

**Bucket Interface**: `bucket_count()`, `bucket_size(n)`, `bucket(key)` (which bucket), `begin(n)`/`end(n)` (iterate bucket n). Useful for analyzing distribution and diagnosing performance issues.

**Structured Bindings**: C++17 enables `for (auto& [key, value] : map)` instead of `for (auto& pair : map)` with `pair.first` and `pair.second`. Much more readable.

**Try Emplace**: C++17 `try_emplace(key, args...)` only constructs value if key absent. Unlike `emplace`, doesn't move key on failed insertion. Better for expensive keys.

**Extract and Merge**: C++17 `extract(key)` removes entry and returns node handle (ownership transferred). Can modify key in node handle, then `insert()` back. `merge(other_map)` transfers entries from other map.

**Heterogeneous Lookup**: C++14+ enables lookup with comparable types without creating temporary key object. Requires custom comparator with `is_transparent`. Example: find in `map<string, V>` using `string_view` without constructing `string`.

**Performance Tuning**: For `unordered_map`, profile bucket distribution with bucket interface. If many collisions, improve hash function. Pre-reserve capacity if size known. For `map`, prefer `find` over `operator[]` for lookup-only to avoid insertion.

**Multi-Maps**: `std::multimap` and `std::unordered_multimap` allow duplicate keys. Use `equal_range(key)` to get iterator range for all entries with key, or `count(key)` for number of entries.

## 💧 Elixir Maps

---

Elixir maps are immutable key-value structures implemented using Hash Array Mapped Tries (HAMT) with structural sharing.

**Implementation Details**: Small maps (≤32 entries) use flat array of key-value pairs with linear search. Larger maps use HAMT—tree structure where hash bits determine path. Efficient for both small and large maps. Immutable structure enables sharing subtrees between map versions.

**Immutability**: Maps can never be modified. All operations return new maps. Due to structural sharing, this is efficient—only modified path from root to changed entry is copied. `map = %{a: 1, b: 2}` followed by `new_map = Map.put(map, :c, 3)` creates new map sharing most structure with original.

**Structural Sharing**: When updating entry, only O(log n) nodes copied along path in tree. Remaining subtrees shared. Memory efficiency despite immutability. Reading old version doesn't interfere with new version.

**Syntax**: Map literals use `%{key => value}` or `%{atom_key: value}` for atom keys. `%{name: "John", age: 30}` for atom keys, `%{1 => "one", "two" => 2}` for mixed types. Atom-key syntax cleaner but only for atom keys.

**Pattern Matching**: Core feature for map manipulation. `%{name: name, age: age} = person` extracts values. `%{name: "John"} = person` matches only if name is "John". `%{name: name} = person` matches any map with name key, binds value.

**Access**: `map[:key]` returns value or `nil` if absent (for atom keys can use `map.key`). `Map.get(map, key, default)` returns value or default. `Map.fetch(map, key)` returns `{:ok, value}` or `:error` tuple. `Map.fetch!(map, key)` returns value or raises.

**Update Syntax**: `%{map | existing_key: new_value}` updates existing key (raises if key doesn't exist). Shorthand for updating existing entries. For potentially new keys, use `Map.put(map, key, value)`.

**Map Module Functions**: `Map.put(map, key, value)` adds/updates entry, `Map.delete(map, key)` removes entry, `Map.has_key?(map, key)` checks existence, `Map.keys(map)` returns list of keys, `Map.values(map)` returns list of values, `Map.merge(map1, map2)` merges maps (right precedence).

**Performance Characteristics**: O(log n) for all operations (get, put, delete) due to HAMT structure. Constant factor is small—32-way branching. Practical performance excellent for maps up to millions of entries.

**Map Update Function**: `Map.update(map, key, default, update_fn)` applies function to value if key exists, otherwise inserts default. `Map.update!(map, key, update_fn)` applies function, raises if key absent.

**Get and Update**: `Map.get_and_update(map, key, fn)` atomically gets and updates in one operation. Function receives current value (or nil), returns `{get_value, new_value}` or `:pop` to delete.

**Key Types**: Any term can be key. Common to use atoms (`:name`), strings, integers. Atoms are interned—same atom always same memory location. Efficient for frequently-used keys.

**Structs**: `defstruct` defines named maps with default values and compile-time key checking. `%User{name: "John"}` creates struct. Structs are maps with `__struct__` key. Pattern matching on structs ensures expected structure.

**Enumeration**: Maps implement `Enumerable` protocol. `Enum.map(map, fn {k, v} -> ... end)`, `Enum.filter(map, fn {k, v} -> ... end)`. Map comprehensions: `for {k, v} <- map, v > 10, into: %{}, do: {k, v * 2}`.

**Nested Updates**: `put_in`, `update_in`, `get_in` for nested access. `put_in(user, [:address, :city], "NYC")` updates nested value. Works with keyword lists and maps. Lens-like functionality built-in.

**Access Behavior**: Maps implement Access behavior. Enables `map[key]` and `get_in`/`put_in`. Can define custom access for structs to control field access.

**Comparison**: Maps compare based on content, not structure. Two maps with same entries are equal regardless of order or internal structure. Enables value-based testing and equality checking.

**Memory Considerations**: Each map version occupies memory until garbage collected. Avoid accumulating many intermediate versions. Process-local garbage collection prevents issues—when process dies, all its memory reclaimed.

**When Not to Use**: For very frequent updates to same keys, ETS (Erlang Term Storage) provides mutable key-value store. For ordered iteration, consider keyword lists (small) or OrderedDict library (larger).

## ⚡ Zig HashMap

---

Zig provides `std.HashMap` and `std.AutoHashMap` for hash table functionality with manual memory management and explicit allocators.

**Implementation**: Open addressing with linear probing. Stores keys, values, and metadata (empty/deleted/occupied) in separate arrays. Power-of-2 sizing for fast modulo via bitwise AND.

**Memory Management**: Must explicitly provide allocator: `var map = std.AutoHashMap(K, V).init(allocator)`. Must call `map.deinit()` to free memory. Zig's ownership patterns help prevent leaks but responsibility is programmer's.

**AutoHashMap vs HashMap**: `AutoHashMap` provides default hash and equality for standard types. `HashMap` requires explicit hash and equality functions: `HashMap(K, V, hash_fn, eql_fn, max_load_percentage)`. Use `AutoHashMap` for convenience, `HashMap` for custom types or hash functions.

**Allocator Pattern**: Like ArrayList, HashMap takes allocator explicitly. Makes allocation strategy visible and controllable. Common allocators: `std.heap.page_allocator`, `std.heap.GeneralPurposeAllocator`, arena allocators, or custom.

**Generic Types**: `AutoHashMap(K, V)` where K and V are any types. Compile-time generics mean no runtime overhead. Can store structs, pointers, or primitives.

**Basic Operations**: `try map.put(key, value)` inserts/updates (returns error on allocation failure), `map.get(key)` returns `?V` (optional value), `map.remove(key)` returns bool indicating if removed, `map.contains(key)` returns bool, `map.count()` returns entry count.

**Error Handling**: Operations that may allocate return errors. `put` returns `error{OutOfMemory}!void`. Must handle with `try`, `catch`, or explicit error union handling. Allocation failure is explicit, not hidden.

**GetOrPut**: `try map.getOrPut(key)` returns `GetOrPutResult` with `found_existing: bool` and `value_ptr: *V`. If key existed, points to existing value; if not, inserts default-initialized value and points to it. Enables conditional initialization.

**Entry API**: `map.getEntry(key)` returns optional pointer to internal entry. `entry.key_ptr` and `entry.value_ptr` provide pointers to key and value. Allows in-place modification without rehashing.

**Iteration**: Iterate with `while (map.iterator().next()) |entry| { ... }` where entry has `key_ptr` and `value_ptr`. Order is arbitrary (determined by hash table layout). Iterator invalidated by modifications.

**Capacity Management**: `try map.ensureTotalCapacity(n)` pre-allocates for n entries, `try map.ensureUnusedCapacity(n)` ensures room for n additional entries. Useful to avoid reallocations when final size known.

**Load Factor**: Default max load percentage is 80%. Can customize when creating `HashMap`. Higher load factors trade space for time (more collisions). Lower load factors trade time for space (more memory, fewer collisions).

**Hash Functions**: `std.hash_map.autoHash` for standard types. Custom hash functions take key and return `u64` (or `u32`). Use `std.hash.Wyhash` or similar for custom types. Must provide consistent hashing (equal keys must hash equally).

**Equality Functions**: `std.hash_map.eqlFn` for standard types. Custom equality takes two keys, returns bool. Must be consistent with hash function and reflexive, symmetric, transitive.

**Cloning**: `try map.clone()` creates deep copy with same allocator. Allocates new hash table and copies all entries. Must deinit clone separately.

**Memory Representation**: HashMap struct contains allocator, metadata array (bytes indicating slot state), keys array, values array, count, and capacity. Separate arrays enable type-specific alignment.

**Comptime Optimizations**: Hash and equality functions can be comptime-known, enabling optimizations. Zig's comptime evaluation can unroll loops, inline functions, and optimize based on key types.

**Context Pattern**: HashMap supports context parameter for custom behavior. Can pass context to hash and equality functions, enabling things like case-insensitive string maps or custom comparison logic.

**No RAII**: Unlike C++, no automatic cleanup. Must explicitly call `deinit()`. Common pattern: `defer map.deinit()` immediately after `init()` to ensure cleanup on scope exit.

**Comparison to Rust**: Similar in manual memory control (no GC), but Zig has explicit allocators vs Rust's global allocator, no borrow checker (programmer ensures safety), error unions vs Result, and comptime vs macros/const generics.

## 🦀 Rust HashMap and BTreeMap

---

Rust provides `HashMap` (hash table) and `BTreeMap` (B-tree) in `std::collections` with ownership semantics and compile-time safety.

**Ownership Model**: HashMaps own their data. Keys and values must be owned or references with appropriate lifetimes. Moving map transfers ownership. Borrowing enables shared or mutable access without transfer.

**HashMap Implementation**: Hash table with robinhood hashing (variant of open addressing). Uses SipHash 1-3 by default for hash function (cryptographically resistant to collision attacks). Can use faster hashers like `FxHash` or `AHash` if security not needed.

**BTreeMap Implementation**: B-tree (not binary tree—each node has many children). Maintains sorted order by key. Better worst-case guarantees (O(log n) vs amortized O(1)). Better cache locality than red-black trees.

**Key Requirements**: Keys must implement `Hash` and `Eq` for HashMap, `Ord` for BTreeMap. Standard types implement these. Custom types need `#[derive(Hash, Eq, PartialEq)]` or manual implementation.

**Basic Operations**: `map.insert(key, value)` inserts/updates returning `Option<V>` (previous value if replaced), `map.get(&key)` returns `Option<&V>`, `map.remove(&key)` removes returning `Option<V>`, `map.contains_key(&key)` returns bool.

**Borrowing for Lookup**: Methods take `&K` not `K` for lookup. Avoids moving key for simple check. `map.get(&key)` borrows key. `map.insert(key, value)` takes ownership of key and value.

**Entry API**: Powerful API for conditional insertion. `map.entry(key)` returns `Entry` enum (Occupied or Vacant). `entry(key).or_insert(default)` inserts if absent and returns mutable reference. `entry(key).and_modify(|v| {...}).or_insert(default)` for update-or-insert patterns.

**Entry Examples**: `*map.entry(key).or_insert(0) += 1` for counting. `map.entry(key).or_insert_with(|| expensive_default())` for lazy defaults. `map.entry(key).or_default()` inserts `Default::default()` if absent.

**Iteration**: `for (k, v) in &map` borrows key-value pairs, `for (k, v) in &mut map` borrows mutably (can modify values, not keys), `for (k, v) in map` consumes map. HashMap iteration order arbitrary; BTreeMap sorted by key.

**Drain**: `map.drain()` returns iterator that removes all entries while iterating. Enables moving values out while clearing map. `map.drain().filter(|(k, v)| condition)` enables conditional removal.

**References as Keys**: Can use `HashMap<&str, V>` with string slice keys if entries outlive map. Requires lifetime management. Alternative: use owned `String` keys or `Cow<str>` for flexibility.

**Custom Hashers**: Specify hasher with `HashMap::with_hasher(hasher)` or use type alias: `HashMap<K, V, BuildHasherDefault<FxHasher>>`. FxHash faster for non-adversarial data. AHash balanced speed/security.

**Capacity**: `HashMap::with_capacity(n)` pre-allocates, `map.reserve(n)` reserves additional capacity, `map.shrink_to_fit()` reduces capacity to minimum. Prevents reallocations when size known.

**Clone Requirements**: Cloning HashMap requires `K: Clone` and `V: Clone`. Deep copies all entries. Expensive for large maps. Consider `Rc` or `Arc` for sharing.

**Borrow Checker Integration**: Can't modify map while holding references to entries. `let v = map.get(&key); map.insert(...);` won't compile if `v` still in use. Prevents iterator invalidation bugs at compile time.

**No Operator[] Overload**: Rust has no `map[key]` syntax that inserts on missing key. Must use `get` (returns Option) or entry API. Prevents surprising mutations.

**Derive Macros**: `#[derive(Hash)]` for structs hashes all fields. Order matters—different field order means different hash. Can implement `Hash` manually for custom behavior.

**Performance Tuning**: Profile hash distribution with custom code. Switch hasher if many collisions. Use `BTreeMap` if iteration order important or worst-case guarantees needed. Consider `FxHashMap` from `fxhash` crate for non-cryptographic hashing.

**Arc for Sharing**: `Arc<HashMap<K, V>>` enables sharing map across threads (HashMap not Sync/Send). `Arc<RwLock<HashMap<K, V>>>` for concurrent modification. `DashMap` crate provides concurrent HashMap.

**Serde Integration**: Popular `serde` crate enables serialization/deserialization. `#[derive(Serialize, Deserialize)]` on types containing HashMap enables JSON, MessagePack, etc.

## ☕ Java HashMap and TreeMap

---

Java provides `HashMap` (hash table) and `TreeMap` (red-black tree) with reference semantics and generic type parameters.

**Reference Semantics**: Map variables are references. Assignment `HashMap<K, V> b = a` creates new reference to same object. Modifications through either affect both. To copy, use `new HashMap<>(original)`.

**Generics and Type Erasure**: `HashMap<String, Integer>` uses generics for compile-time type safety. At runtime, type information erased—just `HashMap` with `Object` references. Can't use primitives directly—must use wrapper classes (autoboxing).

**HashMap Implementation**: Array of buckets, each containing linked list (or tree if many collisions). Java 8+ converts long lists (8+ entries) to balanced trees for O(log n) worst-case. Good hash functions keep lists short.

**TreeMap Implementation**: Red-black tree. Maintains sorted order by natural ordering (`Comparable`) or custom `Comparator`. All keys must be mutually comparable.

**Hash Code Contract**: `hashCode()` and `equals()` must be consistent: equal objects must have equal hash codes. Unequal hash codes guarantee unequal objects, but equal hash codes don't guarantee equality.

**Basic Operations**: `put(key, value)` inserts/updates returning previous value or null, `get(key)` returns value or null, `remove(key)` removes returning value or null, `containsKey(key)` returns boolean, `size()` returns count.

**Null Keys and Values**: HashMap allows one null key and multiple null values. TreeMap disallows null keys (can't compare null) but allows null values. Use of null discouraged—consider Optional or sentinel values.

**putIfAbsent**: Java 8+ `putIfAbsent(key, value)` only inserts if key absent, returns existing or new value. Atomic operation for concurrent maps. Similar to Rust's entry API but less flexible.

**compute Methods**: `compute(key, remappingFunction)` computes new value based on key and current value, `computeIfAbsent(key, mappingFunction)` computes if absent (lazy initialization), `computeIfPresent(key, remappingFunction)` computes if present.

**Merge**: `merge(key, value, remappingFunction)` inserts value if absent, otherwise applies function to current and new value. Useful for combining values: `map.merge(key, 1, Integer::sum)` for counting.

**Iteration**: `for (Map.Entry<K, V> entry : map.entrySet())` iterates entries, `for (K key : map.keySet())` iterates keys, `for (V value : map.values())` iterates values. HashMap order arbitrary; TreeMap sorted.

**Entry Set**: `map.entrySet()` returns `Set<Map.Entry<K, V>>`. Entries support `getKey()`, `getValue()`, `setValue(v)`. Modifying value through entry modifies map.

**Streams**: Java 8+ `map.entrySet().stream()` enables functional operations: `map.entrySet().stream().filter(e -> e.getValue() > 10).collect(Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue))`.

**Concurrent Modification**: Iterator throws `ConcurrentModificationException` if map structurally modified during iteration (except via iterator's remove). Single-threaded modification detection, not concurrency control.

**Synchronization**: HashMap not synchronized. Use `Collections.synchronizedMap(map)` wrapper or `ConcurrentHashMap` for thread-safe operations. `ConcurrentHashMap` provides better concurrency than synchronized wrapper.

**Load Factor**: Default load factor 0.75 (resize at 75% full). Initial capacity defaults to 16. Can specify both: `new HashMap<>(initialCapacity, loadFactor)`. Higher load factor saves space but increases collision probability.

**Capacity and Rehashing**: When threshold exceeded (capacity * load factor), doubles capacity and rehashes all entries. Expensive operation. Pre-allocate if size known: `new HashMap<>((int)(expectedSize / loadFactor) + 1)`.

**LinkedHashMap**: Subclass maintaining insertion order (or access order). Uses doubly-linked list through entries. Slightly more memory and overhead than HashMap but predictable iteration order.

**Identity HashMap**: Uses reference equality (`==`) instead of `equals()`. Keys are same only if identical object. Useful for object graphs and special scenarios.

**Weak HashMap**: Keys held with weak references—entries garbage collected when keys no longer strongly reachable. Useful for caches and memory-sensitive scenarios.

**Comparable vs Comparator**: TreeMap requires keys implement `Comparable` or provide `Comparator` to constructor. Comparison must be consistent with equals for correct behavior.

**Navigable Methods**: TreeMap implements `NavigableMap`: `firstKey()`, `lastKey()`, `lowerKey(key)`, `higherKey(key)`, `subMap(fromKey, toKey)` for range views. Powerful for ordered scenarios.

## 🌐 JavaScript Maps and Objects

---

JavaScript provides both `Map` objects (ES6+) and plain objects for key-value storage, each with different characteristics.

**Map vs Object**: `Map` is dedicated key-value structure supporting any key type, maintaining insertion order, with size property and clear iteration. Objects are property containers optimized for string keys, with prototype chain, no guaranteed order (though modern engines preserve insertion order in many cases).

**Map Creation**: `new Map()` creates empty map, `new Map([[key1, value1], [key2, value2]])` creates from array of pairs, `new Map(otherMap)` copies entries from another map or iterable.

**Key Types**: Maps support any value as key—objects, functions, primitives. Keys compared with SameValueZero (like `===` but NaN equals NaN). Objects as keys maintain identity—`map.set(obj, value)` uses object reference, not value.

**Basic Operations**: `map.set(key, value)` inserts/updates returning map (chainable), `map.get(key)` returns value or undefined, `map.has(key)` returns boolean, `map.delete(key)` removes returning boolean, `map.clear()` removes all entries, `map.size` returns count.

**Method Chaining**: `set()` returns map, enabling chaining: `map.set('a', 1).set('b', 2).set('c', 3)`. Convenient for initialization.

**Iteration**: `for (let [key, value] of map)` iterates entries in insertion order, `map.forEach((value, key, map) => {...})` callback iteration, `map.keys()` returns iterator of keys, `map.values()` returns iterator of values, `map.entries()` returns iterator of [key, value] pairs.

**Insertion Order**: Maps guarantee iteration in insertion order. Newer entries appear later. Order is reliable and predictable, unlike objects (though modern engines often preserve object insertion order).

**WeakMap**: Keys must be objects, held with weak references. Entries garbage collected when key no longer reachable. No iteration (unpredictable due to GC). Useful for private data and caches.

**WeakMap Use Cases**: Associate metadata with objects without preventing garbage collection. Example: `const privateData = new WeakMap(); class MyClass { constructor() { privateData.set(this, {...}); } }`.

**Performance**: Maps optimized for frequent addition/removal. Objects optimized for string keys and fixed structure. For dynamic key-value storage, Maps typically faster. For object-like structures, plain objects often better.

**Object as Map**: Traditional approach uses objects as maps: `const obj = {}; obj[key] = value; const value = obj[key];`. Works for string/symbol keys. Issues: prototype pollution, string coercion, no size property, no iteration without `Object.keys()`.

**Prototype Pollution**: `obj.toString` accesses inherited property. `obj.hasOwnProperty('toString')` checks own property. Maps don't have this issue—no prototype chain. For objects as maps, use `Object.create(null)` for no prototype.

**String Coercion**: Object keys coerced to strings: `obj[1] = 'a'` creates property `'1'`. Arrays, objects coerced: `obj[{a: 1}] = 'b'` creates property `'[object Object]'`. Maps preserve key types.

**Object Methods**: `Object.keys(obj)` returns array of own property names, `Object.values(obj)` returns values, `Object.entries(obj)` returns `[key, value]` pairs. Enable iteration over objects used as maps.

**Object.assign**: `Object.assign(target, ...sources)` merges objects. Shallow copy—nested objects referenced. `{...obj1, ...obj2}` spread syntax more concise for merging.

**Map Conversion**: Convert object to Map: `new Map(Object.entries(obj))`. Convert Map to object: `Object.fromEntries(map)` (ES2019+) or manual: `const obj = {}; map.forEach((v, k) => obj[k] = v);`.

**JSON Serialization**: Plain objects serialize to JSON directly. Maps don't—must convert: `JSON.stringify(Object.fromEntries(map))` to serialize, `new Map(Object.entries(JSON.parse(json)))` to deserialize.

**Set vs Map**: `Set` for unique values (values are keys). `Map` for key-value pairs. Both maintain insertion order, support any value type, have size property and similar methods.

**Common Patterns**: Use Map for dynamic collections with non-string keys, frequent add/remove, need for size/iteration. Use objects for fixed structure, string keys, JSON serialization, or when prototypal inheritance desired.

## 🔷 Go Maps

---

Go's maps are built-in hash tables with reference-like semantics and concurrent access restrictions.

**Map Declaration**: `var m map[KeyType]ValueType` declares nil map (unusable until initialized), `m := make(map[KeyType]ValueType)` creates empty map, `m := map[KeyType]ValueType{key1: value1, key2: value2}` creates with literals, `m := make(map[KeyType]ValueType, capacity)` pre-allocates capacity.

**Nil Maps**: Uninitialized map is nil. Reading from nil map returns zero value; writing panics. Check `if m == nil` before use. Safe to check length of nil map: `len(m) == 0`.

**Key Requirements**: Key type must be comparable with `==` operator. Most types work: integers, strings, pointers, structs/arrays with comparable fields. Slices, maps, functions not comparable—can't be keys.

**Basic Operations**: `m[key] = value` assigns, `value := m[key]` retrieves (zero value if absent), `delete(m, key)` removes entry, `len(m)` returns count. No built-in method to check existence—use comma ok idiom.

**Comma Ok Idiom**: `value, ok := m[key]` returns value and boolean indicating presence. If key absent, `value` is zero value and `ok` is false. Essential for distinguishing absent keys from zero values.

**Zero Values**: Accessing missing key returns zero value for value type: `0` for numbers, `""` for strings, `false` for bools, `nil` for pointers/slices/maps. Can't distinguish missing from actual zero without comma ok.

**Iteration**: `for key, value := range m` iterates entries in random order. Order intentionally randomized to prevent dependence. Can ignore key or value with `_`. Modification during iteration allowed (with caveats).

**Range Modification**: Can delete entries during range iteration. Can add entries but no guarantee new entries visited. Use caution—collect keys first if order matters.

**Reference Semantics**: Maps are references to underlying hash table. Assignment `b := a` makes both reference same map. Modifications through either affect both. Passing to function passes reference—function can modify caller's map.

**Copying**: No built-in copy. Must manually iterate and copy: `new := make(map[K]V); for k, v := range old { new[k] = v }`. Shallow copy—values themselves not copied (references/pointers still shared).

**Concurrency**: Maps not safe for concurrent access. Concurrent reads ok; concurrent writes or read-write causes race condition (may panic or corrupt data). Use `sync.RWMutex` or `sync.Map` for concurrent scenarios.

**sync.Map**: Specialized map for concurrent use. Methods: `Store(key, value)`, `Load(key)` returns `(value, ok)`, `LoadOrStore(key, value)` returns `(actual, loaded)`, `Delete(key)`, `Range(func(key, value))`. Optimized for specific patterns (stable keys, write-once read-many).

**Map Capacity**: Can specify capacity hint: `make(map[K]V, capacity)`. Reduces reallocation during population. Actual capacity may exceed hint. No method to query capacity.

**Memory**: Maps grow but never shrink automatically. Deleting entries doesn't reduce memory. Create new map and copy if shrinking needed. Consider map overhead for small value types.

**Struct as Value**: Common pattern: `map[string]*StructType` stores pointers to structs. Allows modification: `m["key"].field = value`. If storing structs by value, can't modify fields directly—must reassign entire struct.

**String Keys**: Strings are commonly used keys. Efficient—Go strings immutable and interned in many cases. Use string builders for construction: `var sb strings.Builder; sb.WriteString(...)`.

**Composite Keys**: No built-in composite keys. Solutions: concatenate strings with delimiter, use struct as key (if comparable), hash multiple values to single key. Struct keys type-safe: `type CompKey struct { a, b string }`.

**Default Values**: Use `make()` inside struct with map fields: `type MyStruct struct { data map[string]int }; func New() *MyStruct { return &MyStruct{data: make(map[string]int)} }`. Nil maps cause panics on write.

**Map vs Slice**: Maps for sparse data, non-integer keys, unordered data. Slices for dense sequences, integer indices, ordered data. Don't use maps where slices sufficient.

## 🏛️ Haskell Maps

---

Haskell provides `Data.Map` from containers package—immutable, ordered maps implemented as balanced trees with structural sharing.

**Implementation**: Size-balanced binary tree. Each node contains key, value, size, left subtree, right subtree. Balanced to maintain O(log n) operations. Immutable with structural sharing.

**Immutability**: All operations return new maps. Original unchanged. Due to sharing, efficient—only O(log n) nodes created for single modification. Enables persistent data structures—maintain all versions.

**Import**: `import qualified Data.Map as Map` or `import Data.Map (Map)` with qualified for functions. Avoids name clashes with Prelude.

**Creation**: `Map.empty` creates empty map, `Map.singleton key value` creates single-entry map, `Map.fromList [(k1, v1), (k2, v2)]` creates from association list, `Map.fromListWith combine [(k, v)]` handles duplicate keys with combining function.

**Key Requirements**: Keys must have `Ord` instance (must be orderable). Standard types have instances. Custom types need `deriving Ord` or manual implementation.

**Lookup**: `Map.lookup key map` returns `Maybe value` (`Just value` or `Nothing`), `Map.!? key map` same as lookup (operator form), `map Map.! key` partial function that errors if key absent, `Map.findWithDefault default key map` returns value or default.

**Insertion**: `Map.insert key value map` inserts or updates, `Map.insertWith combine key value map` combines with existing value if present, `Map.insertWithKey combine key value map` combine function also receives key.

**Deletion**: `Map.delete key map` removes entry (no-op if absent), `Map.alter (const Nothing) key map` removes (more general), `Map.update updateFn key map` applies function, deletes if returns Nothing.

**Modification**: `Map.adjust fn key map` applies function to value if present, `Map.update fn key map` like adjust but can delete (if fn returns Nothing), `Map.alter fn key map` most general—handles absent/present with Maybe.

**Queries**: `Map.member key map` returns Bool, `Map.notMember key map` opposite, `Map.size map` returns count, `Map.null map` checks if empty.

**Combination**: `Map.union map1 map2` left-biased union, `Map.unionWith combine map1 map2` combines values for duplicate keys, `Map.unions [map1, map2, ...]` unions list of maps, `Map.intersection map1 map2` keeps keys present in both.

**Folds and Traversals**: `Map.foldr fn acc map` right fold over values, `Map.foldl fn acc map` left fold, `Map.foldrWithKey fn acc map` fold receives keys too, `Map.map fn map` maps function over values, `Map.mapWithKey fn map` function receives keys.

**Filtering**: `Map.filter predicate map` keeps values matching predicate, `Map.filterWithKey predicate map` predicate receives key and value, `Map.partition predicate map` returns pair of maps (matching, non-matching).

**List Conversion**: `Map.toList map` converts to `[(key, value)]` in ascending key order, `Map.fromList [(k, v)]` creates from list (later entries override earlier for duplicate keys), `Map.elems map` returns list of values, `Map.keys map` returns list of keys.

**Ordered Iteration**: Iteration always in ascending key order. `Map.toAscList` same as `toList` (explicit), `Map.toDescList` returns descending order.

**Strictness**: Maps lazy by default—values not evaluated until accessed. Use `Data.Map.Strict` for strict maps (values evaluated on insertion). Strict maps prevent space leaks in many scenarios.

**Performance**: All operations O(log n). Constant factor small (balanced tree). Practical performance excellent up to millions of entries. More memory overhead than hash tables but provides ordering.

**Structural Sharing**: Sharing enables efficient versioning. Can maintain old versions of map while creating new. Garbage collector reclaims unreachable versions. Enables elegant functional algorithms.

**Functor, Foldable, Traversable**: Map is instance of these type classes. `fmap f map` maps over values, `traverse fn map` applicative traversal, `foldMap fn map` monoidal fold.

**IntMap**: Specialized `Data.IntMap` for Int keys. Uses radix tree—more efficient than Map for integer keys. Similar API. Use when keys are integers.

**HashMap**: `Data.HashMap` from unordered-containers package provides hash-based maps. O(1) average-case operations. No ordering. Use when don't need ordering and want better constant factors.

**Common Patterns**: Build map with `Map.fromListWith (+)` for counting, use `Map.insertWith (++)` for grouping, leverage immutability for undo/history, use Map as multiset with Int values.

## 📊 Comparison of Common Map Operations Complexity

---

| Operation | Python dict | C Hash Table* | C++ map | C++ unordered_map | Elixir Map | Zig HashMap | JS Map | Rust HashMap | Java HashMap | Go map | Haskell Map |
|-----------|-------------|---------------|---------|-------------------|------------|-------------|--------|--------------|--------------|--------|-------------|
| **Lookup** | O(1) avg | O(1) avg | O(log n) | O(1) avg | O(log n) | O(1) avg | O(1) avg | O(1) avg | O(1) avg | O(1) avg | O(log n) |
| **Insert** | O(1) avg | O(1) avg | O(log n) | O(1) avg | O(log n) | O(1) avg | O(1) avg | O(1) avg | O(1) avg | O(1) avg | O(log n) |
| **Delete** | O(1) avg | O(1) avg | O(log n) | O(1) avg | O(log n) | O(1) avg | O(1) avg | O(1) avg | O(1) avg | O(1) avg | O(log n) |
| **Iterate** | O(n) | O(n+m)** | O(n) | O(n+m)** | O(n) | O(n+m)** | O(n) | O(n+m)** | O(n+m)** | O(n) | O(n) |
| **Size** | O(1) | O(1) | O(1) | O(1) | O(1) | O(1) | O(1) | O(1) | O(1) | O(1) | O(1) |
| **Min/Max** | O(n) | O(n+m)** | O(log n) | O(n+m)** | O(n) | O(n+m)** | O(n) | O(n+m)** | O(n+m)** | O(n) | O(log n) |

*Manual implementation dependent, **m is table capacity

## 🔗 Related Concepts

---

- [[Hash Tables]]
- [[Hash Functions]]
- [[Dictionaries]]
- [[Associative Arrays]]
- [[Binary Search Trees]]
- [[Red-Black Trees]]
- [[B-Trees]]
- [[HAMT]] (Hash Array Mapped Trie)
- [[Data Structures]]
- [[Algorithms]]
- [[Big O Notation]]
- [[Load Factor]]
- [[Collision Resolution]]
- [[Open Addressing]]
- [[Chaining]]
- [[Memory Management]]
- [[Immutability]]
- [[Persistent Data Structures]]
- [[Caching]]
- [[Sets]]
- [[Indexing]]
- [[Key-Value Stores]]
- [[Distributed Hash Tables]]
- [[Bloom Filters]]

## 💡 Design Patterns and Best Practices

---

**Pre-sizing**: When final size known or estimable, pre-allocate to avoid rehashing. Python: `dict.fromkeys(keys)` or comprehension, C++: `unordered_map.reserve(n)`, Rust: `HashMap::with_capacity(n)`, Go: `make(map[K]V, capacity)`, Java: `new HashMap<>(initialCapacity)`.

**String Interning**: For maps with many repeated string keys, intern strings to share memory and speed equality checks. Python's strings auto-interned in many cases. Consider custom interning for extreme cases.

**Weak References**: Use weak-referenced maps (Python `weakref.WeakKeyDictionary`, Java `WeakHashMap`, JS `WeakMap`) for caches where entries can be garbage collected when keys no longer used.

**Multimap Pattern**: Languages without built-in multimaps (multiple values per key) can use `Map<K, List<V>>`. Initialize lists lazily with entry API or default dict pattern.

**Composite Keys**: For multi-field keys, use tuples (Python), structs (Rust/Zig/Go if comparable), or concatenated strings with delimiters. Ensure equality and hashing cover all fields.

**Default Values**: Use `defaultdict` (Python), entry API (Rust/Java), `computeIfAbsent` (Java), or manual checking to provide defaults for missing keys. Cleaner than explicit checks.

**Batch Operations**: For multiple insertions, use batch methods if available or pre-size and iterate. Reduces overhead compared to individual insertions with capacity checks.

**Immutable Keys**: Ensure keys immutable after insertion. Modifying key after insertion breaks map invariant—lookups fail. Defensive copy if necessary.

**Hash Quality**: For custom types, ensure good hash distribution. Combine field hashes with XOR and bit shifting. Avoid simple addition—correlates fields.

**Ordered Maps**: Use tree-based maps when need sorted iteration or range queries. Slight overhead but enables powerful operations (`lower_bound`, `upper_bound`, subranges).

**Memory-Sensitive**: For memory-constrained scenarios, consider alternatives: sorted arrays with binary search, minimal perfect hashing, or compressed structures.

## 🧮 Memory Overhead Analysis

---

**Python dict**: Object overhead (≈240 bytes for empty dict), entry array (24 bytes per entry: hash, key pointer, value pointer), sparse index array. 100-entry dict: ≈240 + 3,200 (entries) + indices ≈ 4KB minimum.

**C Hash Table**: Depends on implementation. Chaining: array of pointers (8 bytes each) plus linked list nodes (typically 24 bytes: key pointer, value pointer, next pointer). 100 entries, load factor 0.75: 1,067 buckets * 8 + 100 * 24 ≈ 10.9KB. Open addressing more compact.

**C++ unordered_map**: Hash table with chaining. Each entry: hash, key, value, next pointer, plus buckets array. Approximately (sizeof(K) + sizeof(V) + 24) per entry plus buckets. 100 int→int entries: ≈3.2KB.

**C++ map**: Red-black tree. Each node: key, value, three pointers (parent, left, right), color bit. Typically (sizeof(K) + sizeof(V) + 24-32) per entry. 100 int→int entries: ≈3.6KB.

**Elixir Map**: Small maps (≤32 entries) use array. Large maps use HAMT with 32-way branching. Each tree node: 32-element array (256 bytes) plus bitmap (4 bytes). Sharing reduces overhead. 100 entries: ≈6-8KB depending on distribution.

**Zig HashMap**: Metadata array (1 byte per slot), keys array, values array, plus struct overhead. 100 int→int entries at 80% load: 125 bytes (metadata) + 500 (keys) + 500 (values) ≈ 1.1KB. Compact.

**Rust HashMap**: Similar to C++ unordered_map. Hash, key, value, control bytes. Approximately (sizeof(K) + sizeof(V) + 16) per entry plus buckets overhead. 100 int→int entries: ≈2.4KB.

**Java HashMap**: Entry objects (object header, hash, key ref, value ref, next ref ≈ 40 bytes), buckets array. Autoboxing for primitives expensive. 100 Integer→Integer: ≈4KB for map + ≈3.2KB for Integer objects ≈ 7.2KB.

**Go map**: Buckets of 8 entries each. Each bucket: 8 keys, 8 values, overflow pointer, top-hash bytes. Compact layout. 100 int→int entries: ≈1.8KB.

**Haskell Map**: Tree nodes with key, value, size, two child pointers. Approximately (sizeof(K) + sizeof(V) + 32) per entry. 100 Int→Int entries: ≈4.4KB. Sharing amortizes cost across versions.

## 🎯 Use Case Guidelines

---

**Caching**: Hash maps excel. Fast lookup, insertion, deletion. Consider LRU eviction (ordered map or linked hash map), weak references for automatic cleanup, or expiration timestamps.

**Counting/Frequency**: Maps with numeric values. Use entry API or default dict pattern to avoid explicit zero-checks. Increment pattern common: `count[key] = count.get(key, 0) + 1`.

**Grouping**: Map keys to lists/sets of values. Multimap pattern. Example: group students by grade, words by first letter, events by date.

**Indexing**: Map unique IDs to records. Fast lookup by ID. Consider memory vs rebuild time tradeoff—may be cheaper to recompute than store large index.

**Graph Adjacency**: Map nodes to lists of neighbors. Efficient for sparse graphs. Consider adjacency matrix for dense graphs.

**Configuration**: Map setting names to values. String keys, mixed value types (if language supports). Consider nested maps for hierarchical config.

**Memoization**: Cache function results. Map arguments (as key) to result. Consider size limits, cache invalidation, and thread safety.

**Symbol Tables**: Compilers/interpreters map identifiers to information. Often nested scopes requiring chained maps or stack of maps.

**Routing/Dispatch**: Map URLs/paths to handlers, commands to functions, event types to listeners. Fast dispatch based on input.

## 📚 Further Reading

---

- "Introduction to Algorithms" by Cormen, Leiserson, Rivest, and Stein (CLRS)—comprehensive hash table coverage
- "The Art of Computer Programming" Volume 3 by Donald Knuth—authoritative treatment of searching and hashing
- "Purely Functional Data Structures" by Chris Okasaki—immutable maps and persistent structures
- "Hash Tables" chapter in "Programming Pearls" by Jon Bentley
- Language-specific documentation: Python dict implementation details, Rust HashMap docs, Go map internals
- "Ideal Hash Trees" paper by Phil Bagwell—HAMT implementation
- "Algorithmic Improvements for Fast Concurrent Caching" for concurrent map designs
- CPython source code for dict implementation details
- Rust standard library HashMap source
- Blog posts on Go map implementation internals
- Academic papers on perfect hashing, cuckoo hashing, and Robin Hood hashing
