# Lists

Lists are fundamental ordered data structures that store sequences of elements. They are one of the most common and versatile data structures in programming, used for everything from simple collections to complex algorithms. Understanding how different languages implement lists—their memory models, mutability, performance characteristics, and operations—is critical for writing efficient code.

## 🎯 Overview

---

A list is an ordered collection of elements that can typically be accessed by their position (index). Lists may go by different names in different languages: arrays, vectors, slices, or linked lists. The concept remains similar—storing multiple values in a sequence—but the implementation details vary dramatically across languages, affecting performance, memory usage, and available operations.

Lists can be implemented as contiguous arrays in memory (array-based lists) or as chains of nodes with pointers (linked lists). Some languages provide both, while others choose one approach. The choice impacts everything from access speed to insertion cost to memory overhead.

## 🧠 Core Concepts

---

**Indexing**: Accessing elements by position, typically zero-indexed. Random access is O(1) for array-based lists but O(n) for linked lists.

**Mutability**: Whether list contents can be changed after creation. Immutable lists prevent modification, while mutable lists allow in-place changes.

**Resizing**: How lists grow when capacity is exceeded. Dynamic arrays typically double in size, amortizing the cost of reallocation.

**Memory Layout**: Contiguous memory provides cache locality and fast access. Linked structures sacrifice access speed for efficient insertion/deletion.

**Reference vs Value Semantics**: Whether assignment copies the list data or just the reference. This fundamentally affects how lists behave when passed to functions.

**Homogeneous vs Heterogeneous**: Whether all elements must be the same type (statically typed languages) or can be mixed (dynamic languages).

**Growth Strategy**: How capacity increases—linear growth (add constant amount) vs exponential growth (multiply by factor). Exponential is generally preferred for amortized O(1) append.

**Slicing**: Creating sublists from ranges of elements, which may copy data or create views.

## 📊 Comparison Chart

---

| Language | Name | Mutable | Contiguous | Dynamic Size | Pass By | Copy on Assign | Append Cost | Random Access |
|----------|------|---------|------------|--------------|---------|----------------|-------------|---------------|
| **Python** | list | Yes | Yes | Yes | Reference | No (ref copy) | Amortized O(1) | O(1) |
| **C** | array | Yes | Yes | No | Value* | Yes | N/A (fixed) | O(1) |
| **C++** | vector | Yes | Yes | Yes | Value | Yes (deep) | Amortized O(1) | O(1) |
| **Elixir** | List | No | No (linked) | Yes | Value | Yes (structural sharing) | O(1) prepend | O(n) |
| **Zig** | ArrayList | Yes | Yes | Yes | Value | Yes | Amortized O(1) | O(1) |
| **JavaScript** | Array | Yes | Yes** | Yes | Reference | No (ref copy) | Amortized O(1) | O(1) |
| **Rust** | Vec | Yes | Yes | Yes | Move/Borrow | Move (ownership) | Amortized O(1) | O(1) |
| **Java** | ArrayList | Yes | Yes | Yes | Reference | No (ref copy) | Amortized O(1) | O(1) |
| **Go** | slice | Yes | Yes | Yes | Reference*** | Shallow copy | Amortized O(1) | O(1) |
| **Haskell** | List | No | No (linked) | Yes | Value | Yes (lazy) | O(1) prepend | O(n) |

*Arrays decay to pointers, **Implementation detail, ***Slices are reference-like but arrays are values

## 🐍 Python Lists

---

Python's `list` is a dynamic array implementation with reference semantics that provides excellent flexibility and ease of use.

**Implementation Details**: Python lists are implemented as dynamic arrays of pointers to PyObject structures. The list itself stores pointers, not the actual objects, which means lists can contain heterogeneous types. When capacity is exceeded, Python allocates a new array with approximately 1.125x growth factor plus constant.

**Memory Model**: Assignment creates a new reference, not a copy. Multiple variables can reference the same list. Modifications through any reference affect all references. The list object itself contains a pointer to the array of pointers, a size, and an allocated capacity.

**Mutability**: Lists are fully mutable. You can modify elements in place with `lst[i] = value`, append with `lst.append(x)`, extend with `lst.extend(iterable)`, insert with `lst.insert(i, x)`, remove with `lst.remove(x)` or `del lst[i]`, and pop with `lst.pop(i)`.

**Performance Characteristics**: Indexing is O(1), appending is amortized O(1), insertion at arbitrary position is O(n) due to shifting, deletion is O(n), searching is O(n), and slicing creates a new list in O(k) where k is slice size.

**Slicing**: Python slices create shallow copies. `lst[1:4]` creates a new list with references to the same objects. Modifications to the new list don't affect the original, but modifications to mutable objects within affect both. Slice assignment `lst[1:3] = [a, b, c]` modifies in place.

**List Comprehensions**: Python provides `[expr for item in iterable if condition]` which is both readable and often faster than explicit loops due to optimized C implementation.

**Memory Sharing**: Python lists never share underlying array storage between list objects. Even slices get new arrays. However, the elements themselves are shared references.

**Common Pitfall**: `lst = [[]]*3` creates three references to the same list, not three separate lists. Use `[[] for _ in range(3)]` instead.

**Internal Structure**: CPython's list structure contains `ob_refcnt` (reference count), `ob_type` (type pointer), `ob_size` (current length), `allocated` (capacity), and `ob_item` (pointer to array of pointers).

**Growth Pattern**: Python uses `new_allocated = (size_t)newsize + (newsize >> 3) + (newsize < 9 ? 3 : 6)` which gives approximately 1.125x growth with small constant additions.

## 🔧 C Arrays

---

C provides fixed-size arrays as the fundamental sequential data structure, with no built-in dynamic resizing.

**Implementation Details**: C arrays are contiguous blocks of memory containing elements of the same type. The array name is essentially a pointer to the first element. Size must be known at compile time for stack arrays, or allocated dynamically on heap.

**Memory Model**: Arrays are values, but they decay to pointers when passed to functions. `int arr[10]` allocates 40 bytes (assuming 4-byte ints) on the stack. Assignment doesn't copy—you must use `memcpy` or loop explicitly. Passing to functions passes a pointer, allowing modification of original.

**Declaration Types**: Stack arrays with `int arr[10]`, heap arrays with `int *arr = malloc(10 * sizeof(int))`, variable-length arrays (C99+) with `int arr[n]` where n is runtime value, and multidimensional with `int matrix[3][4]`.

**No Bounds Checking**: C performs no runtime bounds checking. Accessing `arr[100]` on a 10-element array is undefined behavior that may crash, corrupt memory, or silently produce garbage.

**Manual Memory Management**: Heap-allocated arrays require explicit `free(arr)` to prevent memory leaks. Failure to free causes memory leaks; freeing twice causes undefined behavior.

**Dynamic Arrays**: Must be manually implemented. Typical approach: maintain pointer, size, and capacity. When size reaches capacity, allocate new larger array (typically 2x), copy data with `memcpy`, free old array, update pointer. Example pattern:
```
typedef struct {
    int *data;
    size_t size;
    size_t capacity;
} DynamicArray;
```

**Array Decay**: In most contexts, `int arr[10]` decays to `int *`. This means `sizeof(arr)` gives array size in current scope but `sizeof` on parameter gives pointer size. Use separate size parameter for functions.

**Pointer Arithmetic**: `arr[i]` is syntactic sugar for `*(arr + i)`. Understanding this is crucial for advanced C programming. You can iterate with `for(int *p = arr; p < arr + size; p++)`.

**String Arrays**: Strings are character arrays terminated by null byte `\0`. Declaration: `char str[] = "hello"` allocates 6 bytes (including null terminator). Must ensure sufficient space for operations.

**Multidimensional Arrays**: `int matrix[3][4]` allocates contiguous 12 integers. Row-major order means `matrix[i][j]` is at `*(matrix + i*4 + j)`. When passing to functions, all dimensions except first must be specified.

**Common Patterns**: Flexible array members (C99) allow struct with array at end: `struct { int len; int data[]; }` allocated with `malloc(sizeof(struct) + n*sizeof(int))`.

## ⚡ C++ Vectors and Lists

---

C++ provides `std::vector` for dynamic arrays and `std::list` for doubly-linked lists, along with `std::array` for fixed-size arrays.

**std::vector Implementation**: Template class managing dynamic array. Stores pointer to array, size, and capacity. When capacity exceeded, allocates new array (typically 1.5x or 2x growth), move-constructs elements, destroys old elements, deallocates old memory.

**Memory Model**: Vectors own their data with value semantics. Assignment performs deep copy unless moved. `vector<int> b = a` creates independent copy. `vector<int> b = std::move(a)` transfers ownership, leaving `a` empty.

**Move Semantics**: C++11 move semantics avoid unnecessary copies. Moving a vector is O(1)—just transfers pointers. Enables efficient return by value and passing to functions. Move is automatic for temporaries.

**Capacity vs Size**: `size()` returns element count, `capacity()` returns allocated space. `reserve(n)` pre-allocates capacity without changing size. `shrink_to_fit()` requests capacity reduction to size (non-binding).

**Element Access**: `operator[]` provides unchecked access, `at()` provides bounds-checked access throwing `std::out_of_range`. `front()` and `back()` access first and last elements. `data()` provides raw pointer to underlying array.

**Modifiers**: `push_back(x)` appends (amortized O(1)), `emplace_back(args...)` constructs in place, `insert(it, x)` inserts at iterator position (O(n)), `erase(it)` removes element (O(n)), `clear()` removes all elements.

**Iterator Invalidation**: Iterators/pointers/references invalidated when vector reallocates (capacity changes) or when elements shift (insertion/deletion before position). `reserve()` before operations prevents reallocation.

**std::list Implementation**: Doubly-linked list where each node contains value, pointer to next node, and pointer to previous node. Nodes allocated individually on heap. No random access—must traverse from beginning or end.

**std::list Operations**: `push_front(x)` and `push_back(x)` are both O(1). `insert(it, x)` is O(1) given iterator. Splicing `splice(it, other_list)` moves elements between lists in O(1) by pointer manipulation. No reallocation, so iterators remain valid except to erased elements.

**When to Use Which**: Use `std::vector` for default sequential container—fast access, cache-friendly, low overhead. Use `std::list` when frequent insertion/deletion in middle and don't need random access. Use `std::deque` for efficient operations at both ends.

**std::array**: Fixed-size array wrapper with container interface. `std::array<int, 10> arr;` is stack-allocated, knows its size, provides bounds-checked `at()`, but can't resize. Safer than C arrays with no overhead.

**Allocators**: Vectors support custom allocators for specialized memory management. `std::vector<int, MyAllocator<int>>` uses custom allocation strategy. Useful for memory pools, aligned allocation, or tracking.

**Vector of Bools**: `std::vector<bool>` is specialized to store bits, not bytes. Controversial because it violates normal vector semantics—can't get `bool&` from `operator[]`. Consider `std::vector<char>` or `std::deque<bool>` if problems arise.

**Initialization**: C++11 provides `std::vector<int> v = {1, 2, 3, 4, 5}`. `std::vector<int> v(10)` creates 10 default-initialized elements. `std::vector<int> v(10, 42)` creates 10 elements all set to 42.

**Algorithms**: C++ standard algorithms work with vectors via iterators: `std::sort(v.begin(), v.end())`, `std::find(v.begin(), v.end(), value)`, `std::accumulate(v.begin(), v.end(), 0)`. Separation of algorithms and containers is key design principle.

## 💧 Elixir Lists

---

Elixir lists are immutable, singly-linked lists with structural sharing, fundamentally different from array-based lists in imperative languages.

**Implementation Details**: Each list is either empty `[]` or a cons cell `[head | tail]` where head is any value and tail is another list. This is inherited from Erlang's BEAM VM. Stored as linked nodes in heap memory.

**Immutability**: Lists can never be modified. Operations that appear to modify actually create new lists. `list = [1, 2, 3]` followed by `new_list = [0 | list]` creates new cons cell pointing to existing list—no copying of `[1, 2, 3]`.

**Structural Sharing**: Key to performance with immutability. `[0 | existing_list]` creates single new cons cell in O(1). The tail is shared reference to existing structure. `[1, 2, 3]` and `[0, 1, 2, 3]` share the `[1, 2, 3]` portion in memory.

**Prepending vs Appending**: Prepending `[x | list]` is O(1) because it creates single cons cell. Appending `list ++ [x]` is O(n) because entire list must be reconstructed with new cons cells (original tail can't be shared since it doesn't end with `[x]`).

**Pattern Matching**: Elixir excels at list pattern matching. `[head | tail] = [1, 2, 3]` binds `head = 1` and `tail = [2, 3]`. `[a, b | rest] = list` extracts first two elements. `[first, second, third] = list` matches exactly three elements.

**Recursion**: Idiomatic list processing uses recursion. Example summing:
```
def sum([]), do: 0
def sum([head | tail]), do: head + sum(tail)
```

This is how you "iterate" in functional languages without mutation.

**Performance Characteristics**: Prepend O(1), append O(n), length O(n) (must traverse), access by index O(n), concatenation O(n) of first list (second list shared).

**Length Calculation**: `length(list)` must traverse entire list. If you need length frequently, consider passing it alongside list or using different structure. Pattern: `{list, length}` tuple.

**Enum Module**: Provides higher-order functions like `Enum.map(list, fn x -> x * 2 end)`, `Enum.filter(list, fn x -> x > 5 end)`, `Enum.reduce(list, 0, fn x, acc -> x + acc end)`. These are recursive under the hood but optimized by VM.

**List Comprehensions**: `for x <- list, x > 5, do: x * 2` creates new list. More readable than nested `Enum` operations. Can iterate multiple lists with multiple generators.

**Improper Lists**: Normally lists end with `[]`, but can end with non-list: `[1 | 2]`. These are improper lists, rarely used but appear in some protocols. Most functions expect proper lists.

**Memory Layout**: Each cons cell is two words (typically 16 bytes on 64-bit)—one for head, one for tail pointer. Empty list `[]` is a special atom. This overhead makes lists memory-intensive for small elements compared to arrays.

**Tail Call Optimization**: BEAM VM optimizes tail-recursive list functions to constant stack space. Write recursive functions in tail-recursive form when possible: `defp helper([], acc), do: acc` and `defp helper([h|t], acc), do: helper(t, acc + h)`.

**When Not to Use**: Don't use lists when you need frequent random access, frequent appends, or O(1) length. Consider tuples (fixed-size), maps (key-value), or Erlang's `:array` module (rare) for these cases.

**Lists vs Keyword Lists**: Keyword lists are lists of tuples: `[name: "John", age: 30]` is syntactic sugar for `[{:name, "John"}, {:age, 30}]`. Used for function options. Allow duplicate keys, maintain order.

## ⚡ Zig ArrayList

---

Zig provides `std.ArrayList` as a dynamic array with manual memory management and value semantics.

**Implementation**: Generic structure containing allocator, items slice (pointer and length), and capacity. Items stored contiguously in heap memory obtained from allocator.

**Memory Management**: Must explicitly specify allocator: `var list = std.ArrayList(i32).init(allocator)`. Must call `list.deinit()` to free memory. Zig's compile-time checks and ownership patterns help prevent leaks, but responsibility is programmer's.

**Allocator Pattern**: Zig uses explicit allocators passed to data structures. `std.heap.page_allocator`, `std.heap.GeneralPurposeAllocator`, `std.testing.allocator`, or custom allocators. This makes allocation strategy visible and controllable.

**Generic Type**: `ArrayList(T)` where T is any type. Use like `ArrayList(u8)` for bytes, `ArrayList(Person)` for structs, `ArrayList(*Node)` for pointers. Compile-time generics mean no runtime overhead.

**Basic Operations**: `try list.append(value)` appends (returns error on allocation failure), `list.appendAssumeCapacity(value)` appends without checking capacity (faster but unsafe), `list.items[i]` accesses element, `list.pop()` removes and returns last element, `list.insert(index, value)` inserts at position.

**Error Handling**: Allocation can fail. `append` and other growing operations return `error{OutOfMemory}!void`. Must handle with `try`, `catch`, or explicit error handling. Zig forces acknowledgment of allocation failure.

**Capacity Management**: `try list.ensureTotalCapacity(n)` pre-allocates, `try list.resize(n)` changes size (may allocate), `list.clearRetainingCapacity()` sets length to 0 without deallocating, `list.clearAndFree()` deallocates memory.

**Slicing**: `list.items` is a slice (`[]T`) providing view of current elements. Can slice further: `list.items[1..4]`. Slices are fat pointers (pointer + length) passed by value. Slices don't own memory, so no deallocation needed.

**Memory Representation**: ArrayList struct contains allocator interface, `items: []T` (slice), and `capacity: usize`. The slice's pointer points to heap-allocated array. Capacity tracks allocated size.

**Growth Strategy**: Default growth is approximately 1.5x. Implementation: `new_capacity = old_capacity + old_capacity / 2 + 8`. Addition of constant helps with small sizes.

**Ownership**: ArrayList owns its memory and is responsible for freeing. Assignment copies the struct (shallow copy), meaning both refer to same memory—usually wrong. Use `list.clone()` for deep copy or transfer ownership explicitly.

**Comptime Features**: Can create ArrayList with comptime-known elements: `const list = std.ArrayList(i32).fromSlice(&[_]i32{1, 2, 3})`. Zig's comptime evaluation can optimize many operations.

**Integration with Slices**: Zig's ecosystem works with slices. Can convert ArrayList to slice with `.items`, pass to functions expecting slices. Functions taking slices are more flexible than functions taking ArrayList.

**No Iterator Invalidation Tracking**: Zig doesn't track iterator validity. If you hold pointer into ArrayList's memory and ArrayList reallocates, pointer becomes dangling. Programmer must ensure safety.

**Common Patterns**: Often see `defer list.deinit()` immediately after initialization to ensure cleanup. Zig's defer runs at scope exit. Arena allocators useful for collections that live for specific scope.

**Comparison to C++**: Similar to `std::vector` but explicit allocator, mandatory error handling, no RAII (manual deinit), value semantics without automatic deep copy, and no move semantics (just struct copy).

## 🦀 Rust Vec

---

Rust's `Vec<T>` is a growable array with ownership semantics and compile-time safety guarantees.

**Ownership Model**: Vectors own their data. Only one binding can own a vector at a time. Assignment moves ownership: `let b = a;` makes `a` invalid. Prevents double-free and use-after-free at compile time.

**Borrowing**: Can borrow immutably `&vec` (many readers) or mutably `&mut vec` (one writer, no readers). Compiler ensures no data races. Functions take `&Vec<T>` or `&[T]` (slice) to avoid taking ownership.

**Memory Layout**: Triple-word struct: pointer to heap allocation, length, and capacity. Elements stored contiguously on heap. Zero-sized types (like `()`) optimized to no heap allocation.

**Creation**: `Vec::new()` creates empty vector, `vec![1, 2, 3]` macro creates with initial elements, `Vec::with_capacity(n)` pre-allocates capacity, `vec.reserve(n)` reserves additional capacity.

**Element Access**: `vec[i]` panics on out-of-bounds, `vec.get(i)` returns `Option<&T>` safely, `vec.first()` and `vec.last()` return `Option<&T>`, unsafe `vec.get_unchecked(i)` skips bounds check.

**Modification**: `vec.push(x)` appends, `vec.pop()` returns `Option<T>`, `vec.insert(i, x)` inserts at index, `vec.remove(i)` removes and returns element, `vec.clear()` removes all elements keeping capacity.

**Iterators**: `vec.iter()` yields `&T`, `vec.iter_mut()` yields `&mut T`, `vec.into_iter()` consumes vector yielding `T`. Rich iterator adapter methods like `map`, `filter`, `fold`. Iterator invalidation prevented by borrow checker.

**Slicing**: `&vec[1..4]` creates slice reference. Slices are views, don't own data. Functions typically take `&[T]` rather than `&Vec<T>` for flexibility—works with arrays, vectors, and other slices.

**Growth Strategy**: Typically doubles capacity when full. Exact strategy is implementation detail but provides amortized O(1) push. Can inspect with `vec.capacity()`.

**Move Semantics**: Moving vector transfers ownership without copying data. Moving is default for non-Copy types. `Vec<i32>` moves cheaply (24 bytes on 64-bit). Clone explicitly with `.clone()` for deep copy.

**Drop**: When vector goes out of scope, Rust runs `Drop` trait implementation. This deallocates heap memory and runs destructors for all elements. Automatic, deterministic cleanup—no garbage collector needed.

**String Relation**: `String` is essentially `Vec<u8>` with UTF-8 validity guarantee. Many APIs similar. Understanding `Vec` helps understand `String`.

**Unsafe Operations**: `vec.set_len(n)` unsafely sets length without initialization (must ensure validity), `vec.as_mut_ptr()` gets raw pointer for FFI. Unsafe code requires careful reasoning.

**Deref Coercion**: `Vec<T>` implements `Deref<Target = [T]>`, so vector automatically coerces to slice in many contexts. Can call slice methods on vectors.

**Comparison to C++**: Similar to `std::vector` but ownership prevents common bugs, no iterator invalidation bugs (caught at compile time), no move constructor syntax (moves automatic), and mandatory initialization.

**SmallVec**: Popular crate providing `SmallVec` which stores small number of elements inline, avoiding heap allocation for small vectors. Useful for performance-critical code.

## ☕ Java ArrayList

---

Java's `ArrayList<E>` is a resizable array implementation with reference semantics and generic type parameters.

**Implementation**: Internally uses `Object[]` array to store elements. Size tracks element count, array capacity can exceed size. Generic type parameter provides compile-time type safety with runtime type erasure.

**Reference Semantics**: ArrayList variables are references. Assignment `ArrayList<Integer> b = a` creates new reference to same object. Modifications through either reference affect same list. To copy, use `new ArrayList<>(original)` or `original.clone()`.

**Generics and Type Erasure**: `ArrayList<String>` uses generic types for compile-time checking, but at runtime it's just `ArrayList` holding `Object` references. Can't create `ArrayList<int>` (must use `ArrayList<Integer>` with autoboxing).

**Autoboxing**: Primitive types automatically box to wrapper objects. `list.add(5)` boxes `int` to `Integer`. `int x = list.get(0)` unboxes. Convenient but has performance cost—boxing allocates objects.

**Capacity Management**: Initial capacity is 10 by default. Can specify with `ArrayList(initialCapacity)`. Grows by approximately 1.5x: `newCapacity = oldCapacity + (oldCapacity >> 1)`. Use `ensureCapacity(n)` to pre-allocate, `trimToSize()` to reduce capacity to size.

**Common Operations**: `add(e)` appends in amortized O(1), `add(index, e)` inserts at position in O(n), `get(index)` accesses in O(1), `set(index, e)` replaces in O(1), `remove(index)` removes in O(n), `size()` returns count in O(1).

**Enhanced For Loop**: `for (String s : list)` iterates over elements. Syntactic sugar for iterator. Can't modify list structure during iteration or `ConcurrentModificationException` thrown.

**Iterators**: `Iterator<E> it = list.iterator()` provides `hasNext()` and `next()`. `ListIterator<E>` additionally provides `hasPrevious()`, `previous()`, `add()`, `set()`. Fail-fast—throw exception if list modified during iteration.

**Concurrent Modification**: Iterator tracks modification count. If list structurally modified (add/remove) while iterating (except through iterator's own methods), `ConcurrentModificationException` thrown. Not a concurrent access bug—single-threaded modification detection.

**Synchronization**: ArrayList is not synchronized. Concurrent access from multiple threads requires external synchronization or `Collections.synchronizedList(list)` wrapper. Consider `CopyOnWriteArrayList` for concurrent scenarios with more reads than writes.

**Null Elements**: Can contain null elements. `list.add(null)` is valid. Methods like `contains(null)` and `remove(null)` work correctly.

**Arrays.asList()**: `List<String> list = Arrays.asList("a", "b", "c")` creates fixed-size list backed by array. Cannot add or remove elements, but can modify existing elements. Often used for initialization.

**Conversion**: `list.toArray()` returns `Object[]`, `list.toArray(new String[0])` returns `String[]`. ArrayList to array is O(n) copy. Array to ArrayList with `new ArrayList<>(Arrays.asList(array))`.

**Performance Considerations**: Good for random access and iteration. Poor for insertions/deletions in middle. Consider `LinkedList` for frequent insertions/deletions but note `LinkedList` has poor cache locality.

**Streams**: Java 8+ provides `list.stream()` for functional-style operations: `list.stream().filter(x -> x > 5).map(x -> x * 2).collect(Collectors.toList())`. Creates new list with results.

## 🔷 Go Slices

---

Go's slices are a powerful abstraction over arrays with reference-like semantics and built-in support for growth.

**Slice Structure**: A slice is a descriptor containing pointer to array, length, and capacity. Slices are values, but they contain a pointer, so they have reference-like behavior for the underlying array.

**Declaration**: `var s []int` declares nil slice, `s := []int{1, 2, 3}` creates slice with literal, `s := make([]int, length)` creates slice with specified length (zero-initialized), `s := make([]int, length, capacity)` specifies both length and capacity.

**Underlying Arrays**: Slices reference arrays. Multiple slices can reference the same underlying array. Modifying element through one slice affects all slices sharing that array segment.

**Slicing Operation**: `arr[low:high]` creates slice from index low (inclusive) to high (exclusive). `arr[:]` slices entire array, `arr[1:]` from index 1 to end, `arr[:3]` from start to index 3. Creates new slice descriptor, shares underlying array.

**Length vs Capacity**: `len(s)` returns number of elements, `cap(s)` returns capacity from slice start to end of underlying array. `s[0:cap(s)]` can extend slice up to capacity without allocation.

**Append Function**: `s = append(s, element)` adds element. If capacity sufficient, modifies underlying array and extends length. If capacity exceeded, allocates new larger array (typically 2x), copies elements, appends new element. Must assign result back to variable.

**Append Multiple**: `s = append(s, elem1, elem2, elem3)` appends multiple elements. `s = append(s, otherSlice...)` appends all elements from another slice (unpacking with `...`).

**Copy Function**: `copy(dst, src)` copies elements from src to dst. Returns number of elements copied (minimum of lengths). Slices can overlap—copy handles correctly.

**Nil Slices**: Uninitialized slice is nil: `var s []int` has `s == nil`, `len(s) == 0`, `cap(s) == 0`. Nil slices are valid to use with `append`, `len`, `cap`. Different from non-nil empty slice from `make([]int, 0)`.

**Reference Behavior**: Passing slice to function passes descriptor (pointer, len, cap). Function can modify elements (shared array), but appends that reallocate don't affect caller's slice. This can be confusing—must return and reassign if function may grow slice.

**Growth Strategy**: When append exceeds capacity, Go allocates new array. Growth factor varies: doubles for small slices, approaches 1.25x for larger slices. Exact strategy is implementation detail.

**Memory Sharing**: Slicing doesn't copy data. If you slice first 10 elements of million-element array, remaining elements can't be garbage collected (slice holds reference to array). To avoid, copy to new slice: `new := append([]int{}, original[:10]...)`.

**Three-Index Slice**: `s[low:high:max]` creates slice with specified capacity: length is `high-low`, capacity is `max-low`. Limits capacity to prevent unintended sharing beyond intended range.

**Array vs Slice**: Arrays are values with compile-time fixed size: `[3]int{1, 2, 3}`. Slices are dynamic, reference underlying array. Arrays rarely used directly—slices are idiomatic.

**Strings and Slices**: Strings are similar to `[]byte` but immutable. Can convert: `[]byte(str)` copies string to byte slice, `string(bytes)` creates string from bytes. Slicing strings yields substring sharing underlying bytes.

**Range Loop**: `for i, v := range slice` iterates with index and value. Index or value can be ignored with `_`. Range copies values, so modifying `v` doesn't affect slice (use index to modify: `slice[i] = newValue`).

**Common Patterns**: Pre-allocate with known size using `make([]int, 0, capacity)` to avoid reallocations. Filter pattern: create new slice with capacity, append elements matching condition. Delete element: `s = append(s[:i], s[i+1:]...)`.

## 🎭 JavaScript Arrays

---

JavaScript arrays are dynamic, heterogeneous, and have significant implementation flexibility hidden behind a simple interface.

**Dynamic Nature**: Arrays automatically resize. `arr.push(x)` always succeeds (until memory exhaustion). Can assign to any index: `arr[1000] = "hello"` creates sparse array with gaps.

**Heterogeneous**: Can mix types freely. `[1, "hello", {name: "John"}, [1, 2, 3]]` is valid. No compile-time type checking (unless using TypeScript).

**Reference Semantics**: Arrays are objects. Assignment creates new reference. `let b = a` makes both variables reference same array. Modifications through either affect both. To copy, use `[...arr]`, `arr.slice()`, or `Array.from(arr)`.

**Sparse Arrays**: Arrays can have gaps. `arr = [1, , , 4]` creates array with length 4 but indices 1 and 2 are "holes". `arr.length = 100` sets length without populating elements. Holes behave differently from `undefined` values in some methods.

**Length Property**: `arr.length` is highest index plus 1, not count of elements. Setting `arr.length` truncates or extends array. Length is writable, unlike other languages.

**Array Methods**: `push(x)` appends, `pop()` removes and returns last element, `shift()` removes first element, `unshift(x)` prepends, `splice(start, deleteCount, ...items)` modifies array in place, `slice(start, end)` returns shallow copy of portion.

**Higher-Order Functions**: `map(fn)` creates new array with function applied to each element, `filter(fn)` creates array with elements passing test, `reduce(fn, initial)` accumulates single value, `forEach(fn)` executes function for each element (returns undefined).

**Iteration**: `for (let i = 0; i < arr.length; i++)` traditional loop, `for (let item of arr)` for-of loop over values, `for (let index in arr)` for-in loop over indices (not recommended—includes inherited properties), `arr.forEach((item, index) => {...})` functional iteration.

**Array-like Objects**: Objects with numeric indices and length property but not arrays. Examples: `arguments`, DOM NodeLists. Convert with `Array.from(arrayLike)` or `[...arrayLike]`.

**Typed Arrays**: `Uint8Array`, `Int32Array`, `Float64Array`, etc. for binary data. Fixed-size, typed, better performance. Used for WebGL, Canvas, binary protocols. Backed by `ArrayBuffer`.

**Implementation Details**: Modern engines use multiple internal representations. Dense arrays for contiguous elements (fast), sparse representations for arrays with many holes, hash-map-like structure for very sparse arrays. Engine optimizes based on usage patterns.

**Performance Characteristics**: Access is generally O(1) but can degrade for sparse arrays. Push/pop are typically O(1). Shift/unshift are O(n). Splice is O(n). Actual performance depends on engine optimizations.

**Array.isArray()**: Reliable way to check if value is array. `typeof arr === "object"` for arrays, so use `Array.isArray(arr)`.

**Destructuring**: `let [a, b, c] = arr` extracts elements into variables. `let [first, ...rest] = arr` extracts first element and rest into new array. Useful for swapping: `[a, b] = [b, a]`.

**Spread Operator**: `[...arr1, ...arr2]` concatenates arrays. `[...arr]` creates shallow copy. `Math.max(...arr)` unpacks array as arguments.

**Finding Elements**: `indexOf(item)` returns first index or -1, `includes(item)` returns boolean, `find(fn)` returns first element matching predicate or undefined, `findIndex(fn)` returns index or -1.

**Sorting**: `arr.sort()` sorts in place, converting elements to strings by default. `arr.sort((a, b) => a - b)` for numeric sort. Sort mutates original array. Use `arr.slice().sort()` to avoid mutation.

**Chaining**: Methods like `map`, `filter`, `slice` return new arrays, enabling chaining: `arr.filter(x => x > 0).map(x => x * 2).slice(0, 10)`.

## 🏛️ Haskell Lists

---

Haskell's lists are immutable, lazy, singly-linked structures that are central to the language's functional programming paradigm.

**Construction**: Lists use cons operator `(:)` and empty list `[]`. `1 : 2 : 3 : []` constructs `[1, 2, 3]`. Syntactic sugar allows `[1, 2, 3]` notation. Lists are homogeneous—all elements must have same type.

**Lazy Evaluation**: Lists are evaluated on demand. Can create infinite lists like `[1..]` or `[1, 3..]` (infinite arithmetic sequences). Elements computed only when needed. Enables elegant algorithms that appear to work with infinite data.

**Pattern Matching**: Fundamental to Haskell list processing. `(x:xs)` pattern matches non-empty list, binding head to `x` and tail to `xs`. `[]` matches empty list. Example:
```
length [] = 0
length (_:xs) = 1 + length xs
```

**Immutability**: Lists never modified. All operations create new lists. Due to sharing, this is efficient—prepending `x : xs` creates new cons cell pointing to existing tail.

**Cons vs Snoc**: Prepending with `(:)` is O(1). Appending with `(++)` is O(n) of first list (must reconstruct it with new tail). Idiomatically build lists by prepending, then reverse if needed.

**List Comprehensions**: Powerful syntax for generating lists. `[x*2 | x <- [1..10], x `mod` 3 == 0]` creates list of doubled multiples of 3 from 1 to 10. Can have multiple generators and guards.

**Higher-Order Functions**: `map f list` applies function to each element, `filter p list` keeps elements matching predicate, `foldr f z list` right fold, `foldl f z list` left fold. These are building blocks of list processing.

**List Type**: `[a]` is syntactic sugar for linked list of type `a`. Actually defined as `data [a] = [] | a : [a]`—either empty or element consed onto another list.

**Strictness**: Haskell lists are lazy by default. `[1, 2, undefined, 4]` can be partially processed—accessing third element causes error, but first two accessible. Laziness enables working with infinite structures.

**Performance**: O(n) length, O(n) indexing, O(1) prepend, O(n) append, O(n) access by index. Not suitable for random access. For indexing-heavy workloads, use `Data.Vector` or `Data.Array`.

**Infinite Lists**: `ones = 1 : ones` defines infinite list of 1s. `fibs = 0 : 1 : zipWith (+) fibs (tail fibs)` defines infinite Fibonacci sequence. Functions like `take 10 fibs` extract finite portions.

**List Fusion**: GHC's optimizer can fuse multiple list operations into single pass via rewrite rules. `map f (map g xs)` can optimize to single traversal. Makes high-level functional code efficient.

**Monadic Operations**: Lists are monads. `>>= ` (bind) for lists is concatMap—enables representing non-deterministic computations. List monad used for elegant solutions to certain problems.

**Alternatives**: `Data.Vector` for efficient indexing and cache-friendly layout, `Data.Sequence` for efficient operations at both ends, `Data.ByteString` for binary data, `Data.Text` for text processing.

**Strings**: `String` is type alias for `[Char]`—list of characters. Simple but inefficient for text processing. Production code typically uses `Data.Text` or `Data.ByteString`.

## 📊 Comparison of List Operations Complexity

---

| Operation | Python list | C array | C++ vector | Elixir List | Zig ArrayList | Rust Vec | Java ArrayList | Go slice | Haskell List |
|-----------|-------------|---------|------------|-------------|---------------|----------|----------------|----------|--------------|
| **Append** | O(1)* | N/A | O(1)* | O(n) | O(1)* | O(1)* | O(1)* | O(1)* | O(n) |
| **Prepend** | O(n) | N/A | O(n) | O(1) | O(n) | O(n) | O(n) | O(n) | O(1) |
| **Access** | O(1) | O(1) | O(1) | O(n) | O(1) | O(1) | O(1) | O(1) | O(n) |
| **Insert** | O(n) | O(n) | O(n) | O(n) | O(n) | O(n) | O(n) | O(n) | O(n) |
| **Delete** | O(n) | O(n) | O(n) | O(n) | O(n) | O(n) | O(n) | O(n) | O(n) |
| **Search** | O(n) | O(n) | O(n) | O(n) | O(n) | O(n) | O(n) | O(n) | O(n) |
| **Length** | O(1) | O(1)** | O(1) | O(n) | O(1) | O(1) | O(1) | O(1) | O(n) |
| **Concat** | O(k) | O(n+m) | O(m) | O(n) | O(m) | O(m) | O(m) | O(m) | O(n) |

*Amortized, **With sizeof if in scope

## 🔗 Related Concepts

---

- [[Arrays]]
- [[Linked Lists]]
- [[Dynamic Arrays]]
- [[Data Structures]]
- [[Algorithms]]
- [[Memory Management]]
- [[Pointers]]
- [[References]]
- [[Iterators]]
- [[Slices]]
- [[Vectors]]
- [[Collections]]
- [[Garbage Collection]]
- [[Stack vs Heap]]
- [[Big O Notation]]
- [[Amortized Analysis]]
- [[Cache Locality]]
- [[Functional Programming]]
- [[Immutability]]
- [[Type Systems]]
- [[Generic Programming]]
- [[Memory Layout]]
- [[Copy Semantics]]
- [[Move Semantics]]
- [[Ownership]]

## 💡 Design Patterns and Best Practices

---

**Pre-allocation**: When final size is known or estimable, pre-allocate capacity to avoid reallocations. Python: `list * n` or list comprehension with known size, C++: `vector.reserve(n)`, Rust: `Vec::with_capacity(n)`, Go: `make([]T, 0, capacity)`.

**Builder Pattern**: Accumulate elements in mutable list during construction, then convert to immutable form or return. Common in languages with both mutable and immutable collections.

**Pooling**: Reuse list objects to reduce allocation overhead in tight loops. Clear and repopulate rather than creating new lists. More relevant in languages with garbage collection overhead.

**Chunking**: Process large lists in chunks to improve cache locality and enable parallelism. Split into fixed-size sublists and process independently.

**Lazy Evaluation**: Generate elements on demand rather than materializing entire list. Python generators, Haskell's lazy lists, Rust iterators. Saves memory and enables working with infinite sequences.

**Functional vs Imperative**: Functional languages favor building new lists; imperative languages favor in-place modification. Choose approach matching language idioms and performance characteristics.

**Avoiding Copies**: Pass by reference/pointer when possible. Use slices or views instead of copying portions. In languages with move semantics, transfer ownership when appropriate.

**Immutable Updates**: In functional languages, express "modifications" as building new lists with shared structure. Append to accumulator parameter in tail-recursive functions.

**Type Safety**: Use generic collections with appropriate type parameters. Avoid raw types or Object-based collections in typed languages—loses compile-time safety.

## 🧮 Memory Overhead Analysis

---

**Python list**: Object overhead (16-24 bytes) plus pointer array. Each element is pointer (8 bytes on 64-bit). List of 100 integers: 16 + 800 = 816 bytes minimum, plus integer objects themselves (28 bytes each) totaling ~3,616 bytes.

**C array**: Exact element size times count. `int arr[100]` is exactly 400 bytes (assuming 4-byte ints). No overhead. Most memory-efficient for simple types.

**C++ vector**: Three pointers (24 bytes) plus elements array. `vector<int>(100)` uses 24 + 400 = 424 bytes. Minimal overhead for non-trivial types.

**Elixir list**: Two words per cons cell (16 bytes on 64-bit) plus values. List of 100 integers: 100 * 16 + value storage. Immutable structures enable sharing, potentially saving memory across multiple lists.

**Zig ArrayList**: Allocator interface, slice, capacity. Approximately 32 bytes overhead plus elements. Efficient for value types.

**Rust Vec**: Three words (24 bytes) plus elements. Similar to C++ vector. Highly efficient with no hidden costs.

**Java ArrayList**: Object header (~16 bytes), size field (4 bytes), array reference (8 bytes), plus Object array. 100 Integer objects: 28 + 400 (array) + 1600 (Integer objects) = ~2,028 bytes. Autoboxing expensive.

**Go slice**: Three words (24 bytes on 64-bit): pointer, length, capacity. Plus underlying array. Slice of 100 ints: 24 + 400 = 424 bytes. Multiple slices can share array, amortizing overhead.

## 📚 Further Reading

---

- "Introduction to Algorithms" by Cormen, Leiserson, Rivest, and Stein (CLRS)
- "The Art of Computer Programming" Volume 1 by Donald Knuth
- "Programming Pearls" by Jon Bentley
- "Purely Functional Data Structures" by Chris Okasaki
- Language-specific documentation: Python tutorial, Rust book, Go specification
- "Data Structures and Algorithm Analysis" by Mark Allen Weiss
- "Functional Programming in Scala" by Chiusano and Bjarnason
- CPython source code for implementation details
- GCC libstdc++ source code for C++ containers
- Papers on persistent data structures and structural sharing
