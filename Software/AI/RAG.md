# RAG (Retrieval-Augmented Generation)

Retrieval-Augmented Generation (RAG) is an AI architecture pattern that enhances Large Language Models (LLMs) by connecting them to external knowledge sources. Instead of relying solely on static training data, RAG systems dynamically retrieve relevant information from authoritative databases, documents, or APIs at query time, then use that retrieved context to ground the LLM's response. This approach reduces hallucinations, enables access to current information, and allows models to incorporate domain-specific or proprietary knowledge without expensive retraining. RAG has become the dominant pattern for building trustworthy, enterprise-grade AI applications in 2024-2025.

## 🎯 Overview

RAG was first introduced in a 2020 research paper by Meta AI (then Facebook AI Research) as "a general-purpose fine-tuning recipe" for connecting LLMs to external knowledge sources. The core insight: LLMs excel at understanding language patterns but struggle with factual accuracy and currency since they're frozen at their training cutoff date. RAG solves this by treating LLM interactions like an "open-book exam" - the model reads relevant documents before answering, rather than relying purely on memorized knowledge. As LLM context windows grew (Claude 200K, Gemini 1M+), some predicted RAG's demise, but the opposite occurred. RAG evolved into sophisticated agentic systems with intelligent routing, hybrid retrieval, and conditional attention. Today, over 60% of organizations building AI applications use RAG patterns, making it foundational infrastructure for chatbots, knowledge assistants, customer support, and enterprise search.

---

## ⚙️ Core Architecture

### The RAG Pipeline

**Classic RAG follows three phases:**

1. **Indexing (Offline):**
   - **Chunking:** Split documents into smaller pieces (typically 256-1024 tokens)
   - **Embedding:** Convert chunks to vector representations using embedding models
   - **Storage:** Store vectors in vector database with metadata
   - **Indexing:** Build ANN (Approximate Nearest Neighbor) index for fast retrieval

2. **Retrieval (Query Time):**
   - **Query Embedding:** Convert user question to vector representation
   - **Similarity Search:** Find k most similar document chunks (typically k=3-10)
   - **Reranking:** Optional - reorder results for better relevance
   - **Filtering:** Apply metadata filters (date, department, permissions, etc.)

3. **Generation (Query Time):**
   - **Prompt Augmentation:** Combine user query + retrieved context
   - **LLM Call:** Generate response grounded in retrieved documents
   - **Citation:** Optionally include source references in response
   - **Validation:** Check response aligns with retrieved context

### Key Components

**Vector Database:**
- Stores embeddings and enables similarity search
- Popular options: Pinecone, Weaviate, Qdrant, Milvus, Chroma, pgvector
- Hybrid search combines vector + keyword (BM25) for better recall
- Metadata filtering enforces access control and constraints

**Embedding Model:**
- Converts text to dense vector representations
- Common models: OpenAI ada-002, Cohere embed-v3, Sentence-BERT
- Dimensionality: typically 384-1536 dimensions
- Critical to use same model for indexing and querying

**Chunking Strategy:**
- Fixed-size: 512 tokens with 50-token overlap (simple, fast)
- Semantic: Split on paragraphs, sentences, or topics (better quality)
- Recursive: Hierarchical splitting for structure preservation
- Trade-off: Smaller chunks = better precision, larger = better context

**Retriever:**
- Dense retrieval: Vector similarity (semantic understanding)
- Sparse retrieval: BM25, TF-IDF (exact keyword matching)
- Hybrid retrieval: Combines both approaches (best of both worlds)
- Reranking: Cross-encoder models refine initial results

---

## 🆚 Comparison Chart: RAG vs Alternatives

| Approach | Grounding | Real-time | Setup Complexity | Cost | Best For | Limitations |
|----------|-----------|-----------|------------------|------|----------|-------------|
| **Naive LLM** | ❌ Training data only | ❌ Frozen | Very Low | Low | General Q&A | Hallucinations, outdated |
| **Fine-tuning** | ⚠️ Custom training | ❌ Frozen | Very High | Very High | Domain adaptation | Requires retraining, expensive |
| **Classic RAG** | ✅ Retrieved docs | ✅ Live data | Medium | Medium | Knowledge Q&A | Retrieval failures, latency |
| **Agentic RAG** | ✅ Multi-step retrieval | ✅ Live + planning | High | Medium-High | Complex reasoning | Complexity, debugging |
| **MCP** | ✅ Tool calls | ✅ Live actions | Medium | Medium | Tool orchestration | Protocol adoption |
| **Long Context** | ⚠️ Everything loaded | ❌ One-time | Low | Very High | Full document analysis | Cost prohibitive |
| **Hybrid (RAG + MCP)** | ✅ Docs + tools | ✅ Live everything | High | Medium-High | Enterprise AI | Integration complexity |

---

## 🔄 RAG Evolution: Three Paradigms

### Naive RAG (2020-2023)

**Simple linear flow:**
- Query → Embed → Retrieve top-k → Augment prompt → Generate

**Limitations:**
- **Low precision:** Retrieves irrelevant chunks
- **Low recall:** Misses relevant information
- **No reasoning:** Cannot break down complex queries
- **Static retrieval:** Always retrieves k docs, even if not needed
- **Quality issues:** Redundancy, style mismatches, hallucination

### Advanced RAG (2023-2024)

**Improvements:**
- **Pre-retrieval:** Query rewriting, expansion, decomposition
- **Post-retrieval:** Reranking, context compression, filtering
- **Hybrid search:** Vector + BM25 for better recall
- **Metadata filtering:** Access control, time ranges, document types
- **Contextual embeddings:** Add chunk context during indexing
- **Iterative retrieval:** Multiple retrieval rounds for complex questions

**Key techniques:**
- HyDE: Generate hypothetical answer, use for retrieval
- Query decomposition: Break complex questions into sub-queries
- Reranking: Cross-encoder models refine initial results
- Context compression: Remove irrelevant parts of retrieved chunks

### Modular/Agentic RAG (2024-2025)

**Agent-driven workflow:**
- **Query analysis:** Understand intent, determine if retrieval needed
- **Planning:** Decompose into sub-tasks, decide retrieval strategy
- **Tool use:** Call retrieval, web search, calculators, APIs
- **Reasoning:** ReAct pattern (Reasoning + Acting)
- **Reflection:** Self-critique, verify retrieved context is sufficient
- **Memory:** Track conversation, build context over time

**Agentic components:**
- **Router:** Decides whether to retrieve, which knowledge base
- **Planner:** Creates multi-step execution plan
- **Tool library:** Retrieval, web search, code execution, APIs
- **Evaluator:** Checks quality, requests more retrieval if needed
- **Orchestrator:** Coordinates multiple agents/tools

---

## 🔨 RAG in Ash AI (Ash Framework)

### Overview

Ash AI is an LLM toolbox extension for Ash Framework (Elixir/Phoenix) that provides comprehensive RAG capabilities alongside prompt-backed actions, tool calling, and MCP server support. Released in 2024 and officially announced at ElixirConf EU 2025, Ash AI takes a declarative, data-driven approach to implementing RAG that leverages Ash Framework's core strengths: modeling application logic as data, automatic API generation, and built-in authorization policies.

### Core RAG Features

**Automatic Vectorization:**
- Declarative `vectorize` blocks in resource definitions
- Automatic embedding generation on create/update
- Multiple vectorization strategies (after_action, ash_oban, manual)
- Support for multiple text combinations per resource
- Built-in pgvector integration via AshPostgres

**Embedding Management:**
```elixir
# in MyApp.Shopping.Product
vectorize do
  full_text do
    text fn product ->
      """
      Name: #{product.name}
      Description: #{product.description}
      Category: #{product.category}
      """
    end
    
    # Only rebuild embeddings when these attributes change
    used_attributes [:name, :description, :category]
  end
  
  # Store individual attribute embeddings separately
  attributes(
    description: :vectorized_description,
    name: :vectorized_name
  )
  
  # Async updates via Oban for production
  strategy :ash_oban
  ash_oban_trigger_name :update_product_embeddings
  
  # Bring your own embedding model (OpenAI, Cohere, local, etc.)
  embedding_model MyApp.OpenAiEmbeddingModel
end
```

**Vector Similarity Queries:**
```elixir
# Use vector_cosine_distance in filters and sorts
Product
|> Ash.Query.sort(vector_cosine_distance(full_text_vector, query_embedding))
|> Ash.Query.limit(5)
|> Ash.read!()

# Filter by similarity threshold
Product
|> Ash.Query.filter(vector_cosine_distance(full_text_vector, query_embedding) < 0.3)
|> Ash.read!()
```

### Integration with Ash Ecosystem

**Authorization Policies:**
- Vector embeddings respect Ash policies automatically
- Bypass for internal embedding updates:
```elixir
bypass action(:ash_ai_update_embeddings) do
  authorize_if AshAi.Checks.ActorIsAshAi
end
```

**Generated Chat UI:**
- `mix ash_ai.gen.chat` generates complete chat interface
- Phoenix LiveView-based UI with streaming
- Built-in conversation persistence
- Tool calling integration

**MCP Server Support:**
- Production MCP server exposes Ash actions as tools
- Development MCP server provides app structure to IDEs
- Direct integration with Claude Desktop, Cursor, etc.
- Security layer enforces Ash policies on tool calls

### Vectorization Strategies

**1. After Action (Synchronous - Default):**
```elixir
strategy :after_action
```
- Updates embeddings immediately after create/update
- Simplest approach for development/low-volume apps
- Can slow down requests for large text or many embeddings
- **Not recommended for production**

**2. Ash Oban (Asynchronous - Recommended):**
```elixir
strategy :ash_oban
ash_oban_trigger_name :my_vectorize_trigger
```
- Background job processing via Oban
- Non-blocking updates
- Retries on failure
- Scalable for production workloads
- Requires AshOban setup

**3. Manual (On-Demand):**
```elixir
strategy :manual
define_update_action_for_manual_strategy? true
```
- No automatic updates
- Call `:ash_ai_update_embeddings` action manually
- Useful for batch processing or external triggers

### Embedding Model Interface

**Custom Embedding Models:**
```elixir
defmodule MyApp.OpenAiEmbeddingModel do
  @behaviour AshAi.Embedding.Model
  
  @impl true
  def dimensions, do: 3072  # text-embedding-3-large
  
  @impl true
  def generate_embeddings(texts) do
    # Call OpenAI API with Req or similar
    response = Req.post!(
      "https://api.openai.com/v1/embeddings",
      json: %{
        model: "text-embedding-3-large",
        input: texts
      },
      auth: {:bearer, System.fetch_env!("OPENAI_API_KEY")}
    )
    
    embeddings = Enum.map(response.body["data"], & &1["embedding"])
    {:ok, embeddings}
  end
end
```

**Supported Providers:**
- OpenAI (ada-002, text-embedding-3-small/large)
- Cohere (embed-v3.0)
- Local models (via Bumblebee/Nx)
- Any custom implementation via behavior

### PostgreSQL Setup

**Enable pgvector Extension:**
```elixir
# In migration
execute("CREATE EXTENSION IF NOT EXISTS vector")
```

**Define Custom Types:**
```elixir
# lib/my_app/postgrex_types.ex
Postgrex.Types.define(
  MyApp.PostgrexTypes,
  [AshPostgres.Extensions.Vector] ++ Ecto.Adapters.Postgres.extensions(),
  []
)
```

**Configure Repo:**
```elixir
# config/config.exs
config :my_app, MyApp.Repo,
  types: MyApp.PostgrexTypes
```

### Strengths of Ash AI RAG

**Declarative Approach:**
- Define vectorization alongside resource schema
- No separate indexing pipeline to maintain
- Automatic migration generation for vector columns
- Type-safe vector operations

**Integrated Authorization:**
- Respects existing Ash policies
- No separate access control for embeddings
- Secure by default (policies enforced at query level)
- Audit trail via Ash's built-in change tracking

**Elixir/BEAM Benefits:**
- Excellent concurrency for embedding generation
- Fault-tolerant (let it crash philosophy)
- Distributed system support
- Low-latency real-time updates via LiveView

**Framework Integration:**
- Works with AshGraphQL, AshJsonApi automatically
- Leverages Ash's calculated attributes system
- Compatible with multi-tenancy
- Integrates with Ash changesets and validations

**Developer Experience:**
- Single language (Elixir) for entire stack
- Mix tasks for scaffolding (`mix ash_ai.gen.chat`)
- Hot code reloading in development
- Comprehensive error messages

### Limitations

**Ecosystem Maturity:**
- Younger than Python RAG ecosystem (LangChain, LlamaIndex)
- Fewer pre-built integrations
- Smaller community (though growing rapidly)
- Less documentation/examples compared to Python tools

**Model Support:**
- Currently OpenAI-focused (each provider has different JSON schemas)
- Requires custom implementation for new providers
- LangSchema project aims to solve this

**Embedding Models:**
- Must implement custom behavior for each model
- No built-in local model support (requires Bumblebee setup)
- Limited reranking support out of the box

**Query Capabilities:**
- Basic vector similarity queries
- No built-in hybrid search (must implement manually)
- No advanced RAG patterns (yet) - HyDE, query decomposition, etc.
- Reranking requires custom implementation

---

## 🆚 RAG Framework Comparison: Ash AI vs Others

| Framework | Language | Vector DB | Embedding Support | Hybrid Search | Reranking | Auth/Security | Tool Calling | MCP Support | Best For |
|-----------|----------|-----------|-------------------|---------------|-----------|---------------|--------------|-------------|----------|
| **Ash AI** | Elixir | pgvector (built-in) | Custom models | Manual | Manual | ✅ Policy-based | ✅ Native | ✅ Native | Elixir/Phoenix apps, auth-heavy |
| **LangChain** | Python/JS | Agnostic | ✅ 50+ models | ✅ Built-in | ✅ Built-in | ⚠️ Manual | ✅ Native | ⚠️ Via adapters | General RAG, rapid prototyping |
| **LlamaIndex** | Python/TS | Agnostic | ✅ 40+ models | ✅ Built-in | ✅ Built-in | ⚠️ Manual | ✅ Native | ⚠️ Via adapters | Data-centric RAG, indices |
| **Haystack** | Python | Agnostic | ✅ 30+ models | ✅ Built-in | ✅ Built-in | ⚠️ Manual | ⚠️ Via nodes | ❌ No | Pipeline-heavy, search-first |
| **Semantic Kernel** | C#/Python | Agnostic | ✅ Multiple | ✅ Built-in | ⚠️ Limited | ⚠️ Manual | ✅ Native | ⚠️ Experimental | .NET/Microsoft stack |
| **Weaviate** | API (any) | Weaviate | ✅ Built-in | ✅ Native | ✅ Native | ✅ RBAC | ⚠️ Limited | ❌ No | Vector DB as service |
| **OpenAI Assistants** | API (any) | Built-in | Built-in | N/A | N/A | ✅ Per-user | ✅ Native | ❌ No | Simple RAG, OpenAI-only |
| **Azure AI Search** | API (any) | Azure | ✅ Multiple | ✅ Native | ✅ Native | ✅ RBAC | ⚠️ Via functions | ⚠️ Via Azure | Enterprise, Azure stack |
| **Vertex AI** | API (any) | Matching Engine | ✅ Multiple | ✅ Native | ✅ Native | ✅ IAM | ⚠️ Via functions | ❌ No | Enterprise, GCP stack |

### Key Differentiators

**Ash AI Unique Strengths:**
- **Declarative RAG:** Define vectorization in resource schema
- **Built-in Authorization:** Policy enforcement at framework level
- **Type Safety:** Compile-time checks for vector operations
- **BEAM Platform:** Fault tolerance, concurrency, distribution
- **Single Language:** Full-stack Elixir (backend, LiveView frontend, MCP)
- **Auto-Generation:** Mix tasks generate complete chat UIs
- **Native MCP:** First-class MCP server support

**When to Choose Ash AI:**
- Already using Elixir/Phoenix
- Complex authorization requirements (multi-tenant, RBAC, ABAC)
- Need real-time features (LiveView + vector search)
- Want declarative approach over imperative
- Building secure, production RAG with minimal code
- Value type safety and compile-time checks

**When to Choose Alternatives:**
- **LangChain/LlamaIndex:** Python ecosystem, rapid prototyping, many integrations
- **Haystack:** Heavy document processing, pipeline orchestration
- **Semantic Kernel:** Microsoft/.NET stack, enterprise Windows deployments
- **Cloud Services (Azure/Vertex):** Want managed service, not self-hosted
- **Weaviate:** Vector DB as primary interface, simple RAG

---

## 💼 Ash AI RAG Use Cases

### E-commerce Product Search
```elixir
# Semantic search across product catalog
defmodule Shop.Product do
  use Ash.Resource, extensions: [AshAi, AshPostgres]
  
  vectorize do
    full_text do
      text fn p -> 
        "#{p.name} #{p.description} #{p.category} #{p.tags}" 
      end
      used_attributes [:name, :description, :category, :tags]
    end
    strategy :ash_oban
    embedding_model Shop.EmbeddingModel
  end
  
  # Users can only see their organization's products
  policies do
    policy action_type(:read) do
      authorize_if relates_to_actor_via(:organization)
    end
  end
end
```

### Internal Documentation Search
```elixir
# Company wiki/docs with access control
defmodule Wiki.Document do
  vectorize do
    full_text do
      text fn doc ->
        """
        Title: #{doc.title}
        Content: #{doc.content}
        Department: #{doc.department}
        """
      end
    end
    strategy :ash_oban
  end
  
  # Department-level access control
  policies do
    policy action_type(:read) do
      authorize_if actor_attribute_equals(:department, :department)
    end
  end
end
```

### Customer Support Ticket Routing
```elixir
# Find similar resolved tickets
defmodule Support.Ticket do
  vectorize do
    full_text do
      text fn t ->
        "#{t.subject} #{t.description} #{t.resolution}"
      end
      used_attributes [:subject, :description, :resolution]
    end
    # Only vectorize resolved tickets
    filter expr(status == :resolved)
  end
  
  # Suggest similar tickets to support agents
  code_interface do
    define :find_similar, args: [:query_text, :limit]
  end
end
```

### Multi-tenant SaaS Knowledge Base
```elixir
# Isolated per-tenant vector search
defmodule SaaS.Article do
  multitenancy do
    strategy :attribute
    attribute :tenant_id
  end
  
  vectorize do
    full_text do
      text fn a -> "#{a.title} #{a.content}" end
    end
    strategy :ash_oban
  end
  
  # Automatic tenant isolation
  policies do
    policy action_type(:read) do
      authorize_if tenant_matches_actor()
    end
  end
end
```

---

## 🔧 Ash AI Best Practices

### Performance Optimization

**1. Use Ash Oban Strategy:**
```elixir
# Never block requests for embedding generation
strategy :ash_oban
ash_oban_trigger_name :update_embeddings
```

**2. Specify used_attributes:**
```elixir
# Only rebuild when relevant fields change
used_attributes [:name, :description, :summary]
# Don't rebuild on irrelevant changes (view_count, last_accessed, etc.)
```

**3. Filter Before Vectorizing:**
```elixir
# Don't vectorize drafts or archived content
filter expr(status == :published)
```

**4. Batch Embedding Generation:**
```elixir
# Process multiple records in single API call
# Custom embedding model can batch requests
def generate_embeddings(texts) when length(texts) > 1 do
  # Single API call for all texts
end
```

### Security Best Practices

**1. Always Use Policies:**
```elixir
# Never skip authorization
policies do
  policy action_type(:read) do
    # Define who can read
  end
end
```

**2. Bypass for Embedding Updates:**
```elixir
# Allow internal embedding updates
bypass action(:ash_ai_update_embeddings) do
  authorize_if AshAi.Checks.ActorIsAshAi
end
```

**3. MCP Tool Security:**
```elixir
# Expose only safe actions as MCP tools
tools do
  tool :search_products  # Safe read-only
  # Don't expose :delete_all_data!
end
```

**4. Validate Embedding Models:**
```elixir
# Don't trust external embeddings blindly
# Validate dimensions match expected
def generate_embeddings(texts) do
  {:ok, embeddings} = call_external_api(texts)
  
  if length(embeddings) != length(texts) do
    raise "Embedding count mismatch"
  end
  
  {:ok, embeddings}
end
```

### Development Workflow

**1. Start with Manual Strategy:**
```elixir
# Development: rebuild on demand
strategy :manual

# Production: switch to async
strategy :ash_oban
```

**2. Use Mix Tasks:**
```bash
# Generate chat UI
mix ash_ai.gen.chat

# Generate migrations
mix ash_postgres.generate_migrations

# Rebuild embeddings manually
mix ash_ai.rebuild_embeddings MyApp.Product
```

**3. Test Embedding Quality:**
```elixir
# Create test queries with known results
defmodule MyAppTest do
  test "finds semantically similar products" do
    create_product!(name: "red cotton t-shirt")
    create_product!(name: "blue denim jacket")
    
    results = search_products("scarlet shirt")
    
    assert hd(results).name =~ "red"
    refute hd(results).name =~ "blue"
  end
end
```

---

## 🔗 Ash AI Resources

### Official Documentation
- [Ash AI GitHub](https://github.com/ash-project/ash_ai)
- [Ash Framework Documentation](https://hexdocs.pm/ash/)
- [AshPostgres Vector Extension](https://hexdocs.pm/ash_postgres/AshPostgres.Extensions.Vector.html)
- [Ash AI Blog Post](https://alembic.com.au/blog/ash-ai-comprehensive-llm-toolbox-for-ash-framework)

### Tutorials & Examples
- [Vector Embeddings with Ash, OpenAI, and PostgreSQL](https://www.yellowduck.be/posts/vector-embeddings-with-ash-openai-and-postgresql)
- [ElixirConf EU 2025 Talk](https://www.youtube.com/watch?v=...) (Zach Daniel)
- [Ash Weekly Issue #17](https://json.media/blog/en/ash-weekly-17-recap)

### Community
- [Elixir Forum - Ash Questions](https://elixirforum.com/c/ash-framework/)
- [Ash Discord Server](https://discord.gg/ash-framework)

### Training
- [Ash: Supercharge Your Elixir Apps with AI](https://codebeameurope.com/trainings/ash-ai/) - CodeBEAM Course

---

## 🤖 RAG in Claude AI

### Claude Projects with RAG

**Automatic RAG activation (2024):**
- When project knowledge exceeds context window, RAG auto-enables
- 10x capacity increase (compared to in-context processing)
- Uses `project_knowledge_search` tool for retrieval
- Maintains response quality equivalent to full-context approach
- Visual indicator shows when RAG is active

**How it works:**
- Upload documents to Claude Project (PDFs, docs, code, images)
- Claude indexes content in background
- When context window limit approached, switches to RAG mode
- Searches uploaded knowledge base using semantic similarity
- Retrieves only relevant portions for each query
- Generates response grounded in retrieved context

**User guidance:**
- Add all relevant documents upfront (more context = better results)
- Use descriptive filenames (helps retrieval accuracy)
- Group related documents in same project (enables cross-referencing)
- Reference specific documents by name in queries

### Agentic Retrieval in Claude

**Beyond classic RAG:**
- Claude can decide when retrieval is needed (not always-on)
- Multi-step reasoning before/after retrieval
- Tool orchestration: retrieval + web search + code execution
- Reflection loops: verify sufficiency of retrieved context
- Adaptive querying: reformulate searches based on initial results

**Integration with MCP:**
- MCP servers can provide retrieval as a tool
- Claude reasons about which tools to use (retrieval vs API vs web)
- Hybrid workflows: RAG for knowledge + MCP for actions
- Example: Retrieve policy docs (RAG) + check account balance (MCP API)

---

## 🔌 MCP vs RAG: Complementary Technologies

### What is MCP?

**Model Context Protocol (Anthropic, Nov 2024):**
- Open standard for connecting AI to external data/tools
- "USB-C for AI" - universal, plug-and-play interface
- Client-server architecture: Host (Claude) → Client → Server (tool/data)
- Supports resources (read data), tools (take actions), prompts (templates)
- Real-time, dynamic access to live systems

### Key Differences

**RAG (Retrieval-Augmented Generation):**
- **Purpose:** Fetch knowledge from documents
- **When:** Query time, based on semantic similarity
- **Data type:** Unstructured text, pre-indexed documents
- **Workflow:** Embed → Search → Retrieve → Augment → Generate
- **Strengths:** Semantic search, document grounding, citations
- **Limitations:** Pre-processing required, retrieval failures, latency

**MCP (Model Context Protocol):**
- **Purpose:** Connect to tools, APIs, live data sources
- **When:** As needed during conversation, agent decides
- **Data type:** Structured APIs, databases, file systems, tools
- **Workflow:** LLM decides → Call tool → Get live data → Continue reasoning
- **Strengths:** Real-time data, actions/workflows, standardized protocol
- **Limitations:** Requires server implementation, network latency

### When to Use Each

**Use RAG when:**
- Need semantic search over documents
- Working with unstructured text (PDFs, wikis, manuals)
- Knowledge is relatively static (policies, procedures, history)
- Want citations/traceability to source documents
- Building knowledge Q&A, document search, research assistants

**Use MCP when:**
- Need real-time data (stock prices, inventory, account status)
- Taking actions (send email, create ticket, update database)
- Accessing structured data via APIs (CRM, ERP, databases)
- Triggering workflows (deployment, scheduling, notifications)
- Building task automation, operational tools, agents

**Use Both (Hybrid) when:**
- Need knowledge AND actions
- Example: Customer support - retrieve policy docs (RAG) + check order status (MCP)
- Example: Financial analyst - research reports (RAG) + live market data (MCP)
- Example: Developer assistant - code examples (RAG) + run tests (MCP)
- Most production enterprise AI uses hybrid approach

### How They Work Together

**Complementary architecture:**
1. User asks complex question requiring both knowledge and live data
2. Agent analyzes query, plans multi-step workflow
3. Calls RAG retriever to get relevant documentation/context
4. Calls MCP tool to fetch current data from API
5. Calls MCP tool to perform action (if needed)
6. Synthesizes all information into coherent response

**Example flow (HR assistant):**
```
User: "How much vacation do I have left and what's the policy on rolling over?"

Agent reasoning:
1. Need policy info → Use RAG to retrieve vacation policy docs
2. Need current balance → Use MCP to call HR API
3. Combine both → Generate response

Retrieved via RAG: "Company policy allows rollover of up to 5 days..."
Retrieved via MCP: {"employee_id": "123", "vacation_remaining": 12.5}

Response: "You have 12.5 vacation days remaining. According to company 
policy, you can roll over up to 5 days to next year. Would you like to 
schedule time off?"
```

**MCP doesn't replace RAG:**
- MCP adds higher-level orchestration framework
- RAG remains best approach for semantic document search
- MCP can use RAG as one of its tools
- Together they enable sophisticated agentic workflows

---

## 💪 Strengths of RAG

- **Reduces Hallucination:** Grounds responses in actual documents
- **No Retraining Needed:** Update knowledge by adding documents
- **Cost-Effective:** Cheaper than fine-tuning or long context windows
- **Access Control:** Enforce permissions via metadata filtering
- **Transparency:** Can provide source citations/footnotes
- **Current Information:** Update knowledge base without model changes
- **Domain Expertise:** Incorporate specialized knowledge
- **Privacy:** Keep sensitive data in controlled vector DB
- **Audit Trail:** Track what documents informed each response
- **Scalability:** Works with millions of documents

---

## ⚠️ Weaknesses and Challenges

- **Retrieval Failures:** May not find relevant chunks (low recall/precision)
- **Chunking Issues:** Optimal chunk size varies by use case
- **Context Window Limits:** Retrieved context competes with conversation history
- **Latency:** Extra retrieval step adds 100-500ms delay
- **Setup Complexity:** Requires vector DB, embeddings, indexing pipeline
- **Data Freshness:** Need mechanism to keep index updated
- **Quality Dependency:** Bad retrieval = bad generation
- **Ranking Errors:** Most relevant chunk may not be in top-k
- **Multi-hop Reasoning:** Struggles with questions requiring multiple documents
- **Cost:** Vector DB hosting, embedding API calls, reranking
- **Hallucination:** Can still occur if retrieved docs are ambiguous
- **Context Mismatch:** Retrieved text may lack necessary context

---

## 🎮 Use Cases and Applications

### Enterprise Knowledge Management

- **Internal search:** Employee handbook, policies, SOPs
- **IT support:** Troubleshooting guides, runbooks, incident history
- **Legal/Compliance:** Contract search, regulatory documents, case law
- **HR assistants:** Benefits, leave policies, onboarding materials
- **Finance:** Financial reports, accounting procedures, controls

### Customer-Facing Applications

- **Customer support:** Product manuals, FAQs, support tickets
- **Chatbots:** Website Q&A, product information, troubleshooting
- **E-commerce:** Product search, recommendations, specifications
- **Healthcare:** Medical knowledge bases, patient FAQs, drug information
- **Education:** Course materials, textbook Q&A, research assistants

### Research and Analysis

- **Scientific research:** Paper search, literature review, citation analysis
- **Market research:** Industry reports, competitor analysis, trends
- **Due diligence:** Document analysis, risk assessment, compliance checks
- **Intelligence:** News monitoring, threat analysis, open-source intelligence
- **Academic:** Thesis research, citation networks, knowledge graphs

### Development and Technical

- **Code search:** Find similar code, documentation, API examples
- **Documentation:** Auto-generate docs from codebases
- **Debugging:** Search issue trackers, Stack Overflow, logs
- **DevOps:** Infrastructure docs, incident post-mortems, playbooks

### Content and Media

- **News:** Article recommendations, fact-checking, source verification
- **Publishing:** Content search, plagiarism detection, research tools
- **Archives:** Historical document search, metadata enrichment

---

## 🔧 Implementation Best Practices

### Chunking Strategies

**Fixed-size chunks:**
- 512 tokens with 50-token overlap
- Simple, predictable, works for most cases
- Use when: Consistent document structure

**Semantic chunks:**
- Split on paragraphs, sections, or topics
- Better context preservation
- Use when: Narrative documents, varied structure

**Recursive chunking:**
- Split large chunks into smaller sub-chunks
- Maintains hierarchy (document → section → paragraph)
- Use when: Complex documents with structure

**Contextual embedding (Anthropic, 2024):**
- Add surrounding context to each chunk before embedding
- "This chunk is from document X, section Y: [chunk text]"
- Reduces retrieval failures by 35-50%

### Hybrid Retrieval

**Combine vector + keyword search:**
- Vector (dense): Semantic similarity, handles synonyms
- Keyword (sparse/BM25): Exact matches, technical terms
- Reciprocal Rank Fusion (RRF) to merge results
- 10-20% better recall than either alone

**When to use:**
- Technical documentation (exact terms matter)
- Legal/medical (precise terminology required)
- Enterprise search (users expect keyword behavior)

### Reranking

**Two-stage retrieval:**
1. Fast retrieval: Get top 100 candidates (BM25 + vector)
2. Slow reranking: Cross-encoder model scores top 100 → top 10

**Benefits:**
- 20-40% improvement in relevance
- Handles nuanced semantic matching
- Popular models: Cohere rerank, bge-reranker, cross-encoder

**Cost trade-off:**
- Adds latency (50-200ms)
- Use lite rerankers for speed-critical applications

### Metadata Filtering

**Pre-filter before similarity search:**
- Time ranges: `date >= '2024-01-01'`
- Permissions: `accessible_by = [user_id, user_groups]`
- Document type: `type IN ['policy', 'manual']`
- Department: `department = 'engineering'`

**Benefits:**
- Enforces access control
- Improves relevance (search within subset)
- Reduces latency (smaller search space)

### Evaluation and Monitoring

**Retrieval metrics:**
- **Recall@k:** % of relevant docs in top k
- **Precision@k:** % of top k that are relevant
- **MRR (Mean Reciprocal Rank):** Position of first relevant result
- **NDCG:** Normalized Discounted Cumulative Gain

**Generation metrics:**
- **Faithfulness:** Response grounded in retrieved context
- **Relevance:** Response answers user question
- **Groundedness:** No hallucination beyond retrieved docs
- **Citation accuracy:** Cited sources are correct

**Build evaluation sets:**
- Start with 10-50 golden question-answer pairs
- Expand to 100s-1000s over time
- Include edge cases, multi-hop questions, ambiguous queries
- Continuous evaluation pipeline for production

---

## 📚 Related Concepts/Notes

- [[Embeddings]]
- [[Vector Databases]]
- [[LLMs]] (Large Language Models)
- [[MCP]] (Model Context Protocol)
- [[Semantic Search]]
- [[Agentic AI]]
- [[Transformers]]
- [[BERT]]
- [[Prompt Engineering]]
- [[Fine-tuning]]
- [[Context Window]]
- [[Hallucination]]
- [[BM25]]
- [[HNSW]]
- [[Reranking]]
- [[Chunking Strategies]]
- [[Hybrid Search]]
- [[Knowledge Graphs]]
- [[Enterprise AI]]
- [[Claude AI]]

---

## 🔗 External Resources

### Foundational Papers

- [Retrieval-Augmented Generation for Knowledge-Intensive NLP Tasks](https://arxiv.org/abs/2005.11401) - Lewis et al., Meta AI, 2020 (original RAG paper)
- [Retrieval-Augmented Generation for Large Language Models: A Survey](https://arxiv.org/abs/2312.10997) - Gao et al., 2023

### Tutorials and Guides

- [AWS: What is RAG?](https://aws.amazon.com/what-is/retrieval-augmented-generation/)
- [IBM: What is Retrieval-Augmented Generation?](https://research.ibm.com/blog/retrieval-augmented-generation-RAG)
- [NVIDIA: What is Retrieval-Augmented Generation (RAG)?](https://blogs.nvidia.com/blog/what-is-retrieval-augmented-generation/)
- [Databricks: What is RAG?](https://www.databricks.com/glossary/retrieval-augmented-generation-rag)
- [Prompt Engineering Guide: RAG for LLMs](https://www.promptingguide.ai/research/rag)

### Anthropic/Claude Specific

- [Claude Projects with RAG](https://support.claude.com/en/articles/11473015-retrieval-augmented-generation-rag-for-projects)
- [Contextual Retrieval](https://www.anthropic.com/news/contextual-retrieval) - Anthropic's RAG improvements, 2024
- [Introducing Model Context Protocol](https://www.anthropic.com/news/model-context-protocol) - Nov 2024

### Frameworks and Tools

- [LlamaIndex](https://www.llamaindex.ai/) - RAG framework for Python
- [LangChain](https://www.langchain.com/) - LLM orchestration including RAG
- [LangGraph](https://langchain-ai.github.io/langgraph/) - Agentic workflows
- [Haystack](https://haystack.deepset.ai/) - NLP framework with RAG support

### Vector Databases

- [Pinecone Learning Center](https://www.pinecone.io/learn/)
- [Weaviate Documentation](https://weaviate.io/developers/weaviate)
- [Milvus/Zilliz Cloud](https://milvus.io/)
- [Qdrant](https://qdrant.tech/)
- [Chroma](https://www.trychroma.com/)

### Advanced Topics

- [Graph RAG](https://www.microsoft.com/en-us/research/blog/graphrag-new-tool-for-complex-data-discovery-now-on-github/) - Microsoft Research
- [Agentic RAG](https://zilliz.com/blog/agentic-rag-using-claude-3.5-sonnet-llamaindex-and-milvus)
- [RAG is Dead, Long Live RAG](https://www.lighton.ai/lighton-blogs/rag-is-dead-long-live-rag-retrieval-in-the-age-of-agents) - LightOn, 2025

---

## 🔮 Future Directions

### Emerging Trends (2025+)

**Graph RAG:**
- Knowledge graph-aware retrieval
- Traverse entity relationships, not just text similarity
- Better multi-hop reasoning, complex question answering
- Microsoft GraphRAG, Neo4j vector search

**Multimodal RAG:**
- Retrieve across text, images, tables, charts
- Vision-language models for image understanding
- CLIP for cross-modal retrieval (text → image, image → text)
- Use cases: Technical docs with diagrams, medical imaging + reports

**Agentic RAG:**
- Conditional retrieval (only when needed)
- Multi-step reasoning with tool orchestration
- Self-reflection and query refinement
- Planning and memory in multi-turn conversations
- ReAct, ReWOO, Self-RAG patterns

**Real-time RAG:**
- Streaming updates to knowledge base
- Incremental indexing without full reprocessing
- Change data capture (CDC) pipelines
- Use cases: News, financial markets, IoT/telemetry

**Federated RAG:**
- Retrieve across multiple siloed knowledge bases
- Privacy-preserving retrieval (no centralized store)
- Use cases: Healthcare (patient privacy), multi-tenant SaaS

**Adaptive RAG:**
- Learn from user feedback (relevance signals)
- Personalized retrieval based on user context
- Active learning to improve retrieval over time

### Open Challenges

**Retrieval quality:**
- Handling ambiguous or under-specified queries
- Multi-hop reasoning requiring multiple documents
- Temporal reasoning (what was true when?)
- Conflicting information across sources

**Scalability:**
- Real-time updates at scale (millions of documents)
- Low-latency retrieval under high load
- Cost optimization (embeddings, vector DB, LLM calls)

**Evaluation:**
- Standardized benchmarks for RAG quality
- End-to-end evaluation (retrieval + generation)
- Detecting subtle hallucinations or biases
- Measuring real-world business impact

**Integration:**
- Combining RAG with fine-tuning
- RAG + MCP + agent workflows
- Orchestration complexity in production
- Debugging and observability tools

---

## 💡 Key Takeaways

**RAG is essential infrastructure for trustworthy AI:**
- Reduces hallucination by grounding in actual documents
- Enables current, domain-specific knowledge without retraining
- Provides transparency through source citations
- Powers 60%+ of enterprise AI applications

**RAG hasn't been replaced - it's evolved:**
- From naive linear pipelines to sophisticated agentic systems
- Large context windows complement RAG, don't replace it
- MCP adds tool orchestration, RAG provides knowledge grounding
- Hybrid approaches (RAG + MCP + agents) are the future

**Success requires thoughtful engineering:**
- Chunking, hybrid retrieval, reranking are critical
- Evaluation and monitoring prevent quality degradation
- Metadata filtering enforces access control
- Iterative improvement based on real-world usage

**The future is agentic RAG:**
- Conditional retrieval (only when needed)
- Multi-step reasoning with planning and reflection
- Integration with tools, APIs, and live data sources
- Personalization and continuous learning
