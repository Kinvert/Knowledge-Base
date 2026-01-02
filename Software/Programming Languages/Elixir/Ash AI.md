# Ash AI

Ash AI is an extension for the Ash Framework that provides tools for integrating Large Language Models (LLMs) into Elixir applications. It enables structured LLM interactions, multi-agent conversations, tool calling, embeddings, and semantic search while leveraging Ash's declarative resource model. Ash AI transforms LLM integration from imperative API calls into declarative, composable, and testable resources.

## 🎯 Overview

---

Ash AI extends Ash Framework to treat AI interactions as first-class resources with all the benefits of Ash's architecture: validations, authorization, calculations, relationships, and pub/sub. Rather than scattering LLM API calls throughout your codebase, you define AI agents, conversations, and tools as Ash resources. This enables sophisticated multi-agent systems, deterministic testing, conversation persistence, and seamless integration with your existing Ash resources.

The framework supports multiple LLM providers (OpenAI, Anthropic, local models), function calling/tool use, embeddings for semantic search, streaming responses, conversation history management, and agent orchestration patterns. It's designed for production applications requiring reliability, observability, and maintainability in AI-powered features.

## 🧠 Core Concepts

---

**AI Resources**: Ash resources representing agents, conversations, messages, and tool definitions. Define behavior declaratively rather than imperatively managing API calls.

**Agents**: Configured LLM instances with specific system prompts, temperature settings, model selection, and tool availability. Agents encapsulate reusable AI personalities or capabilities.

**Conversations**: Persistent chat sessions maintaining message history, context, and metadata. Conversations track multi-turn interactions and enable features like conversation branching and resumption.

**Tools/Functions**: Elixir functions exposed to LLMs for taking actions. Tools enable agents to query databases, call APIs, perform calculations, or interact with your application logic.

**Providers**: Abstraction layer supporting different LLM backends (OpenAI, Anthropic Claude, local models via Ollama/LM Studio). Swap providers without changing application logic.

**Embeddings**: Vector representations of text for semantic search, retrieval-augmented generation (RAG), and similarity matching. Ash AI handles embedding generation and storage.

**Streaming**: Support for streaming LLM responses token-by-token, enabling real-time user interfaces and efficient handling of long responses.

**Multi-Agent Orchestration**: Patterns for multiple agents collaborating, debating, or working in pipelines. Enables sophisticated AI workflows beyond single-agent interactions.

## 📊 Comparison Chart

---

| Aspect | Ash AI | LangChain | Instructor | Guidance | Semantic Kernel | AutoGen | Haystack |
|--------|--------|-----------|------------|----------|-----------------|---------|----------|
| **Language** | Elixir | Python/JS | Python | Python | C#/Python | Python | Python |
| **Paradigm** | Declarative resources | Imperative chains | Structured outputs | Constrained generation | Skills/planners | Multi-agent conversations | Pipeline-based |
| **Framework Integration** | Deep (Ash) | Standalone | Standalone | Standalone | .NET integration | Standalone | Standalone |
| **Multi-Agent** | Built-in patterns | Via chains | Manual | Manual | Via planners | Core feature | Manual |
| **Tool Calling** | Declarative tools | Function calling | Function schemas | Grammar constraints | Skills/functions | Function registration | Custom tools |
| **Conversation Persistence** | Ash resources/DB | Memory classes | Manual | Manual | Memory stores | Conversation logs | Manual |
| **Streaming** | Yes | Yes | No | Yes | Yes | Yes | Yes |
| **Local Models** | Ollama/LM Studio | Ollama/HF/local | OpenAI compatible | Any model | OpenAI/local | OpenAI/local | Multiple backends |
| **Type Safety** | Elixir specs/types | Python typing | Pydantic models | Python typing | C# types | Python typing | Python typing |
| **Testing** | Ash test patterns | Pytest/mocks | Pytest | Pytest | xUnit | Pytest | Pytest |
| **Embeddings/RAG** | Built-in | LangChain modules | Manual | Manual | Memory connectors | Retrieval agents | Document stores |
| **Provider Abstraction** | Provider behavior | LLM abstraction | OpenAI focused | Model agnostic | Connector abstraction | Model abstraction | Backend abstraction |

## 💧 Elixir and Ash Framework Foundation

---

**Why Elixir for AI**: Elixir's concurrency model (BEAM VM) naturally handles multiple concurrent LLM requests, agent interactions, and streaming responses. Fault tolerance (OTP) ensures system reliability when external APIs fail. Hot code reloading enables updating AI logic without downtime.

**Ash Framework Integration**: Ash AI leverages Ash's resource system, meaning agents and conversations benefit from: validations (ensure valid inputs), calculations (derived fields), pub/sub (real-time updates), authorization (who can access which conversations), relationships (link messages to users), and changesets (structured updates).

**Data Layer Agnostic**: Ash abstracts data storage—use PostgreSQL with pgvector for embeddings, Mnesia for in-memory conversations, or ETS for caching. Swap data layers without changing AI logic.

**Phoenix LiveView Synergy**: Ash AI integrates seamlessly with Phoenix LiveView for real-time AI interfaces. Stream responses directly to LiveView components, update UI as agents think, show typing indicators during generation.

## 🔧 Provider Configuration and API Keys

---

**Supported Providers**: OpenAI (GPT-4, GPT-4 Turbo, GPT-3.5), Anthropic (Claude 3.5 Sonnet, Claude 3 Opus, Claude 3 Haiku), local models via Ollama, LM Studio, or OpenAI-compatible APIs.

**API Key Configuration**: Configure via application environment or runtime:
```elixir
# config/runtime.exs
config :ash_ai, :providers,
  openai: [
    api_key: System.get_env("OPENAI_API_KEY"),
    organization: System.get_env("OPENAI_ORG_ID")
  ],
  anthropic: [
    api_key: System.get_env("ANTHROPIC_API_KEY")
  ]
```

**Bring Your Own Keys (BYOK)**: For multi-tenant applications, store API keys per user/organization:
```elixir
defmodule MyApp.Organization do
  use Ash.Resource, data_layer: AshPostgres.DataLayer
  
  attributes do
    uuid_primary_key :id
    attribute :openai_api_key, :string, sensitive?: true
    attribute :anthropic_api_key, :string, sensitive?: true
  end
  
  # Encrypt keys at rest with Cloak or similar
end

# Use organization's key for requests
agent = build_agent(
  model: "gpt-4",
  api_key: organization.openai_api_key
)
```

**Environment-Based Selection**: Different providers for dev/test/prod:
```elixir
config :ash_ai, :default_provider,
  dev: {:ollama, model: "llama3"},
  test: {:mock, responses: [...}],
  prod: {:anthropic, model: "claude-3-5-sonnet-20241022"}
```

**Rate Limiting and Quotas**: Implement per-user rate limiting using Ash policies and calculations:
```elixir
policies do
  policy action_type(:create) do
    authorize_if actor_attribute_equals(:role, :premium)
    authorize_if expr(
      fragment("SELECT COUNT(*) FROM messages WHERE user_id = ? AND inserted_at > NOW() - INTERVAL '1 hour'", ^actor(:id)) < 100
    )
  end
end
```

## 🤖 Local Model Integration

---

**Ollama Setup**: Run models locally with Ollama (supports Llama 3, Mistral, CodeLlama, etc.):
```bash
# Install Ollama
curl -fsSL https://ollama.com/install.sh | sh

# Pull model
ollama pull llama3

# Start server (default http://localhost:11434)
ollama serve
```

**Ollama Provider Configuration**:
```elixir
config :ash_ai, :providers,
  ollama: [
    base_url: "http://localhost:11434",
    default_model: "llama3"
  ]

# In your agent
defmodule MyApp.Agent do
  use AshAI.Agent
  
  agent do
    provider :ollama
    model "llama3"
    system_prompt "You are a helpful assistant."
    temperature 0.7
  end
end
```

**LM Studio Integration**: LM Studio provides OpenAI-compatible API for local models:
```elixir
# LM Studio runs OpenAI-compatible server on http://localhost:1234
config :ash_ai, :providers,
  lm_studio: [
    base_url: "http://localhost:1234/v1",
    api_key: "not-needed", # LM Studio doesn't require key
    model: "local-model"
  ]

# Use OpenAI provider with LM Studio URL
agent do
  provider :openai
  base_url "http://localhost:1234/v1"
  model "TheBloke/Mistral-7B-Instruct-v0.2-GGUF"
  api_key "lm-studio" # Any non-empty string
end
```

**vLLM for Production Local Serving**: For production local deployments, use vLLM for optimized inference:
```bash
# Run vLLM server
python -m vllm.entrypoints.openai.api_server \
  --model meta-llama/Llama-3-8b-instruct \
  --port 8000

# Configure as OpenAI-compatible endpoint
config :ash_ai, :providers,
  vllm: [
    base_url: "http://localhost:8000/v1",
    api_key: "EMPTY"
  ]
```

**Quantization and Performance**: Local models often use quantization (GGUF, GPTQ) for efficiency:
```elixir
# Specify quantized model
agent do
  provider :ollama
  model "llama3:7b-q4_K_M" # 4-bit quantization
  context_length 4096
  num_gpu 1 # GPU layers
end
```

**Embedding Models Locally**: Use local embedding models for semantic search:
```elixir
config :ash_ai, :embeddings,
  provider: :ollama,
  model: "nomic-embed-text"

# Generate embeddings
{:ok, embedding} = AshAI.Embeddings.embed(
  "Your text here",
  provider: :ollama,
  model: "nomic-embed-text"
)
```

## 💬 Basic Chatbot Implementation

---

**Define Resources**: Create conversation, message, and agent resources:
```elixir
defmodule MyApp.Conversation do
  use Ash.Resource,
    data_layer: AshPostgres.DataLayer,
    extensions: [AshAI.Conversation]
  
  postgres do
    table "conversations"
    repo MyApp.Repo
  end
  
  attributes do
    uuid_primary_key :id
    attribute :title, :string
    create_timestamp :inserted_at
    update_timestamp :updated_at
  end
  
  relationships do
    has_many :messages, MyApp.Message
    belongs_to :user, MyApp.User
  end
  
  actions do
    defaults [:create, :read, :update, :destroy]
  end
end

defmodule MyApp.Message do
  use Ash.Resource,
    data_layer: AshPostgres.DataLayer,
    extensions: [AshAI.Message]
  
  attributes do
    uuid_primary_key :id
    attribute :role, :atom, constraints: [one_of: [:user, :assistant, :system]]
    attribute :content, :string
    attribute :metadata, :map
    create_timestamp :inserted_at
  end
  
  relationships do
    belongs_to :conversation, MyApp.Conversation
  end
  
  actions do
    defaults [:create, :read]
  end
end
```

**Define Agent**:
```elixir
defmodule MyApp.ChatAgent do
  use AshAI.Agent
  
  agent do
    provider :anthropic
    model "claude-3-5-sonnet-20241022"
    system_prompt """
    You are a helpful, friendly chatbot. Respond conversationally
    and assist users with their questions.
    """
    temperature 0.7
    max_tokens 2048
  end
  
  # Optional: Add tools
  tools do
    tool :get_weather do
      description "Get current weather for a location"
      parameters do
        parameter :location, :string, required: true
      end
      
      execute fn %{location: location} ->
        # Call weather API
        {:ok, "The weather in #{location} is sunny, 72°F"}
      end
    end
  end
end
```

**Chat Endpoint**:
```elixir
defmodule MyAppWeb.ChatController do
  use MyAppWeb, :controller
  
  def send_message(conn, %{"conversation_id" => conv_id, "message" => content}) do
    user = conn.assigns.current_user
    
    # Load conversation
    conversation = MyApp.Conversation
    |> Ash.Query.filter(id == ^conv_id and user_id == ^user.id)
    |> Ash.Query.load(:messages)
    |> Ash.read_one!()
    
    # Add user message
    user_message = MyApp.Message
    |> Ash.Changeset.for_create(:create, %{
      conversation_id: conv_id,
      role: :user,
      content: content
    })
    |> Ash.create!()
    
    # Generate response
    {:ok, response} = AshAI.chat(
      MyApp.ChatAgent,
      messages: build_message_history(conversation),
      user: user
    )
    
    # Save assistant message
    assistant_message = MyApp.Message
    |> Ash.Changeset.for_create(:create, %{
      conversation_id: conv_id,
      role: :assistant,
      content: response.content
    })
    |> Ash.create!()
    
    json(conn, %{message: assistant_message})
  end
  
  defp build_message_history(conversation) do
    Enum.map(conversation.messages, fn msg ->
      %{role: msg.role, content: msg.content}
    end)
  end
end
```

**LiveView Streaming Chat**:
```elixir
defmodule MyAppWeb.ChatLive do
  use MyAppWeb, :live_view
  
  def mount(_params, _session, socket) do
    {:ok, assign(socket, 
      messages: [],
      input: "",
      streaming: false
    )}
  end
  
  def handle_event("send", %{"message" => content}, socket) do
    socket = assign(socket, 
      messages: socket.assigns.messages ++ [%{role: :user, content: content}],
      input: "",
      streaming: true,
      current_response: ""
    )
    
    # Start streaming
    Task.async(fn ->
      AshAI.chat_stream(
        MyApp.ChatAgent,
        messages: build_messages(socket.assigns.messages),
        callback: fn chunk ->
          send(self(), {:chunk, chunk})
        end
      )
    end)
    
    {:noreply, socket}
  end
  
  def handle_info({:chunk, chunk}, socket) do
    updated_response = socket.assigns.current_response <> chunk.content
    {:noreply, assign(socket, current_response: updated_response)}
  end
  
  def handle_info({:complete, final_response}, socket) do
    socket = assign(socket,
      messages: socket.assigns.messages ++ [%{role: :assistant, content: final_response}],
      streaming: false,
      current_response: ""
    )
    {:noreply, socket}
  end
end
```

## 🧑‍🤝‍🧑 Multi-Agent Mastermind Debates

---

**Mastermind Concept**: Multiple AI agents with different perspectives debate a topic, challenge each other's arguments, and synthesize conclusions. Useful for exploring complex topics, red-teaming ideas, or generating diverse perspectives.

**Define Specialized Agents**:
```elixir
defmodule MyApp.Agents.Optimist do
  use AshAI.Agent
  
  agent do
    provider :anthropic
    model "claude-3-5-sonnet-20241022"
    system_prompt """
    You are an optimistic debater who sees possibilities and potential.
    Look for positive outcomes, opportunities, and constructive solutions.
    Challenge overly negative or pessimistic views with realistic optimism.
    """
    temperature 0.8
  end
end

defmodule MyApp.Agents.Pessimist do
  use AshAI.Agent
  
  agent do
    provider :anthropic
    model "claude-3-5-sonnet-20241022"
    system_prompt """
    You are a critical thinker who identifies risks and challenges.
    Point out potential problems, unintended consequences, and obstacles.
    Question assumptions and challenge overly optimistic views.
    """
    temperature 0.8
  end
end

defmodule MyApp.Agents.Realist do
  use AshAI.Agent
  
  agent do
    provider :anthropic
    model "claude-3-5-sonnet-20241022"
    system_prompt """
    You are a pragmatic mediator who balances perspectives.
    Consider both opportunities and risks. Synthesize opposing views.
    Focus on practical, actionable insights.
    """
    temperature 0.6
  end
end

defmodule MyApp.Agents.Moderator do
  use AshAI.Agent
  
  agent do
    provider :anthropic
    model "claude-3-opus-20240229" # More powerful model for synthesis
    system_prompt """
    You are a skilled moderator facilitating a debate.
    Summarize key points, identify areas of agreement/disagreement,
    ask probing questions to deepen discussion, and synthesize
    final conclusions from multiple perspectives.
    """
    temperature 0.7
  end
end
```

**Debate Orchestration**:
```elixir
defmodule MyApp.Debate do
  use Ash.Resource,
    data_layer: AshPostgres.DataLayer
  
  attributes do
    uuid_primary_key :id
    attribute :topic, :string
    attribute :rounds, :integer, default: 3
    attribute :transcript, {:array, :map}, default: []
    attribute :conclusion, :string
    attribute :status, :atom, default: :pending
  end
  
  actions do
    create :start_debate do
      argument :topic, :string, allow_nil?: false
      argument :rounds, :integer, default: 3
      
      change fn changeset, _context ->
        Ash.Changeset.after_action(changeset, fn _changeset, debate ->
          # Start async debate process
          Task.start(fn -> 
            MyApp.DebateOrchestrator.run_debate(debate)
          end)
          {:ok, debate}
        end)
      end
    end
  end
end

defmodule MyApp.DebateOrchestrator do
  @agents [
    {MyApp.Agents.Optimist, "Optimist"},
    {MyApp.Agents.Pessimist, "Pessimist"},
    {MyApp.Agents.Realist, "Realist"}
  ]
  
  def run_debate(debate) do
    initial_prompt = """
    Topic: #{debate.topic}
    
    Please share your initial perspective on this topic.
    """
    
    # Round 1: Initial perspectives
    transcript = Enum.map(@agents, fn {agent, name} ->
      {:ok, response} = AshAI.chat(agent, 
        messages: [%{role: :user, content: initial_prompt}]
      )
      
      %{
        round: 1,
        speaker: name,
        content: response.content,
        timestamp: DateTime.utc_now()
      }
    end)
    
    # Subsequent rounds: Responses to each other
    transcript = Enum.reduce(2..debate.rounds, transcript, fn round, acc ->
      debate_round(round, acc, debate.topic)
    end)
    
    # Moderator synthesis
    conclusion = synthesize_debate(transcript, debate.topic)
    
    # Update debate
    debate
    |> Ash.Changeset.for_update(:update, %{
      transcript: transcript,
      conclusion: conclusion,
      status: :complete
    })
    |> Ash.update!()
  end
  
  defp debate_round(round, previous_transcript, topic) do
    context = build_context(previous_transcript)
    
    new_responses = Enum.map(@agents, fn {agent, name} ->
      prompt = """
      Topic: #{topic}
      
      Previous discussion:
      #{context}
      
      Respond to the other participants' points. Challenge weak arguments,
      acknowledge strong points, and advance the discussion.
      """
      
      {:ok, response} = AshAI.chat(agent,
        messages: [%{role: :user, content: prompt}]
      )
      
      %{
        round: round,
        speaker: name,
        content: response.content,
        timestamp: DateTime.utc_now()
      }
    end)
    
    previous_transcript ++ new_responses
  end
  
  defp build_context(transcript) do
    transcript
    |> Enum.take(-6) # Last 2 contributions from each
    |> Enum.map(fn entry ->
      "#{entry.speaker}: #{entry.content}"
    end)
    |> Enum.join("\n\n")
  end
  
  defp synthesize_debate(transcript, topic) do
    summary = Enum.map(transcript, fn entry ->
      "Round #{entry.round} - #{entry.speaker}:\n#{entry.content}"
    end) |> Enum.join("\n\n---\n\n")
    
    prompt = """
    You are moderating a debate on: #{topic}
    
    Here is the complete transcript:
    
    #{summary}
    
    Please synthesize the debate by:
    1. Summarizing key arguments from each perspective
    2. Identifying areas of consensus and disagreement
    3. Extracting the most valuable insights
    4. Providing a balanced conclusion that considers all viewpoints
    """
    
    {:ok, response} = AshAI.chat(
      MyApp.Agents.Moderator,
      messages: [%{role: :user, content: prompt}]
    )
    
    response.content
  end
end
```

**Real-Time Debate Viewing**:
```elixir
defmodule MyAppWeb.DebateLive do
  use MyAppWeb, :live_view
  
  def mount(%{"id" => id}, _session, socket) do
    debate = MyApp.Debate |> Ash.get!(id)
    
    if connected?(socket) do
      # Subscribe to debate updates
      Phoenix.PubSub.subscribe(MyApp.PubSub, "debate:#{id}")
    end
    
    {:ok, assign(socket, debate: debate)}
  end
  
  def handle_info({:debate_update, updated_debate}, socket) do
    {:noreply, assign(socket, debate: updated_debate)}
  end
  
  # Broadcast from DebateOrchestrator after each contribution
  defp broadcast_update(debate) do
    Phoenix.PubSub.broadcast(
      MyApp.PubSub,
      "debate:#{debate.id}",
      {:debate_update, debate}
    )
  end
end
```

**Alternative Patterns**:

**Sequential Pipeline**: Agents process in sequence, each building on previous:
```elixir
def sequential_pipeline(topic) do
  # Agent 1: Generate initial analysis
  {:ok, analysis} = AshAI.chat(AnalystAgent, ...)
  
  # Agent 2: Critique analysis
  {:ok, critique} = AshAI.chat(CriticAgent, 
    messages: [%{role: :user, content: "Analyze: #{analysis.content}"}]
  )
  
  # Agent 3: Synthesize final output
  {:ok, synthesis} = AshAI.chat(SynthesizerAgent,
    messages: [%{role: :user, content: "Combine: #{analysis.content}\n\n#{critique.content}"}]
  )
end
```

**Parallel Processing**: All agents process simultaneously, moderator combines:
```elixir
def parallel_analysis(topic) do
  tasks = @agents
  |> Enum.map(fn {agent, _name} ->
    Task.async(fn ->
      AshAI.chat(agent, messages: [%{role: :user, content: topic}])
    end)
  end)
  
  responses = Task.await_many(tasks, 30_000)
  
  # Moderator combines responses
  combined = Enum.map_join(responses, "\n\n", fn {:ok, resp} -> resp.content end)
  AshAI.chat(ModeratorAgent, messages: [%{role: :user, content: combined}])
end
```

## 🛠️ Tool Calling and Function Execution

---

**Define Tools in Agent**:
```elixir
defmodule MyApp.AssistantAgent do
  use AshAI.Agent
  
  agent do
    provider :anthropic
    model "claude-3-5-sonnet-20241022"
    system_prompt "You are a helpful assistant with access to various tools."
  end
  
  tools do
    # Database query tool
    tool :search_users do
      description "Search for users in the database"
      parameters do
        parameter :query, :string, required: true
        parameter :limit, :integer, default: 10
      end
      
      execute fn %{query: query, limit: limit} ->
        users = MyApp.User
        |> Ash.Query.filter(contains(name, ^query))
        |> Ash.Query.limit(limit)
        |> Ash.read!()
        
        {:ok, %{
          count: length(users),
          users: Enum.map(users, &format_user/1)
        }}
      end
    end
    
    # API call tool
    tool :get_stock_price do
      description "Get current stock price for a symbol"
      parameters do
        parameter :symbol, :string, required: true
      end
      
      execute fn %{symbol: symbol} ->
        case HTTPoison.get("https://api.example.com/stock/#{symbol}") do
          {:ok, %{status_code: 200, body: body}} ->
            price = Jason.decode!(body)["price"]
            {:ok, %{symbol: symbol, price: price, currency: "USD"}}
          
          {:error, reason} ->
            {:error, "Failed to fetch stock price: #{inspect(reason)}"}
        end
      end
    end
    
    # Calculation tool
    tool :calculate do
      description "Perform mathematical calculations"
      parameters do
        parameter :expression, :string, required: true
      end
      
      execute fn %{expression: expr} ->
        # Use Abacus or similar for safe expression evaluation
        case Abacus.eval(expr) do
          {:ok, result} -> {:ok, %{result: result}}
          {:error, _} -> {:error, "Invalid expression"}
        end
      end
    end
    
    # Resource action tool
    tool :create_task do
      description "Create a new task in the todo list"
      parameters do
        parameter :title, :string, required: true
        parameter :description, :string
        parameter :due_date, :date
      end
      
      execute fn params ->
        case MyApp.Task
        |> Ash.Changeset.for_create(:create, params)
        |> Ash.create() do
          {:ok, task} -> {:ok, %{id: task.id, title: task.title}}
          {:error, changeset} -> {:error, "Failed to create task"}
        end
      end
    end
  end
end
```

**Tool Calling Flow**:
```elixir
# User asks: "What's the stock price of AAPL?"
{:ok, response} = AshAI.chat(
  MyApp.AssistantAgent,
  messages: [%{role: :user, content: "What's the stock price of AAPL?"}]
)

# Response includes tool call
# response.tool_calls => [%{name: "get_stock_price", arguments: %{symbol: "AAPL"}}]

# Tools execute automatically, results fed back to model
# Final response: "The current stock price of AAPL is $178.25 USD."
```

**Authorizing Tool Use**:
```elixir
tools do
  tool :delete_user do
    description "Delete a user account (admin only)"
    parameters do
      parameter :user_id, :string, required: true
    end
    
    # Authorization check
    authorize fn params, context ->
      if context.actor.role == :admin do
        :ok
      else
        {:error, "Unauthorized: admin access required"}
      end
    end
    
    execute fn %{user_id: user_id} ->
      MyApp.User
      |> Ash.get!(user_id)
      |> Ash.destroy!()
      
      {:ok, %{message: "User deleted successfully"}}
    end
  end
end

# Pass actor context
AshAI.chat(
  MyApp.AssistantAgent,
  messages: [...],
  context: %{actor: current_user}
)
```

**Tool Result Formatting**:
```elixir
tool :search_products do
  execute fn params ->
    products = search_products(params)
    
    # Format results for LLM comprehension
    {:ok, %{
      summary: "Found #{length(products)} products",
      products: Enum.map(products, fn p ->
        %{
          name: p.name,
          price: "$#{p.price}",
          in_stock: p.inventory > 0,
          rating: "#{p.rating}/5 stars"
        }
      end)
    }}
  end
end
```

**Parallel Tool Execution**: Some providers support parallel tool calls:
```elixir
# User: "Get weather for NYC and LA, and current time in Tokyo"
# Response may include multiple tool calls executed in parallel:
[
  %{name: "get_weather", arguments: %{location: "NYC"}},
  %{name: "get_weather", arguments: %{location: "LA"}},
  %{name: "get_time", arguments: %{timezone: "Asia/Tokyo"}}
]

# All tools execute concurrently for efficiency
```

## 🔌 Model Context Protocol (MCP) Integration

---

Ash AI supports [[MCP]] (Model Context Protocol) for standardized LLM access to external data and tools. You can create MCP servers exposing Ash resources and tools, connect to external MCP servers, and use MCP tools within agents.

See [[MCP]] for full protocol details, server/client implementation patterns, and transport options.

## 🎲 Deterministic Results and Testing

---

**Temperature Control**: Lower temperature (0.0-0.3) for deterministic outputs:
```elixir
agent do
  temperature 0.0 # Most deterministic
  top_p 1.0
  seed 12345 # Some providers support seeding
end
```

**Seed Support**: OpenAI and some providers support seed parameter:
```elixir
{:ok, response} = AshAI.chat(
  MyApp.Agent,
  messages: [...],
  seed: 42 # Reproducible outputs
)
```

**Structured Output**: Use function calling for deterministic structure:
```elixir
defmodule MyApp.StructuredAgent do
  use AshAI.Agent
  
  agent do
    provider :openai
    model "gpt-4-turbo-preview"
    temperature 0.0
  end
  
  tools do
    tool :return_structured_data do
      description "Always return data in this exact structure"
      parameters do
        parameter :analysis, :string, required: true
        parameter :confidence, :float, required: true
        parameter :categories, {:array, :string}, required: true
      end
      
      execute fn params ->
        {:ok, params} # Just return the structured input
      end
    end
  end
end

# Instruct model to use tool for response
{:ok, response} = AshAI.chat(
  MyApp.StructuredAgent,
  messages: [
    %{role: :system, content: "Always respond using the return_structured_data tool"},
    %{role: :user, content: "Analyze this text: #{text}"}
  ]
)

# response.tool_calls will have structured data
```

**JSON Mode**: OpenAI supports JSON mode for guaranteed JSON output:
```elixir
agent do
  provider :openai
  model "gpt-4-turbo-preview"
  response_format %{type: "json_object"}
end

# System prompt must request JSON
messages = [
  %{role: :system, content: "Respond with valid JSON"},
  %{role: :user, content: "Analyze sentiment of: #{text}"}
]

{:ok, response} = AshAI.chat(agent, messages: messages)
# response.content is guaranteed valid JSON
parsed = Jason.decode!(response.content)
```

**Mock Provider for Testing**:
```elixir
# test/support/mocks.ex
defmodule MyApp.MockProvider do
  use AshAI.Provider
  
  @responses %{
    "hello" => "Hi there!",
    "weather" => "The weather is sunny and 72°F",
    default: "I don't have a mock response for that"
  }
  
  def chat(messages, _opts) do
    last_message = List.last(messages)
    content = last_message.content
    
    response = Map.get(@responses, content, @responses.default)
    
    {:ok, %{
      content: response,
      model: "mock",
      usage: %{prompt_tokens: 10, completion_tokens: 5}
    }}
  end
end

# test/my_app/chat_test.exs
defmodule MyApp.ChatTest do
  use MyApp.DataCase
  
  setup do
    # Use mock provider for tests
    Application.put_env(:ash_ai, :default_provider, MyApp.MockProvider)
    :ok
  end
  
  test "chatbot responds to greeting" do
    {:ok, conversation} = MyApp.Conversation.create(%{})
    
    {:ok, response} = MyApp.ChatService.send_message(
      conversation.id,
      "hello"
    )
    
    assert response.content == "Hi there!"
  end
end
```

**Fixtures for Deterministic Testing**:
```elixir
defmodule MyApp.Fixtures do
  def mock_responses do
    %{
      sentiment_analysis: %{
        tool_calls: [
          %{
            name: "analyze_sentiment",
            arguments: %{
              sentiment: "positive",
              confidence: 0.95,
              entities: ["product", "quality"]
            }
          }
        ]
      },
      summarization: %{
        content: "This is a test summary of the document."
      }
    }
  end
end

# In tests
def chat(_agent, messages: messages, _opts) do
  key = determine_response_key(messages)
  {:ok, MyApp.Fixtures.mock_responses()[key]}
end
```

**Snapshot Testing**:
```elixir
test "agent response format matches snapshot" do
  response = chat_with_agent("Explain quantum computing")
  
  # Save first run as snapshot
  # assert_snapshot(response, "quantum_computing_response.json")
  
  # Subsequent runs compare against snapshot
  stored_response = load_snapshot("quantum_computing_response.json")
  assert response.content == stored_response.content
end
```

**Property-Based Testing**:
```elixir
use ExUnitProperties

property "agent always returns valid JSON for structured queries" do
  check all query <- string(:alphanumeric, min_length: 1, max_length: 100) do
    {:ok, response} = AshAI.chat(
      StructuredAgent,
      messages: [%{role: :user, content: query}]
    )
    
    # Should always be valid JSON
    assert {:ok, _parsed} = Jason.decode(response.content)
  end
end
```

**Regression Testing with Real Responses**:
```elixir
# Record mode: Save actual LLM responses
defmodule MyApp.RecordingProvider do
  def chat(messages, opts) do
    {:ok, response} = RealProvider.chat(messages, opts)
    
    # Save response for replay
    save_response(messages, response)
    
    {:ok, response}
  end
end

# Replay mode: Use saved responses
defmodule MyApp.ReplayProvider do
  def chat(messages, _opts) do
    {:ok, load_response(messages)}
  end
end

# Toggle via config
config :ash_ai, :test_mode, :record # or :replay
```

## 📊 Embeddings and Semantic Search

---

Ash AI provides built-in support for [[Embeddings]], semantic search with [[pgvector]], and [[RAG]] (Retrieval-Augmented Generation) patterns. Define embedding fields on Ash resources, auto-generate embeddings on create/update, and use pgvector for similarity search.

See [[RAG]] for retrieval patterns (chunking, hybrid search, reranking) and [[Embeddings]] for vector concepts.

## 🔗 Related Concepts

---

- [[Ash Framework]]
- [[Elixir]]
- [[Phoenix Framework]]
- [[Large Language Models]]
- [[LLM]]
- [[GPT]]
- [[Claude]]
- [[Anthropic]]
- [[OpenAI]]
- [[LangChain]]
- [[Prompt Engineering]]
- [[Function Calling]]
- [[Tool Use]]
- [[Multi-Agent Systems]]
- [[RAG]] (Retrieval-Augmented Generation)
- [[Embeddings]]
- [[Vector Databases]]
- [[pgvector]]
- [[Semantic Search]]
- [[MCP]] (Model Context Protocol)
- [[Ollama]]
- [[LM Studio]]
- [[Phoenix LiveView]]
- [[OTP]]
- [[BEAM VM]]
- [[Streaming]]
- [[Natural Language Processing]]
- [[Chatbots]]
- [[AI Agents]]

## 💡 Advanced Patterns and Use Cases

---

**Agent Specialization Hierarchy**: Create specialized agents that delegate to each other:
```elixir
defmodule MyApp.Agents.Router do
  use AshAI.Agent
  
  agent do
    provider :anthropic
    model "claude-3-5-sonnet-20241022"
    system_prompt """
    You are a routing agent. Analyze the user's request and determine
    which specialist agent should handle it. Respond with just the
    agent name: CodeAgent, DataAgent, or SupportAgent.
    """
    temperature 0.0
  end
end

defmodule MyApp.AgentOrchestrator do
  @agents %{
    "CodeAgent" => MyApp.Agents.Code,
    "DataAgent" => MyApp.Agents.Data,
    "SupportAgent" => MyApp.Agents.Support
  }
  
  def handle_request(user_message) do
    # Route to appropriate agent
    {:ok, routing} = AshAI.chat(
      MyApp.Agents.Router,
      messages: [%{role: :user, content: user_message}]
    )
    
    agent_name = String.trim(routing.content)
    agent = @agents[agent_name]
    
    # Execute with specialist
    AshAI.chat(agent, messages: [%{role: :user, content: user_message}])
  end
end
```

**Conversation Branching**: Support exploring alternative conversation paths:
```elixir
defmodule MyApp.ConversationBranch do
  use Ash.Resource
  
  attributes do
    uuid_primary_key :id
    attribute :parent_message_id, :uuid
    attribute :branch_point, :integer # Message index where branch occurred
  end
  
  relationships do
    belongs_to :conversation, MyApp.Conversation
    has_many :messages, MyApp.Message
  end
end

# Create branch from specific message
def create_branch(conversation, message_index, new_user_message) do
  # Copy messages up to branch point
  base_messages = conversation.messages
  |> Enum.take(message_index)
  
  # Create branch
  branch = MyApp.ConversationBranch
  |> Ash.Changeset.for_create(:create, %{
    conversation_id: conversation.id,
    branch_point: message_index
  })
  |> Ash.create!()
  
  # Continue conversation in branch with new message
  # ...
end
```

**Iterative Refinement**: Have agent improve its own output:
```elixir
def iterative_refinement(initial_prompt, iterations \\ 3) do
  {:ok, response} = AshAI.chat(Agent, messages: [
    %{role: :user, content: initial_prompt}
  ])
  
  Enum.reduce(1..iterations, response, fn i, current ->
    {:ok, improved} = AshAI.chat(Agent, messages: [
      %{role: :user, content: """
      Here is your previous response:
      
      #{current.content}
      
      Please improve it by:
      1. Fixing any errors
      2. Adding more detail
      3. Improving clarity
      
      This is iteration #{i} of #{iterations}.
      """}
    ])
    
    improved
  end)
end
```

**Cost Tracking**: Monitor API usage and costs:
```elixir
defmodule MyApp.AIUsage do
  use Ash.Resource
  
  attributes do
    uuid_primary_key :id
    attribute :user_id, :uuid
    attribute :model, :string
    attribute :prompt_tokens, :integer
    attribute :completion_tokens, :integer
    attribute :total_cost, :decimal
    create_timestamp :inserted_at
  end
  
  calculations do
    calculate :estimated_cost, :decimal do
      calculation fn records, _context ->
        Enum.map(records, fn record ->
          calculate_cost(record.model, record.prompt_tokens, record.completion_tokens)
        end)
      end
    end
  end
end

# Track usage
defmodule MyApp.AIWrapper do
  def chat(agent, opts) do
    {:ok, response} = AshAI.chat(agent, opts)
    
    # Record usage
    MyApp.AIUsage
    |> Ash.Changeset.for_create(:create, %{
      user_id: opts[:user_id],
      model: response.model,
      prompt_tokens: response.usage.prompt_tokens,
      completion_tokens: response.usage.completion_tokens
    })
    |> Ash.create!()
    
    {:ok, response}
  end
end
```

**Caching Responses**: Cache similar queries to reduce costs:
```elixir
defmodule MyApp.ResponseCache do
  def cached_chat(agent, messages, opts \\ []) do
    cache_key = generate_cache_key(agent, messages)
    
    case Cachex.get(:ai_cache, cache_key) do
      {:ok, cached_response} when not is_nil(cached_response) ->
        {:ok, cached_response}
      
      _ ->
        {:ok, response} = AshAI.chat(agent, messages: messages, opts: opts)
        Cachex.put(:ai_cache, cache_key, response, ttl: :timer.hours(24))
        {:ok, response}
    end
  end
  
  defp generate_cache_key(agent, messages) do
    # Hash agent config and messages
    :crypto.hash(:sha256, :erlang.term_to_binary({agent, messages}))
    |> Base.encode16()
  end
end
```

**Rate Limiting**: Protect against abuse and control costs:
```elixir
defmodule MyApp.RateLimiter do
  use Ash.Policy.Check
  
  @impl true
  def describe(_options), do: "rate limit check"
  
  @impl true
  def strict_check(_actor, _resource, _opts) do
    # Implement using Hammer, ExRated, or similar
    case Hammer.check_rate("ai:#{actor.id}", 60_000, 10) do
      {:allow, _count} -> :authorized
      {:deny, _limit} -> {:error, "Rate limit exceeded. Max 10 requests per minute."}
    end
  end
end

# In resource policies
policies do
  policy action_type(:create) do
    authorize_if MyApp.RateLimiter
  end
end
```

## 📚 Further Reading

---

- Ash Framework documentation: https://hexdocs.pm/ash
- Ash AI documentation and guides
- Anthropic Claude API documentation
- OpenAI API documentation and best practices
- "Building LLM Applications" guides from major providers
- Model Context Protocol specification
- LangChain documentation for comparison
- Papers on multi-agent systems and orchestration
- "Prompt Engineering Guide" for effective prompting techniques
- Elixir Forum discussions on AI integration
- Phoenix LiveView real-time AI patterns
- pgvector documentation for vector storage
- Papers on RAG (Retrieval-Augmented Generation)
- OTP design principles for fault-tolerant AI systems
- Blog posts on production LLM deployments
