# MCP

MCP (Model Context Protocol) is an open protocol developed by Anthropic that standardizes how AI applications provide context to Large Language Models. It enables LLMs to securely access data sources, tools, and resources through a unified interface, replacing ad-hoc integrations with a standardized communication layer. MCP transforms LLM integrations from fragmented, vendor-specific implementations into composable, reusable components.

## 🎯 Overview

---

Model Context Protocol establishes a client-server architecture where AI applications (MCP clients) connect to data sources and tools (MCP servers) through a standardized JSON-RPC protocol. This separation of concerns enables LLMs to access databases, file systems, APIs, development tools, and other resources without requiring custom integration code for each service. MCP servers expose resources (data to read), tools (actions to execute), and prompts (reusable templates) that any MCP-compatible client can utilize.

The protocol supports multiple transport mechanisms (stdio, HTTP with SSE, WebSocket), handles authentication and authorization, provides resource discovery, and enables streaming for real-time interactions. MCP is designed for both local development workflows and production deployments, with official SDKs for Python, TypeScript, and Kotlin, plus community implementations in other languages.

## 🧠 Core Concepts

---

**Client-Server Architecture**: MCP clients (AI applications like Claude Desktop, IDEs, custom apps) connect to MCP servers (data sources, tools, services). Clients request resources and invoke tools; servers provide data and execute actions.

**Resources**: Read-only data exposed by servers that LLMs can access. Examples include files, database records, API responses, or documentation. Resources are identified by URIs and can be listed, read, and monitored for changes.

**Tools**: Functions or actions that LLMs can execute through servers. Tools take structured input (JSON schemas) and return results. Examples include file operations, database queries, API calls, or code execution.

**Prompts**: Reusable prompt templates with arguments that servers can provide to clients. Enable standardized workflows and best practices to be shared across applications.

**Sampling**: LLM text generation capabilities exposed by clients to servers. Allows servers to request LLM completions on behalf of tools or workflows.

**Transport Layer**: Communication mechanism between clients and servers. Supports stdio (for local processes), Server-Sent Events over HTTP (for web services), and WebSocket (for bidirectional streaming).

**JSON-RPC Protocol**: Message format for all MCP communications. Uses JSON-RPC 2.0 for request/response patterns and notifications for one-way messages.

**Resource Templates**: URI templates that enable dynamic resource generation with parameters, supporting pattern-based resource access.

**Subscriptions**: Mechanism for clients to receive real-time updates when resources change, enabling reactive LLM applications.

## 📊 Comparison Chart

---

| Aspect | MCP | OpenAI Function Calling | LangChain Tools | Semantic Kernel Plugins | AutoGen Tools | Custom APIs |
|--------|-----|------------------------|-----------------|------------------------|---------------|-------------|
| **Standardization** | Open protocol | Vendor-specific | Framework-specific | Framework-specific | Framework-specific | Ad-hoc |
| **Scope** | Resources + Tools + Prompts | Tools only | Tools + chains | Skills + planners | Tools + agents | Varies |
| **Transport** | Stdio/HTTP/WebSocket | HTTPS | Python function calls | .NET/Python | Python function calls | HTTP/gRPC/custom |
| **Discovery** | Built-in | Manual specification | Manual registration | Manual registration | Manual registration | Documentation |
| **Streaming** | Yes | Yes | Yes | Yes | Yes | Depends |
| **Resource Access** | First-class | Via tools | Via tools | Via connectors | Via tools | Custom |
| **Authentication** | Flexible | API keys | Varies | Varies | Varies | Custom |
| **Language Support** | Any (protocol) | Any (REST) | Python/JS | C#/Python/Java | Python | Any |
| **Local/Remote** | Both | Remote (API) | Both | Both | Both | Both |
| **Composability** | High (protocol) | Medium | High (chains) | High (orchestration) | High (multi-agent) | Low |
| **Resource Subscriptions** | Yes | No | No | No | No | Custom |

## 🏗️ Architecture

---

**Protocol Stack**: MCP operates at three layers: (1) Transport layer (stdio/HTTP/WebSocket) handles message delivery, (2) Protocol layer (JSON-RPC) structures requests/responses, (3) Application layer defines resources/tools/prompts semantics.

**Message Flow**: Client sends initialization request → Server responds with capabilities → Client discovers resources/tools/prompts → Client requests resources or invokes tools → Server returns data or results → Optional: Server sends notifications for resource updates.

**Capability Negotiation**: During initialization, client and server exchange supported features. Server declares available capabilities (resources, tools, prompts, logging). Client declares requirements and optional features.

**URI Scheme**: Resources identified by URIs with custom schemes. Examples: `file:///path/to/document`, `postgres://table/users`, `github://repo/owner/name/issues`. Schemes are server-defined and can be hierarchical.

**Tool Schema**: Tools defined with JSON Schema for parameters. Servers declare tool name, description, and input schema. Clients validate arguments before invoking tools.

**Error Handling**: Protocol uses JSON-RPC error codes. Standard errors include invalid request, method not found, invalid parameters. Servers can define custom error codes for domain-specific failures.

## 🔌 MCP Servers

---

**Server Implementation**: Servers expose resources, tools, and prompts through standardized interface:
```python
# Python MCP Server
from mcp.server import Server
from mcp.server.stdio import stdio_server
from mcp.types import Resource, Tool, TextContent

app = Server("my-server")

@app.list_resources()
async def list_resources() -> list[Resource]:
    """List available resources"""
    return [
        Resource(
            uri="file:///documents/readme.md",
            name="README",
            mimeType="text/markdown",
            description="Project documentation"
        )
    ]

@app.read_resource()
async def read_resource(uri: str) -> str:
    """Read resource content"""
    if uri == "file:///documents/readme.md":
        with open("/path/to/readme.md") as f:
            return f.read()
    raise ValueError(f"Unknown resource: {uri}")

@app.list_tools()
async def list_tools() -> list[Tool]:
    """List available tools"""
    return [
        Tool(
            name="search_files",
            description="Search files by content",
            inputSchema={
                "type": "object",
                "properties": {
                    "query": {"type": "string"},
                    "path": {"type": "string"}
                },
                "required": ["query"]
            }
        )
    ]

@app.call_tool()
async def call_tool(name: str, arguments: dict) -> list[TextContent]:
    """Execute tool"""
    if name == "search_files":
        results = search_filesystem(arguments["query"], arguments.get("path", "."))
        return [TextContent(type="text", text=f"Found {len(results)} files")]
    raise ValueError(f"Unknown tool: {name}")

# Run server
async def main():
    async with stdio_server() as (read_stream, write_stream):
        await app.run(read_stream, write_stream)

if __name__ == "__main__":
    import asyncio
    asyncio.run(main())
```

**TypeScript Server**:
```typescript
import { Server } from "@modelcontextprotocol/sdk/server/index.js";
import { StdioServerTransport } from "@modelcontextprotocol/sdk/server/stdio.js";
import { ListResourcesRequestSchema, ReadResourceRequestSchema, 
         ListToolsRequestSchema, CallToolRequestSchema } from "@modelcontextprotocol/sdk/types.js";

const server = new Server({
  name: "my-server",
  version: "1.0.0"
}, {
  capabilities: {
    resources: {},
    tools: {}
  }
});

// List resources
server.setRequestHandler(ListResourcesRequestSchema, async () => {
  return {
    resources: [
      {
        uri: "file:///documents/readme.md",
        name: "README",
        mimeType: "text/markdown",
        description: "Project documentation"
      }
    ]
  };
});

// Read resource
server.setRequestHandler(ReadResourceRequestSchema, async (request) => {
  const uri = request.params.uri;
  if (uri === "file:///documents/readme.md") {
    const content = await readFile("/path/to/readme.md", "utf-8");
    return {
      contents: [{ uri, mimeType: "text/markdown", text: content }]
    };
  }
  throw new Error(`Unknown resource: ${uri}`);
});

// List tools
server.setRequestHandler(ListToolsRequestSchema, async () => {
  return {
    tools: [{
      name: "search_files",
      description: "Search files by content",
      inputSchema: {
        type: "object",
        properties: {
          query: { type: "string", description: "Search query" },
          path: { type: "string", description: "Directory to search" }
        },
        required: ["query"]
      }
    }]
  };
});

// Call tool
server.setRequestHandler(CallToolRequestSchema, async (request) => {
  const { name, arguments: args } = request.params;
  
  if (name === "search_files") {
    const results = await searchFiles(args.query, args.path || ".");
    return {
      content: [{ type: "text", text: `Found ${results.length} files` }]
    };
  }
  
  throw new Error(`Unknown tool: ${name}`);
});

// Start server
const transport = new StdioServerTransport();
await server.connect(transport);
```

**Pre-Built Servers**: Official MCP servers available as npm packages:
```bash
# Filesystem server - access files/directories
npx @modelcontextprotocol/server-filesystem /allowed/path

# PostgreSQL server - query databases
npx @modelcontextprotocol/server-postgres postgresql://user:pass@localhost/db

# GitHub server - access repositories
npx @modelcontextprotocol/server-github

# Brave Search server - web search
npx @modelcontextprotocol/server-brave-search

# Google Drive server - access Drive files
npx @modelcontextprotocol/server-gdrive

# Memory server - persistent KV storage
npx @modelcontextprotocol/server-memory
```

**Server Configuration**: Servers can accept configuration via environment variables or arguments:
```json
{
  "mcpServers": {
    "filesystem": {
      "command": "npx",
      "args": ["-y", "@modelcontextprotocol/server-filesystem", "/Users/me/Documents"],
      "env": {
        "ALLOWED_PATHS": "/Users/me/Documents,/Users/me/Projects"
      }
    },
    "postgres": {
      "command": "npx",
      "args": ["-y", "@modelcontextprotocol/server-postgres"],
      "env": {
        "DATABASE_URL": "postgresql://localhost/mydb"
      }
    }
  }
}
```

## 💻 MCP Clients

---

**Claude Desktop Integration**: Official Claude Desktop app supports MCP servers:
```json
// ~/Library/Application Support/Claude/claude_desktop_config.json (macOS)
// %APPDATA%\Claude\claude_desktop_config.json (Windows)
{
  "mcpServers": {
    "filesystem": {
      "command": "npx",
      "args": ["-y", "@modelcontextprotocol/server-filesystem", "/Users/me/Documents"]
    },
    "postgres": {
      "command": "docker",
      "args": ["run", "-i", "--rm", "mcp/postgres"],
      "env": {
        "POSTGRES_CONNECTION": "postgresql://localhost/mydb"
      }
    }
  }
}
```

**Python Client Implementation**:
```python
from mcp.client import Client
from mcp.client.stdio import stdio_client

async def use_mcp_server():
    # Connect to server
    async with stdio_client(
        command="npx",
        args=["-y", "@modelcontextprotocol/server-filesystem", "/path/to/dir"]
    ) as (read, write):
        async with Client(read, write) as client:
            # Initialize
            await client.initialize()
            
            # List resources
            resources = await client.list_resources()
            print(f"Available resources: {[r.name for r in resources.resources]}")
            
            # Read resource
            content = await client.read_resource("file:///path/to/file.txt")
            print(f"Content: {content.contents[0].text}")
            
            # List tools
            tools = await client.list_tools()
            print(f"Available tools: {[t.name for t in tools.tools]}")
            
            # Call tool
            result = await client.call_tool(
                "search_files",
                {"query": "TODO", "path": "/path/to/dir"}
            )
            print(f"Tool result: {result.content[0].text}")
```

**TypeScript Client**:
```typescript
import { Client } from "@modelcontextprotocol/sdk/client/index.js";
import { StdioClientTransport } from "@modelcontextprotocol/sdk/client/stdio.js";

async function useMCPServer() {
  const transport = new StdioClientTransport({
    command: "npx",
    args: ["-y", "@modelcontextprotocol/server-filesystem", "/path/to/dir"]
  });
  
  const client = new Client({
    name: "my-client",
    version: "1.0.0"
  }, {
    capabilities: {}
  });
  
  await client.connect(transport);
  
  // List resources
  const resources = await client.listResources();
  console.log("Resources:", resources.resources.map(r => r.name));
  
  // Read resource
  const content = await client.readResource({ uri: "file:///path/to/file.txt" });
  console.log("Content:", content.contents[0].text);
  
  // List tools
  const tools = await client.listTools();
  console.log("Tools:", tools.tools.map(t => t.name));
  
  // Call tool
  const result = await client.callTool({
    name: "search_files",
    arguments: { query: "TODO" }
  });
  console.log("Result:", result.content[0].text);
  
  await client.close();
}
```

**Custom Client Integration**: Integrate MCP into your application:
```python
class LLMWithMCP:
    def __init__(self, mcp_servers: list[dict]):
        self.mcp_clients = []
        for server_config in mcp_servers:
            client = await self.connect_mcp_server(server_config)
            self.mcp_clients.append(client)
    
    async def connect_mcp_server(self, config):
        # Start server process and create client
        async with stdio_client(**config) as (read, write):
            client = Client(read, write)
            await client.initialize()
            return client
    
    async def get_available_tools(self):
        """Aggregate tools from all MCP servers"""
        all_tools = []
        for client in self.mcp_clients:
            tools = await client.list_tools()
            all_tools.extend(tools.tools)
        return all_tools
    
    async def execute_tool(self, tool_name: str, arguments: dict):
        """Find and execute tool across MCP servers"""
        for client in self.mcp_clients:
            tools = await client.list_tools()
            if any(t.name == tool_name for t in tools.tools):
                return await client.call_tool(tool_name, arguments)
        raise ValueError(f"Tool not found: {tool_name}")
    
    async def chat_with_context(self, message: str):
        # Get available resources for context
        context = []
        for client in self.mcp_clients:
            resources = await client.list_resources()
            for resource in resources.resources:
                content = await client.read_resource(resource.uri)
                context.append(content.contents[0].text)
        
        # Get available tools
        tools = await self.get_available_tools()
        
        # Make LLM request with context and tools
        response = await self.llm_api_call(
            message=message,
            context="\n\n".join(context),
            tools=[self.tool_to_schema(t) for t in tools]
        )
        
        # Handle tool calls
        if response.tool_calls:
            for tool_call in response.tool_calls:
                result = await self.execute_tool(
                    tool_call.name,
                    tool_call.arguments
                )
                # Feed result back to LLM...
        
        return response
```

## 🗄️ Resource Management

---

**Resource Types**: Resources represent readable data with MIME types:
```python
# Text resources
Resource(
    uri="file:///doc.md",
    name="Documentation",
    mimeType="text/markdown",
    description="Project documentation"
)

# Binary resources (base64 encoded)
Resource(
    uri="file:///image.png",
    name="Screenshot",
    mimeType="image/png",
    description="UI screenshot"
)

# Structured data
Resource(
    uri="postgres://users/table",
    name="Users Table",
    mimeType="application/json",
    description="All users in database"
)
```

**Resource Templates**: Dynamic resources with parameters:
```python
@app.list_resources()
async def list_resources() -> list[Resource]:
    return [
        Resource(
            uri="github://repos/{owner}/{repo}/issues",
            name="GitHub Issues",
            description="Issues from GitHub repository",
            mimeType="application/json"
        )
    ]

@app.read_resource()
async def read_resource(uri: str) -> str:
    # Parse URI template
    match = re.match(r"github://repos/([^/]+)/([^/]+)/issues", uri)
    if match:
        owner, repo = match.groups()
        issues = await fetch_github_issues(owner, repo)
        return json.dumps(issues)
    raise ValueError(f"Unknown resource: {uri}")
```

**Resource Subscriptions**: Clients can subscribe to resource changes:
```python
# Server sends notification when resource changes
@app.on_resource_updated()
async def notify_resource_update(uri: str):
    await server.send_notification(
        "notifications/resources/updated",
        {"uri": uri}
    )

# Client handles notifications
@client.on_notification("notifications/resources/updated")
async def handle_resource_update(params):
    uri = params["uri"]
    # Re-fetch resource
    content = await client.read_resource(uri)
    print(f"Resource updated: {uri}")
```

**Resource Listing Patterns**:
```python
# Static list
@app.list_resources()
async def list_resources() -> list[Resource]:
    return [
        Resource(uri="file:///doc1.md", name="Doc 1"),
        Resource(uri="file:///doc2.md", name="Doc 2")
    ]

# Dynamic discovery
@app.list_resources()
async def list_resources() -> list[Resource]:
    files = glob.glob("/docs/**/*.md", recursive=True)
    return [
        Resource(
            uri=f"file://{path}",
            name=os.path.basename(path),
            mimeType="text/markdown"
        )
        for path in files
    ]

# Hierarchical structure
@app.list_resources()
async def list_resources() -> list[Resource]:
    # Represent directory structure as resources
    return [
        Resource(uri="dir:///docs", name="docs/", description="Documentation directory"),
        Resource(uri="dir:///src", name="src/", description="Source code directory"),
        # Individual files under each directory...
    ]
```

## 🔧 Tool Implementation

---

**Tool Definition**: Tools are functions LLMs can invoke:
```python
@app.list_tools()
async def list_tools() -> list[Tool]:
    return [
        Tool(
            name="execute_sql",
            description="Execute SQL query on database",
            inputSchema={
                "type": "object",
                "properties": {
                    "query": {
                        "type": "string",
                        "description": "SQL query to execute"
                    },
                    "limit": {
                        "type": "integer",
                        "description": "Maximum rows to return",
                        "default": 100
                    }
                },
                "required": ["query"]
            }
        ),
        Tool(
            name="create_file",
            description="Create a new file with content",
            inputSchema={
                "type": "object",
                "properties": {
                    "path": {"type": "string"},
                    "content": {"type": "string"},
                    "overwrite": {"type": "boolean", "default": False}
                },
                "required": ["path", "content"]
            }
        )
    ]

@app.call_tool()
async def call_tool(name: str, arguments: dict) -> list[TextContent]:
    if name == "execute_sql":
        query = arguments["query"]
        limit = arguments.get("limit", 100)
        
        # Validate query (prevent destructive operations)
        if any(keyword in query.upper() for keyword in ["DROP", "DELETE", "TRUNCATE"]):
            raise ValueError("Destructive queries not allowed")
        
        results = await db.execute(query, limit=limit)
        return [TextContent(
            type="text",
            text=json.dumps(results, indent=2)
        )]
    
    elif name == "create_file":
        path = arguments["path"]
        content = arguments["content"]
        overwrite = arguments.get("overwrite", False)
        
        # Validate path (prevent directory traversal)
        if ".." in path or path.startswith("/"):
            raise ValueError("Invalid path")
        
        mode = "w" if overwrite else "x"
        with open(path, mode) as f:
            f.write(content)
        
        return [TextContent(
            type="text",
            text=f"File created: {path}"
        )]
    
    raise ValueError(f"Unknown tool: {name}")
```

**Tool Best Practices**:
```python
# Clear descriptions
Tool(
    name="search_codebase",
    description="""
    Search the codebase for files matching a pattern.
    Returns file paths and optionally file contents.
    Supports glob patterns like '**/*.py' or 'src/**/*.ts'.
    """,
    inputSchema={...}
)

# Proper error handling
@app.call_tool()
async def call_tool(name: str, arguments: dict):
    try:
        # Execute tool
        result = await execute_tool_logic(name, arguments)
        return [TextContent(type="text", text=result)]
    except PermissionError as e:
        return [TextContent(
            type="text",
            text=f"Permission denied: {str(e)}",
            isError=True
        )]
    except Exception as e:
        return [TextContent(
            type="text",
            text=f"Tool execution failed: {str(e)}",
            isError=True
        )]

# Progress reporting for long operations
@app.call_tool()
async def call_tool(name: str, arguments: dict):
    if name == "process_large_dataset":
        await server.send_log_message("info", "Starting dataset processing...")
        
        for i, batch in enumerate(process_in_batches(arguments["data"])):
            result = await process_batch(batch)
            progress = (i + 1) / total_batches * 100
            await server.send_log_message("info", f"Progress: {progress:.1f}%")
        
        await server.send_log_message("info", "Processing complete")
        return [TextContent(type="text", text="Dataset processed successfully")]
```

**Tool Composition**: Tools can call other tools or resources:
```python
@app.call_tool()
async def call_tool(name: str, arguments: dict):
    if name == "analyze_codebase":
        # Use other tools
        search_result = await call_tool("search_files", {"query": "*.py"})
        files = parse_file_list(search_result)
        
        # Read resources for each file
        analyses = []
        for file_uri in files:
            content = await read_resource(file_uri)
            analysis = analyze_code(content)
            analyses.append(analysis)
        
        return [TextContent(
            type="text",
            text=json.dumps({"files_analyzed": len(files), "results": analyses})
        )]
```

## 📝 Prompts

---

**Prompt Templates**: Reusable prompt patterns with arguments:
```python
from mcp.types import Prompt, PromptArgument

@app.list_prompts()
async def list_prompts() -> list[Prompt]:
    return [
        Prompt(
            name="code_review",
            description="Review code changes for quality and best practices",
            arguments=[
                PromptArgument(
                    name="file_path",
                    description="Path to file to review",
                    required=True
                ),
                PromptArgument(
                    name="focus",
                    description="Specific aspect to focus on (security, performance, style)",
                    required=False
                )
            ]
        ),
        Prompt(
            name="debug_error",
            description="Help debug an error message",
            arguments=[
                PromptArgument(
                    name="error_message",
                    description="The error message to debug",
                    required=True
                ),
                PromptArgument(
                    name="context",
                    description="Additional context (stack trace, recent changes)",
                    required=False
                )
            ]
        )
    ]

@app.get_prompt()
async def get_prompt(name: str, arguments: dict) -> GetPromptResult:
    if name == "code_review":
        file_path = arguments["file_path"]
        focus = arguments.get("focus", "general quality")
        
        # Read file content
        content = await read_file(file_path)
        
        return GetPromptResult(
            description=f"Code review for {file_path}",
            messages=[
                PromptMessage(
                    role="user",
                    content=TextContent(
                        type="text",
                        text=f"""
Please review this code with focus on {focus}:

File: {file_path}
```
{content}
```

Provide feedback on:
1. Code quality and maintainability
2. Potential bugs or issues
3. Performance considerations
4. Best practices adherence
"""
                    )
                )
            ]
        )
    
    elif name == "debug_error":
        error_message = arguments["error_message"]
        context = arguments.get("context", "")
        
        return GetPromptResult(
            description="Debug error message",
            messages=[
                PromptMessage(
                    role="user",
                    content=TextContent(
                        type="text",
                        text=f"""
I'm getting this error:

{error_message}

{f"Context: {context}" if context else ""}

Please help me:
1. Understand what's causing this error
2. Suggest possible solutions
3. Recommend preventive measures
"""
                    )
                )
            ]
        )
    
    raise ValueError(f"Unknown prompt: {name}")
```

**Prompt Usage in Clients**:
```python
# List available prompts
prompts = await client.list_prompts()
print("Available prompts:", [p.name for p in prompts.prompts])

# Get prompt
prompt = await client.get_prompt(
    "code_review",
    {"file_path": "src/main.py", "focus": "security"}
)

# Use prompt messages with LLM
response = await llm_api_call(messages=prompt.messages)
```

**Dynamic Prompts**: Generate prompts based on context:
```python
@app.get_prompt()
async def get_prompt(name: str, arguments: dict):
    if name == "project_context":
        # Gather project information
        files = await list_project_files()
        recent_commits = await get_recent_commits()
        open_issues = await get_open_issues()
        
        context = f"""
Project Overview:
- Files: {len(files)} total
- Recent Commits: {len(recent_commits)}
- Open Issues: {len(open_issues)}

File Structure:
{format_file_tree(files)}

Recent Activity:
{format_commits(recent_commits[:5])}

Active Issues:
{format_issues(open_issues[:10])}
"""
        
        return GetPromptResult(
            description="Complete project context",
            messages=[
                PromptMessage(
                    role="user",
                    content=TextContent(type="text", text=context)
                )
            ]
        )
```

## 🌐 Transport Mechanisms

---

**Stdio Transport**: Process-based communication for local tools:
```python
# Server using stdio
from mcp.server.stdio import stdio_server

async def main():
    async with stdio_server() as (read_stream, write_stream):
        await server.run(read_stream, write_stream)

# Client connecting to stdio server
from mcp.client.stdio import stdio_client

async with stdio_client(
    command="python",
    args=["my_mcp_server.py"]
) as (read, write):
    client = Client(read, write)
    await client.initialize()
```

**HTTP with SSE**: Server-Sent Events for web services:
```python
# Server
from mcp.server.sse import sse_server
from starlette.applications import Starlette
from starlette.routing import Route

app = Starlette(routes=[
    Route("/sse", endpoint=sse_server(mcp_server))
])

# Run with uvicorn
import uvicorn
uvicorn.run(app, host="0.0.0.0", port=8000)

# Client
from mcp.client.sse import sse_client

async with sse_client("http://localhost:8000/sse") as client:
    await client.initialize()
    # Use client...
```

**Custom Transport**: Implement custom transport layer:
```python
from mcp.transport import Transport

class CustomTransport(Transport):
    async def read(self) -> bytes:
        # Read message from custom source
        pass
    
    async def write(self, data: bytes):
        # Write message to custom destination
        pass
    
    async def close(self):
        # Cleanup
        pass

# Use custom transport
transport = CustomTransport()
client = Client(transport)
await client.initialize()
```

## 🔐 Security and Authentication

---

**Environment-Based Authentication**: Pass credentials via environment variables:
```json
{
  "mcpServers": {
    "github": {
      "command": "npx",
      "args": ["-y", "@modelcontextprotocol/server-github"],
      "env": {
        "GITHUB_TOKEN": "${GITHUB_TOKEN}"
      }
    },
    "postgres": {
      "command": "docker",
      "args": ["run", "-i", "--rm", "mcp/postgres"],
      "env": {
        "DATABASE_URL": "${DATABASE_URL}"
      }
    }
  }
}
```

**Token-Based Authentication**: Server validates tokens:
```python
from mcp.server.models import InitializeRequest

@app.on_initialize()
async def handle_initialize(request: InitializeRequest):
    # Extract auth from client metadata
    auth_token = request.params.clientInfo.get("auth_token")
    
    if not validate_token(auth_token):
        raise ValueError("Invalid authentication token")
    
    # Continue initialization
    return InitializeResult(...)
```

**Capability-Based Authorization**: Limit client capabilities:
```python
@app.on_initialize()
async def handle_initialize(request: InitializeRequest):
    client_id = request.params.clientInfo.get("client_id")
    
    # Load client permissions
    permissions = load_client_permissions(client_id)
    
    # Store permissions for later checks
    app.client_permissions[client_id] = permissions
    
    return InitializeResult(
        capabilities={
            "resources": {} if permissions.can_read_resources else None,
            "tools": {} if permissions.can_use_tools else None,
        }
    )

@app.call_tool()
async def call_tool(name: str, arguments: dict):
    # Check permissions
    if not app.client_permissions[current_client_id].can_use_tool(name):
        raise PermissionError(f"Client not authorized to use tool: {name}")
    
    # Execute tool
    ...
```

**Resource-Level Authorization**:
```python
@app.read_resource()
async def read_resource(uri: str) -> str:
    # Extract client identity
    client_id = get_current_client_id()
    
    # Check if client can access this resource
    if not can_access_resource(client_id, uri):
        raise PermissionError(f"Access denied to resource: {uri}")
    
    # Return resource content
    return read_resource_content(uri)
```

**Rate Limiting**:
```python
from collections import defaultdict
from datetime import datetime, timedelta

class RateLimiter:
    def __init__(self, max_requests: int, window: timedelta):
        self.max_requests = max_requests
        self.window = window
        self.requests = defaultdict(list)
    
    def check_rate_limit(self, client_id: str) -> bool:
        now = datetime.now()
        cutoff = now - self.window
        
        # Remove old requests
        self.requests[client_id] = [
            req_time for req_time in self.requests[client_id]
            if req_time > cutoff
        ]
        
        # Check limit
        if len(self.requests[client_id]) >= self.max_requests:
            return False
        
        # Record request
        self.requests[client_id].append(now)
        return True

rate_limiter = RateLimiter(max_requests=100, window=timedelta(minutes=1))

@app.call_tool()
async def call_tool(name: str, arguments: dict):
    client_id = get_current_client_id()
    
    if not rate_limiter.check_rate_limit(client_id):
        raise ValueError("Rate limit exceeded. Try again later.")
    
    # Execute tool
    ...
```

## 🏭 Production Deployment

---

**Docker Deployment**: Package MCP server as container:
```dockerfile
FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

CMD ["python", "mcp_server.py"]
```

**Docker Compose with Multiple Servers**:
```yaml
version: '3.8'

services:
  mcp-filesystem:
    image: mcp/filesystem-server
    volumes:
      - ./data:/data:ro
    environment:
      - ALLOWED_PATHS=/data
  
  mcp-postgres:
    image: mcp/postgres-server
    environment:
      - DATABASE_URL=postgresql://user:pass@db:5432/mydb
    depends_on:
      - db
  
  db:
    image: postgres:15
    environment:
      - POSTGRES_PASSWORD=secret
```

**Kubernetes Deployment**:
```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: mcp-server
spec:
  replicas: 3
  selector:
    matchLabels:
      app: mcp-server
  template:
    metadata:
      labels:
        app: mcp-server
    spec:
      containers:
      - name: mcp-server
        image: myregistry/mcp-server:latest
        ports:
        - containerPort: 8000
        env:
        - name: DATABASE_URL
          valueFrom:
            secretKeyRef:
              name: mcp-secrets
              key: database-url
---
apiVersion: v1
kind: Service
metadata:
  name: mcp-server
spec:
  selector:
    app: mcp-server
  ports:
  - port: 80
    targetPort: 8000
  type: LoadBalancer
```

**Health Checks and Monitoring**:
```python
from prometheus_client import Counter, Histogram
import time

# Metrics
tool_calls = Counter('mcp_tool_calls_total', 'Total tool calls', ['tool_name', 'status'])
tool_duration = Histogram('mcp_tool_duration_seconds', 'Tool execution time', ['tool_name'])

@app.call_tool()
async def call_tool(name: str, arguments: dict):
    start_time = time.time()
    
    try:
        result = await execute_tool(name, arguments)
        tool_calls.labels(tool_name=name, status='success').inc()
        return result
    except Exception as e:
        tool_calls.labels(tool_name=name, status='error').inc()
        raise
    finally:
        duration = time.time() - start_time
        tool_duration.labels(tool_name=name).observe(duration)

# Health check endpoint
@app.get("/health")
async def health_check():
    # Check server health
    healthy = await check_dependencies()
    return {"status": "healthy" if healthy else "unhealthy"}
```

**Logging**:
```python
import structlog

logger = structlog.get_logger()

@app.call_tool()
async def call_tool(name: str, arguments: dict):
    logger.info(
        "tool_call_started",
        tool_name=name,
        arguments=arguments,
        client_id=get_current_client_id()
    )
    
    try:
        result = await execute_tool(name, arguments)
        logger.info(
            "tool_call_completed",
            tool_name=name,
            success=True
        )
        return result
    except Exception as e:
        logger.error(
            "tool_call_failed",
            tool_name=name,
            error=str(e),
            exc_info=True
        )
        raise
```

## 🧪 Testing MCP Servers

---

**Unit Testing**:
```python
import pytest
from mcp.server import Server
from mcp.types import TextContent

@pytest.fixture
async def mcp_server():
    server = Server("test-server")
    
    @server.list_tools()
    async def list_tools():
        return [Tool(name="test_tool", description="Test", inputSchema={})]
    
    @server.call_tool()
    async def call_tool(name: str, arguments: dict):
        if name == "test_tool":
            return [TextContent(type="text", text="success")]
    
    return server

@pytest.mark.asyncio
async def test_tool_call(mcp_server):
    result = await mcp_server.call_tool("test_tool", {})
    assert result[0].text == "success"

@pytest.mark.asyncio
async def test_list_tools(mcp_server):
    tools = await mcp_server.list_tools()
    assert len(tools) == 1
    assert tools[0].name == "test_tool"
```

**Integration Testing with Mock Client**:
```python
from mcp.client import Client
from mcp.client.stdio import stdio_client

@pytest.mark.asyncio
async def test_server_integration():
    async with stdio_client(
        command="python",
        args=["test_mcp_server.py"]
    ) as (read, write):
        client = Client(read, write)
        await client.initialize()
        
        # Test resource listing
        resources = await client.list_resources()
        assert len(resources.resources) > 0
        
        # Test resource reading
        content = await client.read_resource(resources.resources[0].uri)
        assert content.contents[0].text is not None
        
        # Test tool execution
        tools = await client.list_tools()
        result = await client.call_tool(tools.tools[0].name, {})
        assert result.content[0].type == "text"
```

**Mock Server for Client Testing**:
```python
class MockMCPServer:
    def __init__(self):
        self.resources = [
            Resource(uri="file:///test.txt", name="test", mimeType="text/plain")
        ]
        self.tools = [
            Tool(name="mock_tool", description="Mock", inputSchema={})
        ]
    
    async def list_resources(self):
        return {"resources": self.resources}
    
    async def read_resource(self, uri: str):
        return {"contents": [{"type": "text", "text": "mock content"}]}
    
    async def list_tools(self):
        return {"tools": self.tools}
    
    async def call_tool(self, name: str, arguments: dict):
        return {"content": [{"type": "text", "text": "mock result"}]}

@pytest.fixture
def mock_server():
    return MockMCPServer()

@pytest.mark.asyncio
async def test_client_with_mock_server(mock_server):
    # Test client logic with predictable mock responses
    result = await mock_server.call_tool("mock_tool", {})
    assert result["content"][0]["text"] == "mock result"
```

## 🔗 Related Concepts

---

- [[LLM]] (Large Language Models)
- [[Claude]]
- [[Anthropic]]
- [[OpenAI]]
- [[Function Calling]]
- [[Tool Use]]
- [[API Design]]
- [[JSON-RPC]]
- [[REST API]]
- [[GraphQL]]
- [[Protocol Buffers]]
- [[WebSocket]]
- [[Server-Sent Events]]
- [[Stdio]]
- [[RAG]] (Retrieval-Augmented Generation)
- [[Embeddings]]
- [[Vector Databases]]
- [[Semantic Search]]
- [[LangChain]]
- [[Semantic Kernel]]
- [[AutoGen]]
- [[AI Agents]]
- [[Prompt Engineering]]
- [[Microservices]]
- [[Service-Oriented Architecture]]
- [[Ash AI]]

## 💡 Advanced Use Cases

---

**Multi-Tenant MCP Server**: Serve multiple clients with isolated data:
```python
class MultiTenantMCPServer:
    def __init__(self):
        self.tenant_data = {}
    
    def get_tenant_id(self) -> str:
        # Extract from auth context
        return current_request.client_id
    
    @app.list_resources()
    async def list_resources(self):
        tenant_id = self.get_tenant_id()
        
        # Return tenant-specific resources
        return [
            Resource(
                uri=f"tenant://{tenant_id}/documents/{doc.id}",
                name=doc.name,
                mimeType="text/plain"
            )
            for doc in self.tenant_data.get(tenant_id, {}).get("documents", [])
        ]
    
    @app.read_resource()
    async def read_resource(self, uri: str):
        tenant_id = self.get_tenant_id()
        
        # Validate tenant access
        if not uri.startswith(f"tenant://{tenant_id}/"):
            raise PermissionError("Access denied")
        
        # Return tenant data
        return get_tenant_resource_content(tenant_id, uri)
```

**MCP Server Gateway**: Proxy multiple MCP servers:
```python
class MCPGateway:
    def __init__(self, backend_servers: dict[str, str]):
        self.backends = {}
        for name, connection_string in backend_servers.items():
            self.backends[name] = connect_to_mcp_server(connection_string)
    
    @app.list_resources()
    async def list_resources(self):
        # Aggregate resources from all backends
        all_resources = []
        for name, backend in self.backends.items():
            resources = await backend.list_resources()
            # Prefix URIs with backend name
            for resource in resources.resources:
                resource.uri = f"{name}://{resource.uri}"
            all_resources.extend(resources.resources)
        return all_resources
    
    @app.read_resource()
    async def read_resource(self, uri: str):
        # Route to appropriate backend
        backend_name, backend_uri = uri.split("://", 1)
        backend = self.backends[backend_name]
        return await backend.read_resource(backend_uri)
```

**Caching Layer**: Cache expensive operations:
```python
from functools import lru_cache
import hashlib

class CachingMCPServer:
    def __init__(self):
        self.resource_cache = {}
        self.cache_ttl = 300  # 5 minutes
    
    @app.read_resource()
    async def read_resource(self, uri: str):
        cache_key = hashlib.sha256(uri.encode()).hexdigest()
        
        # Check cache
        if cache_key in self.resource_cache:
            cached_data, cached_time = self.resource_cache[cache_key]
            if time.time() - cached_time < self.cache_ttl:
                return cached_data
        
        # Fetch and cache
        data = await fetch_resource_content(uri)
        self.resource_cache[cache_key] = (data, time.time())
        return data
    
    @app.on_resource_updated()
    async def handle_resource_update(self, uri: str):
        # Invalidate cache
        cache_key = hashlib.sha256(uri.encode()).hexdigest()
        self.resource_cache.pop(cache_key, None)
```

**Workflow Orchestration**: Chain tools into workflows:
```python
@app.call_tool()
async def call_tool(name: str, arguments: dict):
    if name == "analyze_repository":
        # Multi-step workflow
        
        # Step 1: Clone repository
        clone_result = await call_tool("git_clone", {
            "url": arguments["repo_url"],
            "path": "/tmp/repo"
        })
        
        # Step 2: Analyze code
        analysis_result = await call_tool("code_analysis", {
            "path": "/tmp/repo",
            "language": arguments.get("language", "auto")
        })
        
        # Step 3: Generate report
        report = await call_tool("generate_report", {
            "analysis": analysis_result,
            "format": "markdown"
        })
        
        # Step 4: Cleanup
        await call_tool("cleanup", {"path": "/tmp/repo"})
        
        return [TextContent(type="text", text=report)]
```

**Streaming Large Results**: Stream data for large responses:
```python
@app.call_tool()
async def call_tool(name: str, arguments: dict):
    if name == "stream_logs":
        # Return streaming response
        async def stream_log_lines():
            with open(arguments["log_file"]) as f:
                for line in f:
                    yield TextContent(type="text", text=line)
                    # Optional: throttle streaming
                    await asyncio.sleep(0.01)
        
        return stream_log_lines()
```

## 📚 Further Reading

---

- Model Context Protocol specification: https://spec.modelcontextprotocol.io
- MCP GitHub repository: https://github.com/modelcontextprotocol
- Official MCP documentation and tutorials
- Anthropic's MCP announcement and rationale
- JSON-RPC 2.0 specification
- "Building AI Agents with MCP" guides
- Server-Sent Events (SSE) specification
- WebSocket protocol documentation
- Python `asyncio` documentation for async programming
- TypeScript async patterns for Node.js servers
- Blog posts on MCP server implementations
- Community MCP servers and examples
- Security best practices for API protocols
- Microservices architecture patterns
