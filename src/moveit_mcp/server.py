"""Main MCP server for MoveIt2."""

import argparse
import asyncio
import logging
import os
import sys
from pathlib import Path
from typing import Optional

import yaml

try:
    from mcp.server import Server
    from mcp.server.stdio import stdio_server
    from mcp.server.sse import SseServerTransport
    from mcp.server.streamable_http_manager import StreamableHTTPSessionManager
    from starlette.applications import Starlette
    from starlette.routing import Route, Mount
    from starlette.responses import Response
except ImportError as e:
    print(f"Error: Required packages not installed: {e}", file=sys.stderr)
    print("Install with: pip install mcp starlette uvicorn", file=sys.stderr)
    sys.exit(1)

from .async_ops import AsyncOperationManager
from .moveit_wrapper import MoveItWrapper
from .resources import register_resources
from .prompts import register_prompts
from .tools.registry import ToolRegistry
from .tools.planning import get_planning_tools
from .tools.execution import get_execution_tools
from .tools.scene import get_scene_tools
from .tools.queries import get_query_tools


logger = logging.getLogger(__name__)


class MoveItMCPServer:
    """MCP Server for MoveIt2 motion planning."""

    def __init__(self, config_path: Optional[str] = None):
        """
        Initialize the MoveIt MCP server.

        Args:
            config_path: Path to configuration file
        """
        # Load configuration
        self.config = self._load_config(config_path)

        # Setup logging
        self._setup_logging()

        # Initialize components
        self.server = Server("moveit-mcp-server")
        self.moveit: Optional[MoveItWrapper] = None
        self.op_manager: Optional[AsyncOperationManager] = None
        self._initialized = False

        logger.info("MoveItMCPServer initialized")

    def _load_config(self, config_path: Optional[str]) -> dict:
        """Load configuration from file."""
        if config_path is None:
            # Look for config in standard locations
            search_paths = [
                Path.cwd() / "config" / "panda_mcp_server.yaml",
                Path.home() / ".config" / "moveit_mcp" / "config.yaml",
                Path(__file__).parent.parent.parent / "config" / "panda_mcp_server.yaml",
            ]

            for path in search_paths:
                if path.exists():
                    config_path = str(path)
                    break

        if config_path and Path(config_path).exists():
            logger.info(f"Loading configuration from {config_path}")
            with open(config_path, "r") as f:
                return yaml.safe_load(f)
        else:
            logger.warning("No configuration file found, using defaults")
            return self._default_config()

    def _default_config(self) -> dict:
        """Return default configuration."""
        return {
            "robot": {"name": "panda", "description_package": "moveit_resources_panda_moveit_config"},
            "planning": {
                "default_planner": "RRTConnect",
                "default_timeout": 5.0,
                "planning_attempts": 10,
                "planning_groups": ["panda_arm", "panda_hand"],
            },
            "execution": {
                "controllers": ["panda_arm_controller", "hand_controller"],
                "execution_timeout": 30.0,
            },
            "mcp_server": {
                "operation_timeout": 60.0,
                "max_concurrent_operations": 10,
                "resource_update_rate": 10.0,
                "node_name": "moveit_mcp_server",
            },
            "logging": {"level": "INFO", "format": "%(asctime)s - %(name)s - %(levelname)s - %(message)s"},
        }

    def _setup_logging(self):
        """Setup logging configuration."""
        log_config = self.config.get("logging", {})
        level = getattr(logging, log_config.get("level", "INFO"))
        format_str = log_config.get("format", "%(asctime)s - %(name)s - %(levelname)s - %(message)s")

        logging.basicConfig(level=level, format=format_str, stream=sys.stderr)

    def is_initialized(self) -> bool:
        """Check if the server is fully initialized and ready to handle requests."""
        return self._initialized

    async def start_with_wrapper(self, wrapper):
        """Start the MCP server using an existing MoveItWrapper instance."""
        self.moveit = wrapper
        await self._start_services()

    async def start(self):
        """Start the MCP server and MoveIt components."""
        try:
            logger.info("Starting MoveIt MCP Server...")

            # Initialize MoveIt wrapper
            logger.info("Initializing MoveIt...")
            self.moveit = MoveItWrapper(self.config)
            await self._start_services()
        except Exception as e:
            logger.error(f"Failed to start server: {e}", exc_info=True)
            raise

    async def _start_services(self):
        """Initialize operation manager, register tools, resources, and prompts."""
        # Initialize async operation manager
        mcp_config = self.config.get("mcp_server", {})
        self.op_manager = AsyncOperationManager(
            max_concurrent_operations=mcp_config.get("max_concurrent_operations", 10),
            operation_timeout=mcp_config.get("operation_timeout", 60.0),
        )
        await self.op_manager.start()

        # Register tools using unified registry
        logger.info("Registering MCP tools...")
        registry = ToolRegistry()

        # Get tools from each module
        planning_tools, planning_handlers = get_planning_tools(self.moveit, self.op_manager, server_instance=self)
        execution_tools, execution_handlers = get_execution_tools(self.moveit, self.op_manager, server_instance=self)
        scene_tools, scene_handlers = get_scene_tools(self.moveit, server_instance=self)
        query_tools, query_handlers = get_query_tools(self.moveit, server_instance=self)

        # Register all tools
        registry.register_tools(planning_tools, planning_handlers)
        registry.register_tools(execution_tools, execution_handlers)
        registry.register_tools(scene_tools, scene_handlers)
        registry.register_tools(query_tools, query_handlers)

        # Setup unified server handlers
        registry.setup_server_handlers(self.server)

        # Register resources
        logger.info("Registering MCP resources...")
        register_resources(self.server, self.moveit, self.op_manager)

        # Register prompts
        logger.info("Registering MCP prompts...")
        register_prompts(self.server)

        # Mark as initialized
        self._initialized = True

        logger.info("MoveIt MCP Server started successfully")

    async def stop(self):
        """Stop the MCP server and cleanup."""
        logger.info("Stopping MoveIt MCP Server...")

        if self.op_manager:
            await self.op_manager.stop()

        if self.moveit:
            self.moveit.shutdown()

        logger.info("MoveIt MCP Server stopped")

    async def run(self, transport: str = "stdio", host: str = "0.0.0.0", port: int = 8000):
        """
        Run the MCP server.

        Args:
            transport: Transport type - "stdio", "sse", or "http" (default: "stdio")
            host: Host to bind to when using SSE transport (default: "0.0.0.0")
            port: Port to bind to when using SSE transport (default: 8000)
        """
        try:
            await self.start()

            if transport == "stdio":
                # Run the server with stdio transport (for local execution)
                logger.info("Starting MCP server with stdio transport")
                async with stdio_server() as (read_stream, write_stream):
                    await self.server.run(
                        read_stream,
                        write_stream,
                        self.server.create_initialization_options(),
                    )

            elif transport == "sse":
                # Run the server with SSE/HTTP transport (for remote/container execution)
                logger.info(f"Starting MCP server with SSE transport on {host}:{port}")

                # Create SSE transport
                sse = SseServerTransport("/messages")

                # Create Starlette app with SSE endpoint
                async def handle_root(request):
                    """Handle root endpoint - health check and info"""
                    from starlette.responses import JSONResponse
                    return JSONResponse({
                        "name": "moveit-mcp-server",
                        "version": "0.1.0",
                        "status": "ok",
                        "transport": "sse",
                        "capabilities": {
                            "tools": True,
                            "resources": True,
                            "prompts": True
                        },
                        "endpoints": {
                            "sse": "/sse",
                            "messages": "/messages"
                        }
                    }, status_code=200)

                # Create raw ASGI callable endpoints
                async def sse_endpoint(scope, receive, send):
                    """ASGI app for SSE connections"""
                    async with sse.connect_sse(scope, receive, send) as streams:
                        await self.server.run(
                            streams[0],
                            streams[1],
                            self.server.create_initialization_options(),
                        )

                async def messages_endpoint(scope, receive, send):
                    """ASGI app for message posting"""
                    await sse.handle_post_message(scope, receive, send)

                # Use Starlette routes with raw ASGI apps
                from starlette.responses import Response
                from starlette.types import ASGIApp

                # Wrapper class to make ASGI callables work with Starlette Route
                class ASGIWrapper:
                    def __init__(self, asgi_app):
                        self.asgi_app = asgi_app

                    async def __call__(self, scope_or_request, receive=None, send=None):
                        # Handle both ASGI (scope, receive, send) and endpoint (request) signatures
                        if receive is None and send is None:
                            # Called as endpoint: __call__(request)
                            request = scope_or_request
                            await self.asgi_app(request.scope, request.receive, request._send)
                            # Return a dummy response that does nothing
                            class AlreadySentResponse(Response):
                                async def __call__(self, scope, receive, send):
                                    pass  # Response already sent
                            return AlreadySentResponse()
                        else:
                            # Called as ASGI app: __call__(scope, receive, send)
                            await self.asgi_app(scope_or_request, receive, send)

                app = Starlette(
                    routes=[
                        Route("/", endpoint=handle_root, methods=["GET"]),
                        Route("/sse", endpoint=ASGIWrapper(sse_endpoint), methods=["GET"]),
                        Route("/messages", endpoint=ASGIWrapper(messages_endpoint), methods=["POST"]),
                    ],
                )

                # Run with uvicorn
                import uvicorn
                config = uvicorn.Config(
                    app,
                    host=host,
                    port=port,
                    log_level="info",
                    timeout_graceful_shutdown=5  # Force shutdown after 5 seconds
                )
                server = uvicorn.Server(config)
                await server.serve()

            elif transport == "http":
                # Run the server with Streamable HTTP transport (MCP spec 2025-03-26)
                logger.info(f"Starting MCP server with Streamable HTTP transport on {host}:{port}")

                import contextlib
                from starlette.responses import JSONResponse

                session_manager = StreamableHTTPSessionManager(
                    app=self.server,
                    event_store=None,
                    json_response=False,
                    stateless=False,
                )

                async def handle_root(request):
                    return JSONResponse({
                        "name": "moveit-mcp-server",
                        "version": "0.1.0",
                        "status": "ok",
                        "transport": "http",
                        "capabilities": {
                            "tools": True,
                            "resources": True,
                            "prompts": True
                        },
                        "endpoints": {
                            "mcp": "/mcp"
                        }
                    }, status_code=200)

                async def handle_mcp(scope, receive, send):
                    await session_manager.handle_request(scope, receive, send)

                class ASGIWrapper:
                    def __init__(self, asgi_app):
                        self.asgi_app = asgi_app

                    async def __call__(self, scope_or_request, receive=None, send=None):
                        if receive is None and send is None:
                            request = scope_or_request
                            await self.asgi_app(request.scope, request.receive, request._send)
                            class AlreadySentResponse(Response):
                                async def __call__(self, scope, receive, send):
                                    pass
                            return AlreadySentResponse()
                        else:
                            await self.asgi_app(scope_or_request, receive, send)

                @contextlib.asynccontextmanager
                async def lifespan(app):
                    async with session_manager.run():
                        yield

                app = Starlette(
                    lifespan=lifespan,
                    routes=[
                        Route("/", endpoint=handle_root, methods=["GET"]),
                        Route("/mcp", endpoint=ASGIWrapper(handle_mcp), methods=["GET", "POST", "DELETE"]),
                    ],
                )

                import uvicorn
                config = uvicorn.Config(
                    app,
                    host=host,
                    port=port,
                    log_level="info",
                    timeout_graceful_shutdown=5,
                )
                server = uvicorn.Server(config)
                await server.serve()

            else:
                raise ValueError(f"Unknown transport type: {transport}")

        except KeyboardInterrupt:
            logger.info("Received interrupt signal")
        except Exception as e:
            logger.error(f"Server error: {e}", exc_info=True)
            raise
        finally:
            await self.stop()


def apply_overrides(config: dict, use_sim_time: Optional[bool] = None) -> None:
    """Command-line values over the config file."""
    if use_sim_time is not None:
        config.setdefault("mcp_server", {})["use_sim_time"] = use_sim_time


def main():
    """Main entry point for the MCP server."""
    parser = argparse.ArgumentParser(description="MoveIt2 MCP Server")
    parser.add_argument(
        "--config",
        "-c",
        type=str,
        default=None,
        help="Path to configuration file (default: search standard locations)",
    )
    parser.add_argument(
        "--log-level",
        type=str,
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        default=None,
        help="Override log level from config",
    )
    parser.add_argument(
        "--transport",
        "-t",
        type=str,
        choices=["stdio", "sse", "http"],
        default="stdio",
        help="Transport type: stdio (local), sse (legacy HTTP), or http (Streamable HTTP, MCP 2025-03-26) (default: stdio)",
    )
    parser.add_argument(
        "--host",
        type=str,
        default="0.0.0.0",
        help="Host to bind to for SSE transport (default: 0.0.0.0)",
    )
    parser.add_argument(
        "--port",
        "-p",
        type=int,
        default=8000,
        help="Port to bind to for SSE transport (default: 8000)",
    )
    parser.add_argument(
        "--use-sim-time",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Override mcp_server.use_sim_time from the config (--no-use-sim-time on a real robot)",
    )

    args = parser.parse_args()

    # Create and run server
    server = MoveItMCPServer(config_path=args.config)
    apply_overrides(server.config, use_sim_time=args.use_sim_time)

    # Override log level if specified
    if args.log_level:
        logging.getLogger().setLevel(getattr(logging, args.log_level))

    # Run the server
    try:
        asyncio.run(server.run(transport=args.transport, host=args.host, port=args.port))
    except KeyboardInterrupt:
        logger.info("Shutdown complete")
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
