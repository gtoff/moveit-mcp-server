"""Logging for errors in tool handlers."""

import logging


def log_tool_error(logger: logging.Logger, tool: str, e: Exception) -> None:
    """Refusals (bad input) as one warning line; anything else as an error with the traceback."""
    if isinstance(e, (ValueError, KeyError)):
        logger.warning(f"{tool} refused: {e}")
    else:
        logger.error(f"Error in {tool}: {e}", exc_info=True)
