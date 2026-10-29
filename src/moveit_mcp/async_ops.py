"""Async operation manager for tracking long-running MoveIt operations."""

import asyncio
import logging
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, Optional


logger = logging.getLogger(__name__)


class OperationStatus(Enum):
    """Status of an async operation."""
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class OperationType(Enum):
    """Type of operation being performed."""
    PLANNING = "planning"
    EXECUTION = "execution"
    COMBINED = "combined"
    QUERY = "query"


@dataclass
class OperationInfo:
    """Information about an async operation."""
    operation_id: str
    operation_type: OperationType
    status: OperationStatus
    created_at: float
    updated_at: float
    progress: float = 0.0
    result: Optional[Any] = None
    error: Optional[str] = None
    metadata: Dict[str, Any] = field(default_factory=dict)


class AsyncOperationManager:
    """Manages async operations for MoveIt planning and execution."""

    def __init__(self, max_concurrent_operations: int = 10, operation_timeout: float = 60.0):
        """
        Initialize the operation manager.

        Args:
            max_concurrent_operations: Maximum number of concurrent operations
            operation_timeout: Timeout in seconds for operations before cleanup
        """
        self.max_concurrent_operations = max_concurrent_operations
        self.operation_timeout = operation_timeout
        self.operations: Dict[str, tuple[asyncio.Task, OperationInfo]] = {}
        self._cleanup_task: Optional[asyncio.Task] = None
        self._lock = asyncio.Lock()

    async def start(self):
        """Start the operation manager and cleanup task."""
        if self._cleanup_task is None:
            self._cleanup_task = asyncio.create_task(self._cleanup_loop())
            logger.info("AsyncOperationManager started")

    async def stop(self):
        """Stop the operation manager and cancel all operations."""
        if self._cleanup_task:
            self._cleanup_task.cancel()
            try:
                await self._cleanup_task
            except asyncio.CancelledError:
                pass

        # Cancel all running operations (inline to avoid lock reentrance)
        async with self._lock:
            for operation_id, (task, info) in list(self.operations.items()):
                if info.status not in [OperationStatus.COMPLETED, OperationStatus.FAILED]:
                    task.cancel()

        logger.info("AsyncOperationManager stopped")

    def generate_operation_id(self) -> str:
        """Generate a unique operation ID."""
        return str(uuid.uuid4())

    async def submit_operation(
        self,
        operation_fn: Callable,
        operation_type: OperationType,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> str:
        """
        Submit a new async operation.

        Args:
            operation_fn: Async function to execute
            operation_type: Type of operation
            metadata: Optional metadata about the operation

        Returns:
            Operation ID

        Raises:
            RuntimeError: If max concurrent operations exceeded
        """
        async with self._lock:
            # Only unfinished operations count; finished ones are kept for operation_timeout
            active = sum(info.status in (OperationStatus.PENDING, OperationStatus.RUNNING)
                         for _, info in self.operations.values())
            if active >= self.max_concurrent_operations:
                raise RuntimeError(
                    f"Maximum concurrent operations ({self.max_concurrent_operations}) exceeded"
                )

            operation_id = self.generate_operation_id()
            current_time = time.time()

            info = OperationInfo(
                operation_id=operation_id,
                operation_type=operation_type,
                status=OperationStatus.PENDING,
                created_at=current_time,
                updated_at=current_time,
                metadata=metadata or {},
            )

            # Create the task
            task = asyncio.create_task(self._run_operation(operation_id, operation_fn))
            self.operations[operation_id] = (task, info)

            logger.info(f"Operation {operation_id} submitted: {operation_type.value}")
            return operation_id

    async def _run_operation(self, operation_id: str, operation_fn: Callable):
        """Run an operation and update its status."""
        _, info = self.operations[operation_id]

        try:
            info.status = OperationStatus.RUNNING
            info.updated_at = time.time()
            logger.debug(f"Operation {operation_id} started")

            result = await operation_fn()

            info.status = OperationStatus.COMPLETED
            info.progress = 1.0
            info.result = result
            info.updated_at = time.time()
            logger.info(f"Operation {operation_id} completed successfully")

        except asyncio.CancelledError:
            info.status = OperationStatus.CANCELLED
            info.error = "Operation was cancelled"
            info.updated_at = time.time()
            logger.info(f"Operation {operation_id} cancelled")
            raise

        except Exception as e:
            info.status = OperationStatus.FAILED
            info.error = str(e)
            info.updated_at = time.time()
            logger.error(f"Operation {operation_id} failed: {e}", exc_info=True)

    async def get_operation_status(self, operation_id: str) -> Optional[Dict[str, Any]]:
        """
        Get the status of an operation.

        Args:
            operation_id: Operation ID

        Returns:
            Operation status dict or None if not found
        """
        async with self._lock:
            if operation_id not in self.operations:
                return None

            _, info = self.operations[operation_id]

            return {
                "operation_id": info.operation_id,
                "type": info.operation_type.value,
                "status": info.status.value,
                "progress": info.progress,
                "created_at": info.created_at,
                "updated_at": info.updated_at,
                "result": info.result,
                "error": info.error,
                "metadata": info.metadata,
            }

    async def wait_for_operation(
        self, operation_id: str, timeout: Optional[float] = None
    ) -> Optional[Dict[str, Any]]:
        """
        Wait for an operation to complete.

        Args:
            operation_id: Operation ID
            timeout: Optional timeout in seconds

        Returns:
            Operation result or None if not found
        """
        if operation_id not in self.operations:
            return None

        task, _ = self.operations[operation_id]

        try:
            await asyncio.wait_for(task, timeout=timeout)
        except asyncio.TimeoutError:
            logger.warning(f"Operation {operation_id} timed out after {timeout}s")
        except asyncio.CancelledError:
            pass

        return await self.get_operation_status(operation_id)

    async def cancel_operation(self, operation_id: str) -> bool:
        """
        Cancel an operation.

        Args:
            operation_id: Operation ID

        Returns:
            True if cancelled, False if not found
        """
        async with self._lock:
            if operation_id not in self.operations:
                return False

            task, info = self.operations[operation_id]

            if info.status in [OperationStatus.COMPLETED, OperationStatus.FAILED]:
                logger.debug(f"Operation {operation_id} already finished")
                return False

            task.cancel()
            logger.info(f"Operation {operation_id} cancellation requested")
            return True

    async def running_execution(self) -> Optional[str]:
        """ID of an unfinished execution or plan-and-execute; the arm runs one motion at a time."""
        async with self._lock:
            for operation_id, (_, info) in self.operations.items():
                if (info.operation_type in (OperationType.EXECUTION, OperationType.COMBINED)
                        and info.status in (OperationStatus.PENDING, OperationStatus.RUNNING)):
                    return operation_id
        return None

    async def list_operations(self) -> list[Dict[str, Any]]:
        """
        List all operations.

        Returns:
            List of operation status dicts
        """
        async with self._lock:
            result = []
            for operation_id, (_, info) in self.operations.items():
                result.append({
                    "operation_id": info.operation_id,
                    "type": info.operation_type.value,
                    "status": info.status.value,
                    "progress": info.progress,
                    "created_at": info.created_at,
                    "updated_at": info.updated_at,
                    "result": info.result,
                    "error": info.error,
                    "metadata": info.metadata,
                })
            return result

    def not_found_text(self, operation_id: str) -> str:
        """Answer for an unknown operation ID; finished operations are dropped after operation_timeout."""
        return (f"Operation {operation_id} not found: unknown, or removed "
                f"{self.operation_timeout:.0f} s after it finished.")

    async def _cleanup_loop(self):
        """Periodically cleanup completed operations."""
        while True:
            try:
                await asyncio.sleep(10.0)  # Check every 10 seconds
                await self._cleanup_old_operations()
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in cleanup loop: {e}", exc_info=True)

    async def _cleanup_old_operations(self):
        """Remove old completed/failed operations."""
        current_time = time.time()
        async with self._lock:
            to_remove = []

            for operation_id, (task, info) in self.operations.items():
                # Remove if completed/failed and past timeout
                if info.status in [
                    OperationStatus.COMPLETED,
                    OperationStatus.FAILED,
                    OperationStatus.CANCELLED,
                ]:
                    age = current_time - info.updated_at
                    if age > self.operation_timeout:
                        to_remove.append(operation_id)
                        logger.debug(f"Cleaning up operation {operation_id} (age: {age:.1f}s)")

            for operation_id in to_remove:
                del self.operations[operation_id]

            if to_remove:
                logger.info(f"Cleaned up {len(to_remove)} old operations")
