"""A thread for MoveItPy's execute(), so the event loop keeps serving requests while the arm moves."""

import asyncio
import functools
import threading
from concurrent.futures import ThreadPoolExecutor

# Only execute(): planning-scene reads on a second thread were slow on the robot, so the rest stays on the loop
_worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="moveit-execute")

STOPPED = "stopped by stop_arm"
LATCHED_TEXT = "Error: the arm is stopped (stop_arm); call resume_arm first."

# Set by stop_arm, cleared by resume_arm: nothing new plans or moves while it is set
stop_latched = threading.Event()
# Set while execute() runs; _gate makes "check the latch, then mark executing" one step for latch()
executing = threading.Event()
_gate = threading.Lock()


def latch() -> None:
    """Set the stop latch; afterwards every execute() either shows in `executing` or refuses to start."""
    with _gate:
        stop_latched.set()


def _unless_latched(fn, *args, **kwargs):
    with _gate:
        if stop_latched.is_set():
            raise RuntimeError(STOPPED)
        executing.set()
    try:
        return fn(*args, **kwargs)
    finally:
        executing.clear()


async def on_execute_thread(fn, *args, **kwargs):
    """Run a blocking MoveItPy call on the execute thread and wait for it without blocking the loop."""
    return await asyncio.get_running_loop().run_in_executor(
        _worker, functools.partial(_unless_latched, fn, *args, **kwargs))
