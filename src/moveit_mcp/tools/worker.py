"""A thread for MoveItPy's execute(), so the event loop keeps serving requests while the arm moves."""

import asyncio
import functools
from concurrent.futures import ThreadPoolExecutor

# Only execute(): planning-scene reads on a second thread were slow on the robot, so the rest stays on the loop
_worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="moveit-execute")


async def on_execute_thread(fn, *args, **kwargs):
    """Run a blocking MoveItPy call on the execute thread and wait for it without blocking the loop."""
    return await asyncio.get_running_loop().run_in_executor(_worker, functools.partial(fn, *args, **kwargs))
