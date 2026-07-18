"""
MAVSDK asyncio bridge for the ROS 2 landing controller.

The asyncio event loop runs on a dedicated background thread.
rclpy spins on the main thread and calls bridge methods thread-safely.

Two cross-thread entry points:
  - bridge.run(coro)          — submit coroutine, block caller until done
  - bridge.send_velocity_body — non-blocking, for the hot 20 Hz control path
"""

import asyncio
import threading

from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed


class MavsdkBridge:

    def __init__(self, system_address: str = 'udpin://:14540'):
        self.system_address = system_address
        self._drone = System()
        self._offboard_active = False

        self.loop = asyncio.new_event_loop()
        self.setpoint_queue: asyncio.Queue | None = None

        self._loop_ready = threading.Event()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
        self._loop_ready.wait()  # block until the loop is running

    # ── loop bootstrap ────────────────────────────────────────────────────────

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.setpoint_queue = asyncio.Queue()
        self._loop_ready.set()
        self.loop.run_forever()

    # ── cross-thread dispatch ─────────────────────────────────────────────────

    def run(self, coro, timeout: float | None = None):
        """Submit coro to asyncio loop and block the calling thread until done."""
        future = asyncio.run_coroutine_threadsafe(coro, self.loop)
        return future.result(timeout=timeout)

    def submit(self, coro):
        """Schedule coro without waiting (fire-and-forget)."""
        asyncio.run_coroutine_threadsafe(coro, self.loop)

    # ── MAVSDK coroutines (execute on asyncio thread) ─────────────────────────

    async def connect(self):
        await self._drone.connect(system_address=self.system_address)
        async for state in self._drone.core.connection_state():
            if state.is_connected:
                break

    async def arm_and_takeoff(self, altitude_m: float):
        await self._drone.action.arm()
        await asyncio.sleep(1.0)
        await self._drone.action.takeoff()
        async for pos in self._drone.telemetry.position():
            if pos.relative_altitude_m >= altitude_m - 0.5:
                return

    async def _offboard_send_loop(self):
        """
        Continuous 20 Hz velocity setpoint loop.
        PX4 drops Offboard mode if setpoints stop arriving above 2 Hz.
        Never let this coroutine stall or block — use get_nowait, not get().
        """
        self._offboard_active = True
        current = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)

        # MAVSDK requires one setpoint before offboard.start() or it raises NO_SETPOINT_SET
        await self._drone.offboard.set_velocity_body(current)
        await self._drone.offboard.start()

        while self._offboard_active:
            try:
                current = self.setpoint_queue.get_nowait()
            except asyncio.QueueEmpty:
                pass
            try:
                await self._drone.offboard.set_velocity_body(current)
            except Exception:
                pass
            await asyncio.sleep(0.05)  # 20 Hz

    async def land(self):
        """Stop offboard loop, exit Offboard mode, trigger Action.land()."""
        self._offboard_active = False
        await asyncio.sleep(0.2)  # give the loop time to see the flag
        try:
            await self._drone.offboard.stop()
        except Exception:
            pass
        await self._drone.action.land()

    # ── hot path ──────────────────────────────────────────────────────────────

    def start_offboard_loop(self):
        """Fire-and-forget: start the 20 Hz offboard velocity send loop."""
        self.submit(self._offboard_send_loop())

    def send_velocity_body(
        self,
        vx: float,
        vy: float,
        vz: float,
        yawspeed: float = 0.0,
    ):
        """
        Thread-safe, non-blocking.  Call from any thread (rclpy callback, timer, etc.).
        Puts the setpoint into the asyncio queue; the send loop picks it up at 20 Hz.
        """
        self.loop.call_soon_threadsafe(
            self.setpoint_queue.put_nowait,
            VelocityBodyYawspeed(vx, vy, vz, yawspeed),
        )
