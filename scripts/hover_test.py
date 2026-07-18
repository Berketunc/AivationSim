#!/usr/bin/env python3
"""
Quick takeoff-and-hover script for isolation testing.
Connects via MAVSDK (sends GCS heartbeat), arms, takes off to 5 m, then hovers.
Run this while aruco_detector_node is running to verify pose output.
Press Ctrl-C to land.
"""
import asyncio
from mavsdk import System


async def main():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for connection…")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to PX4.")
            break

    print("Waiting for GPS fix and home position…")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("GPS OK, home set.")
            break

    print("Arming…")
    await drone.action.arm()

    print("Taking off to 5 m…")
    await drone.action.set_takeoff_altitude(5.0)
    await drone.action.takeoff()

    # Wait until at altitude
    async for pos in drone.telemetry.position():
        print(f"  AGL: {pos.relative_altitude_m:.1f} m", end="\r")
        if pos.relative_altitude_m >= 4.5:
            print("\nAt altitude. Hovering — check /target_pose and /is_visible now.")
            print("Press Ctrl-C to land.")
            break

    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        pass

    print("\nLanding…")
    await drone.action.land()
    async for state in drone.telemetry.landed_state():
        from mavsdk.telemetry import LandedState
        if state == LandedState.ON_GROUND:
            print("Landed.")
            break


if __name__ == "__main__":
    asyncio.run(main())
