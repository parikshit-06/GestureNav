import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("🔌 Connecting to drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Drone connected")
            break

    print("⏳ Waiting for drone to be ready...")
    async for health in drone.telemetry.health():
        if health.is_home_position_ok:
            print("✅ Drone is ready")
            break

    print("🔐 Arming drone...")
    await drone.action.arm()

    # 1. Send dummy setpoints
    print("📡 Sending initial dummy setpoints...")
    for _ in range(20):  # 2 seconds worth
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.1)

    # 2. Start Offboard
    print("🚁 Starting Offboard mode...")
    try:
        await drone.offboard.start()
        print("✅ Offboard started successfully")
    except OffboardError as error:
        print(f"❌ Offboard start failed: {error._result.result_str}")
        return

    # 3. Send yaw rotation
    print("🌀 Sending yaw rotation (45°/s)...")
    for _ in range(100):  # 10 seconds
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 45.0))
        await asyncio.sleep(0.1)

    # 4. Stop and disarm
    print("🛑 Stopping Offboard and disarming...")
    await drone.offboard.stop()
    await drone.action.disarm()

    print("✅ Done")

if __name__ == "__main__":
    asyncio.run(run())
