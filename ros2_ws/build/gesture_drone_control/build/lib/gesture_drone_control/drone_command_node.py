import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

class DroneCommandNode(Node):
    def __init__(self):
        super().__init__('drone_command_node')
        self.subscription = self.create_subscription(
            String,
            'gesture_command',
            self.listener_callback,
            10
        )
        self.command = None
        self.drone = System()
        self.offboard_started = False
        self.loop = asyncio.get_event_loop()
        self.loop.create_task(self.run_drone())

    async def run_drone(self):
        print("🔌 Connecting to drone...")
        await self.drone.connect(system_address="udp://:14540")

        print("⏳ Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("✅ Drone connected")
                break

        print("⏳ Waiting for drone to be ready...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok or health.is_local_position_ok:
                print("✅ Drone is ready")
                break

        print("🔐 Arming drone...")
        await self.drone.action.arm()

        print("📡 Sending initial dummy setpoints...")
        for _ in range(20):
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
            )
            await asyncio.sleep(0.1)

        print("🚁 Starting Offboard mode...")
        try:
            await self.drone.offboard.start()
            self.offboard_started = True
            print("✅ Offboard mode started!")
        except OffboardError as error:
            print(f"❌ Offboard start failed: {error._result.result_str}")
            return

        # Start command listener loop
        while rclpy.ok():
            if self.command:
                await self.send_command(self.command)
                self.command = None
            await asyncio.sleep(0.1)

    def listener_callback(self, msg):
        self.get_logger().info(f'📥 Received gesture command: "{msg.data}"')
        self.command = msg.data

    async def send_command(self, gesture):
        if not self.offboard_started:
            self.get_logger().warn("Offboard mode not started yet")
            return

        cmd = gesture.lower()

        # Map gestures to simple velocity/yaw commands
        if cmd == "forward":
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(1.0, 0.0, 0.0, 0.0)
            )
        elif cmd == "backward":
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(-1.0, 0.0, 0.0, 0.0)
            )
        elif cmd == "left":
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, -1.0, 0.0, 0.0)
            )
        elif cmd == "right":
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 1.0, 0.0, 0.0)
            )
        elif cmd == "up":
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, -0.5, 0.0)
            )
        elif cmd == "down":
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.5, 0.0)
            )
        elif cmd == "yaw_l":
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, -45.0)
            )
        elif cmd == "yaw_r":
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 45.0)
            )
        elif cmd == "stop":
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
            )
        elif cmd == "arm":
            await self.drone.action.arm()
        elif cmd == "disarm":
            await self.drone.action.disarm()
        else:
            self.get_logger().warn(f"Unknown command: {cmd}")

def main(args=None):
    rclpy.init(args=args)
    node = DroneCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())
