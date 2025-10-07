import rclpy
from rclpy.node import Node
import os
from gazebo_msgs.srv import SpawnEntity

class RoverSpawner(Node):
    def __init__(self):
        super().__init__('rover_spawner')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')

        request = SpawnEntity.Request()
        urdf_path = os.path.join(
            os.getenv('AMENT_PREFIX_PATH').split(':')[0],
            'share/rover2/urdf/rover_first.urdf'
        )
        request.xml = open(urdf_path, 'r').read()
        request.name = 'rover'

        self.future = self.cli.call_async(request)
        self.get_logger().info(f'Spawning rover from {urdf_path}')

def main(args=None):
    rclpy.init(args=args)
    node = RoverSpawner()
    rclpy.spin_until_future_complete(node, node.future)
    node.get_logger().info('Rover spawned successfully!')
    node.destroy_node()
    rclpy.shutdown()
