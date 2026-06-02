# spawn_objects_fixed.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import time
import sys

class ObjectSpawner(Node):
    def __init__(self):
        super().__init__('object_spawner')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        
        # Wait for service with proper timeout check
        self.get_logger().info('Waiting for /spawn_entity service...')
        
        # wait_for_service returns True if service becomes available within timeout
        # Returns False if timeout occurs
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service /spawn_entity not available after 10 seconds')
            self.get_logger().info('Make sure Gazebo is running with ROS plugins')
            rclpy.shutdown()
            sys.exit(1)
        
        self.get_logger().info('Connected to /spawn_entity service!')
        
    def spawn_object(self, name, model_type, x, y, z, color="1 0 0 1"):
        # Different SDF models based on type
        if model_type == "cube":
            sdf = self.create_cube_sdf(name, x, y, z, color)
        elif model_type == "sphere":
            sdf = self.create_sphere_sdf(name, x, y, z, color)
        elif model_type == "cylinder":
            sdf = self.create_cylinder_sdf(name, x, y, z, color)
        else:
            self.get_logger().error(f"Unknown model type: {model_type}")
            return False
            
        request = SpawnEntity.Request()
        request.name = name
        request.xml = sdf
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = z
        
        self.get_logger().info(f'Spawning {name}...')
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✓ Successfully spawned {name} at ({x}, {y}, {z})')
                return True
            else:
                self.get_logger().error(f'✗ Failed to spawn {name}: {response.status_message}')
                return False
        else:
            self.get_logger().error(f'✗ Service call failed for {name}')
            return False
    
    def create_cube_sdf(self, name, x, y, z, color):
        # Simplified SDF without inertial properties (they can cause issues)
        return f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <pose>{x} {y} {z} 0 0 0</pose>
    <static>false</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.5 0.5</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.5 0.5</size>
          </box>
        </geometry>
        <material>
          <ambient>{color}</ambient>
          <diffuse>{color}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''
    
    def create_sphere_sdf(self, name, x, y, z, color):
        return f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <pose>{x} {y} {z} 0 0 0</pose>
    <static>false</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <sphere>
            <radius>0.3</radius>
          </sphere>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>0.3</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>{color}</ambient>
          <diffuse>{color}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''
    
    def create_cylinder_sdf(self, name, x, y, z, color):
        return f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <pose>{x} {y} {z} 0 0 0</pose>
    <static>false</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.25</radius>
            <length>1.0</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.25</radius>
            <length>1.0</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>{color}</ambient>
          <diffuse>{color}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''

def main():
    rclpy.init()
    
    try:
        spawner = ObjectSpawner()
    except SystemExit:
        return  # Exit if service not available
    
    # Spawn various objects
    objects = [
        ("red_cube", "cube", 0, 0, 1.0, "1 0 0 1"),
        ("blue_sphere", "sphere", 2, 0, 1.0, "0 0 1 1"),
        ("green_cube", "cube", -2, 0, 1.0, "0 1 0 1"),
        ("yellow_cube", "cube", 0, 3, 1.0, "1 1 0 1"),
        ("purple_sphere", "sphere", 3, 0, 1.0, "1 0 1 1"),
    ]
    
    success_count = 0
    for obj in objects:
        if spawner.spawn_object(*obj):
            success_count += 1
        time.sleep(0.5)  # Small delay between spawns
    
    spawner.get_logger().info(f"Spawned {success_count}/{len(objects)} objects successfully!")
    
    # Optional: Keep node alive for a bit
    time.sleep(2)
    
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()