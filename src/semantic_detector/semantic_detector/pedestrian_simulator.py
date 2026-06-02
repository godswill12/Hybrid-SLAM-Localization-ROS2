# pedestrian_simulator_terminal.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Point, Quaternion
import numpy as np
import math

class PedestrianSimulator(Node):
    def __init__(self):
        super().__init__('pedestrian_simulator')
        
        # Services
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.state_client = self.create_client(SetEntityState, '/set_entity_state')
        
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        while not self.state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for state service...')
        
        # Pedestrian data
        self.pedestrians = []
        self.num_pedestrians = 3
        
        # Spawn pedestrians
        self.spawn_pedestrians()
        
        # Start movement timer
        self.timer = self.create_timer(0.1, self.move_pedestrians)  # 10Hz
        
    def spawn_pedestrians(self):
        """Spawn multiple pedestrians with different paths"""
        
        # Create pedestrian SDF (simple cylinder for visualization)
        pedestrian_sdf = '''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="pedestrian_template">
    <pose>0 0 0.5 0 0 0</pose>
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>70.0</mass>
        <inertia>
          <ixx>5.0</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>5.0</iyy>
          <iyz>0</iyz>
          <izz>2.0</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.3</radius>
            <length>1.7</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.3</radius>
            <length>1.7</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.8 0.6 0.2 1</ambient>
          <diffuse>0.8 0.6 0.2 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''
        
        # Spawn pedestrians at different locations
        start_positions = [
            (-5, -5, 0.85),  # Bottom-left
            (5, -5, 0.85),   # Bottom-right
            (0, -8, 0.85),   # Bottom-center
        ]
        
        waypoints_sets = [
            [  # Pedestrian 1: Cross left to right
                (-5, -5), (-2, -5), (0, -3), (2, -1), (5, 0)
            ],
            [  # Pedestrian 2: Circle around
                (5, -5), (3, -3), (0, 0), (-3, 3), (-5, 5), (-5, 0), (-5, -5)
            ],
            [  # Pedestrian 3: Zigzag
                (0, -8), (2, -6), (-2, -4), (2, -2), (-2, 0), (0, 2)
            ]
        ]
        
        for i in range(self.num_pedestrians):
            name = f'pedestrian_{i+1}'
            start_x, start_y, start_z = start_positions[i]
            
            # Spawn pedestrian
            request = SpawnEntity.Request()
            request.name = name
            request.xml = pedestrian_sdf.replace('pedestrian_template', name)
            request.initial_pose.position.x = start_x
            request.initial_pose.position.y = start_y
            request.initial_pose.position.z = start_z
            
            future = self.spawn_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                self.get_logger().info(f'Spawned {name}')
                
                # Add to pedestrian list
                self.pedestrians.append({
                    'name': name,
                    'position': np.array([float(start_x), float(start_y), float(start_z)]),
                    'waypoints': [np.array([float(wp[0]), float(wp[1]), start_z]) for wp in waypoints_sets[i]],
                    'current_waypoint': 0,
                    'speed': 0.5 + (i * 0.2),  # Different speeds
                    'state': 'moving'
                })
    
    def move_pedestrians(self):
        """Update positions of all pedestrians"""
        for ped in self.pedestrians:
            if ped['state'] == 'moving':
                current_pos = ped['position']
                target = ped['waypoints'][ped['current_waypoint']]
                
                # Calculate direction to target
                direction = target - current_pos
                distance = np.linalg.norm(direction)
                
                if distance < 0.2:  # Reached waypoint
                    ped['current_waypoint'] = (ped['current_waypoint'] + 1) % len(ped['waypoints'])
                    target = ped['waypoints'][ped['current_waypoint']]
                    direction = target - current_pos
                    distance = np.linalg.norm(direction)
                
                # Move towards target
                if distance > 0:
                    direction_normalized = direction / distance
                    new_pos = current_pos + direction_normalized * ped['speed'] * 0.1
                    ped['position'] = new_pos
                    
                    # Update in Gazebo
                    self.update_pedestrian_state(ped['name'], new_pos, direction_normalized)
    
    def update_pedestrian_state(self, name, position, direction):
        """Send updated state to Gazebo"""
        state = EntityState()
        state.name = name
        state.pose.position = Point(
            x=float(position[0]),
            y=float(position[1]),
            z=float(position[2])
        )
        
        # Calculate orientation from direction (face direction of movement)
        yaw = math.atan2(direction[1], direction[0])
        state.pose.orientation = self.yaw_to_quaternion(yaw)
        
        # Set velocity for smooth movement
        state.twist.linear = Point(
            x=float(direction[0]) * 0.5,
            y=float(direction[1]) * 0.5,
            z=0.0
        )
        
        request = SetEntityState.Request()
        request.state = state
        
        # Async call
        self.state_client.call_async(request)
    
    def yaw_to_quaternion(self, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return Quaternion(x=0.0, y=0.0, z=float(sy), w=float(cy))

def main():
    rclpy.init()
    simulator = PedestrianSimulator()
    simulator.get_logger().info("Pedestrian simulator started!")
    
    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        simulator.get_logger().info("Shutting down pedestrian simulator...")
    finally:
        simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()