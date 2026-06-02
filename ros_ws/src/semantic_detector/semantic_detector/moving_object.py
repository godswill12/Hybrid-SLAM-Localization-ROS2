# moving_objects_controller.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist, Point, Quaternion
import numpy as np
import threading
import time

class ObjectController(Node):
    def __init__(self):
        super().__init__('object_controller')
        self.client = self.create_client(SetEntityState, '/set_entity_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_entity_state service...')
        
        # Define different motion patterns
        self.objects = {
            'red_cube': {
                'type': 'circle',
                'radius': 2.0,
                'speed': 1.0,
                'center': [0, 0, 1],
                'angle': 0.0
            },
            'blue_sphere': {
                'type': 'line',
                'start': [2, -2, 1],
                'end': [2, 2, 1],
                'speed': 0.8,
                'position': 0.0,
                'direction': 1
            },
            'moving_object_1': {
                'type': 'figure8',
                'radius': 1.5,
                'speed': 1.2,
                'center': [0, 3, 1],
                'angle': 0.0
            },
            'moving_object_2': {
                'type': 'square',
                'size': 2.0,
                'speed': 0.5,
                'center': [3, 0, 1],
                'corner': 0
            }
        }
        
        self.timer = self.create_timer(0.05, self.update_objects)  # 20Hz update rate
        
    def update_objects(self):
        for obj_name, obj_data in self.objects.items():
            if obj_data['type'] == 'circle':
                self.move_circle(obj_name, obj_data)
            elif obj_data['type'] == 'line':
                self.move_line(obj_name, obj_data)
            elif obj_data['type'] == 'figure8':
                self.move_figure8(obj_name, obj_data)
            elif obj_data['type'] == 'square':
                self.move_square(obj_name, obj_data)
    
    def move_circle(self, name, data):
        # Circular motion
        x = data['center'][0] + data['radius'] * np.cos(data['angle'])
        y = data['center'][1] + data['radius'] * np.sin(data['angle'])
        
        vx = -data['radius'] * np.sin(data['angle']) * data['speed']
        vy = data['radius'] * np.cos(data['angle']) * data['speed']
        
        self.set_entity_state(name, x, y, data['center'][2], vx, vy, 0)
        data['angle'] += 0.05 * data['speed']
    
    def move_line(self, name, data):
        # Linear back-and-forth motion
        t = data['position']
        x = data['start'][0]
        y = data['start'][1] + (data['end'][1] - data['start'][1]) * t
        
        vy = (data['end'][1] - data['start'][1]) * data['speed'] * data['direction']
        
        self.set_entity_state(name, x, y, data['start'][2], 0, vy, 0)
        
        data['position'] += 0.05 * data['speed'] * data['direction']
        if data['position'] >= 1.0 or data['position'] <= 0.0:
            data['direction'] *= -1
    
    def move_figure8(self, name, data):
        # Figure-8 motion (Lissajous curve)
        x = data['center'][0] + data['radius'] * np.sin(data['angle'])
        y = data['center'][1] + data['radius'] * np.sin(2 * data['angle']) / 2
        
        vx = data['radius'] * np.cos(data['angle']) * data['speed']
        vy = data['radius'] * np.cos(2 * data['angle']) * data['speed']
        
        self.set_entity_state(name, x, y, data['center'][2], vx, vy, 0)
        data['angle'] += 0.03 * data['speed']
    
    def move_square(self, name, data):
        # Square path motion
        size = data['size']
        corners = [
            [data['center'][0] - size/2, data['center'][1] - size/2],
            [data['center'][0] + size/2, data['center'][1] - size/2],
            [data['center'][0] + size/2, data['center'][1] + size/2],
            [data['center'][0] - size/2, data['center'][1] + size/2]
        ]
        
        current_corner = data['corner']
        next_corner = (current_corner + 1) % 4
        
        # Linear interpolation between corners
        start = corners[current_corner]
        end = corners[next_corner]
        
        t = data['position']
        x = start[0] + (end[0] - start[0]) * t
        y = start[1] + (end[1] - start[1]) * t
        
        vx = (end[0] - start[0]) * data['speed']
        vy = (end[1] - start[1]) * data['speed']
        
        self.set_entity_state(name, x, y, data['center'][2], vx, vy, 0)
        
        data['position'] += 0.05 * data['speed']
        if data['position'] >= 1.0:
            data['position'] = 0.0
            data['corner'] = next_corner
    
    def set_entity_state(self, name, x, y, z, vx, vy, vz):
        state = EntityState()
        state.name = name
        state.pose.position = Point(x=float(x), y=float(y), z=float(z))
        state.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        state.twist.linear = Point(x=float(vx), y=float(vy), z=float(vz))
        state.reference_frame = 'world'
        
        request = SetEntityState.Request()
        request.state = state
        
        # Async call - don't wait for response
        self.client.call_async(request)

def main():
    rclpy.init()
    controller = ObjectController()
    controller.get_logger().info("Starting object movement controller...")
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Shutting down controller...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()