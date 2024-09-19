#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

# uncomment when building
# from test1.rotation import Rotation
# from test1.transformManager import TransfromManager
# from test1.wheels import Wheel
# from test1.keyboard import KeyboardInput
# from test1.body import Body


# comment out when building 
from rotation import Rotation
from transformManager import TransformManager
from leg import Leg
from keyboard import KeyboardInput
from body import Body

class TransformPublisher(Node):
    def __init__(self,node_name):
        super().__init__(node_name)

        self.transform_manager_static = TransformManager(node_cls = self, type = 'static')
        self.transform_manager_dynamic = TransformManager(node_cls = self, type = 'dynamic')
        self.rotation = Rotation()
        self.key_input = KeyboardInput()
        self.body = Body()
        self.wheels = [Leg(i,f"leg_{i}") for i in range(6)] # create 6 wheels
        self.init_body()
        self.key_input.keyListener(self.movement)
        # self.timer = self.create_timer(0.1, self.rotate_wheels_sync)

    def init_body(self):
        # Set the body position of rover
        # B(1,0)
        self.transform_manager_static.set_transform('world','body',self.body.pos+self.body.rot, self.get_timestamp() )
        self.transform_manager_static.broadcast_transform()
        # Set the leg_mounts around the body of the rover in circle assuming B is origin
        init_dir = [1,0,0]
        angle = 0
        for i in range(6):
            pos_w = self.rotation.rotate_point(init_dir)
            self.wheels[w].pos[:3] = pos_w
            self.wheels[w].pos[3:] = self.rotation.euler_to_quaternion(*self.wheel_rot_angles[i]) # wheel orientation only effects yaw
            self.transform_manager_static.set_transform('body',f'wheel_{w}', self.wheels[w].pos, self.get_timestamp() )
            self.transform_manager_static.broadcast_transform()
            w+=1

    def get_timestamp(self):
        return self.get_clock().now().to_msg()

    def movement(self,char):
        """
            TODO: add right(a) and left(d) movement ketys to change orientation of wheels 
                : make sure the wheels dont disappear when no key is pressed (add init_body call or save_state call to send a static transform)
        """
        if char == 'w':
            self.rotate_wheels_sync(0.1,1)
        elif char == 's':
            self.rotate_wheels_sync(0.1,-1)
        else:
            self.init_body()

    def rotate_legs_sync(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = TransformPublisher('transform_publisher')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
