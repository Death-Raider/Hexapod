#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import time

# uncomment when building
# from hexapod.rotation import Rotation
# from hexapod.transformManager import TransformManager
# from hexapod.leg import Leg
# from hexapod.keyboardInput import KeyboardInput
# from hexapod.body import Body


# comment out when building 
from rotation import Rotation
from transformManager import TransformManager
from leg import Leg
from keyboardInput import KeyboardInput
from body import Body

class TransformPublisher(Node):
    def __init__(self,node_name):
        super().__init__(node_name)

        self.transform_manager_static = TransformManager(node_cls = self, type = 'static')
        self.transform_manager_dynamic = TransformManager(node_cls = self, type = 'dynamic')
        self.rotation = Rotation()
        self.key_input = KeyboardInput()
        self.body = Body()
        self.legs = [Leg(i,f"leg_{i}") for i in range(6)] # create 6 legs
        self.init_body()
        # self.key_input.keyListener(self.movement)
        self.p1 = 0
        self.p2 = 0
        # self.timer = self.create_timer(0.1, self.stand_body)

    def get_timestamp(self):
        return self.get_clock().now().to_msg()

    def init_body(self): # lay flat on the floor
        # Set the body position of rover
        # B(1,0,0.5)
        self.transform_manager_static.set_transform('world','body',self.body.pos+self.body.rot, self.get_timestamp() )
        self.transform_manager_static.broadcast_transform()
        # Set the leg_mounts around the body of the rover in circle assuming B is origin
        init_dir = [1.0, 0.0, 0.0]
        angle = 0.0
        for i in range(6):
            mount_pos = self.rotation.rotate_point(*init_dir, 0.0, 0.0, angle) # rotate init_dir in increments of 120 deg
            self.legs[i].mount_dir = mount_pos
            # mount the leg in this direction
            self.legs[i].joints_angle[0] = [0.0, 0.0, angle]
            self.legs[i].joints_pos[0] = self.rotation.scale(self.body.radius, *mount_pos)
            self.legs[i].joints_rot[0] = self.rotation.euler_to_quaternion(*self.legs[i].joints_angle[0])
            # set the body and leg transforms
            self.transform_manager_static.set_transform('body', self.legs[i].joints[0], self.legs[i].joints_pos[0]+self.legs[i].joints_rot[0], self.get_timestamp())
            self.transform_manager_static.broadcast_transform()

            # initilize the rest of the leg
            leg_details = self.legs[i].init_leg()
            for data in leg_details:
                self.transform_manager_static.set_transform(data['parent_name'], data['child_name'] , data['child_pos']+data['child_rot'], self.get_timestamp())
                self.transform_manager_static.broadcast_transform()
            angle += np.pi/3
            time.sleep(0.01)
        self.stand_body()
        self.move_leg(2)

    def stand_body(self):
        for i in range(0,6):
            theta1 = np.pi/4
            leg_len_ratio = self.legs[i].lengths[1]/self.legs[i].lengths[2]
            mount_leg_ratio = self.body.pos[-1]/self.legs[i].lengths[2] # height of mount is same as z axis of body
            theta2 = np.arcsin( leg_len_ratio*np.sin(theta1)+ mount_leg_ratio)
            self.transform_manager_static.set_transform(
                'body',
                self.legs[i].joints[0],
                self.legs[i].joints_pos[0]+self.legs[i].joints_rot[0],
                self.get_timestamp()
            )
            self.transform_manager_static.broadcast_transform()
            self.legs[i].joints_angle[1] = [0,-theta1,0]
            self.transform_manager_static.set_transform(
                self.legs[i].joints[0],
                self.legs[i].joints[1],
                self.legs[i].joints_pos[1]+self.rotation.euler_to_quaternion(0,-theta1,0),
                self.get_timestamp()
            )
            self.transform_manager_static.broadcast_transform()
            self.legs[i].joints_angle[2] = [0,theta2+theta1,0]
            self.transform_manager_static.set_transform(
                self.legs[i].joints[1],
                self.legs[i].joints[2],
                self.legs[i].joints_pos[2]+self.rotation.euler_to_quaternion(0,theta2+theta1,0),
                self.get_timestamp()
            )
            self.transform_manager_static.broadcast_transform()
 
    def move_leg(self, leg_index):
        leg = self.legs[leg_index]
        joint_index = 1
        # angles = self.rotation.inverse_kinematics()
        phi = np.radians(30) # stride angle
        leg.joints_angle[joint_index][1] += phi
        leg.joints_rot[joint_index] = self.rotation.euler_to_quaternion(*leg.joints_angle[joint_index])

        self.transform_manager_static.set_transform(
            'body' if joint_index == 0 else leg.joints[joint_index-1],
            leg.joints[joint_index],
            leg.joints_pos[joint_index]+leg.joints_rot[joint_index],
            self.get_timestamp()
        )
        self.transform_manager_static.broadcast_transform()

    def movement(self,char):
        """
            TODO: add right(a) and left(d) movement ketys to change orientation of wheels 
                : make sure the wheels dont disappear when no key is pressed (add init_body call or save_state call to send a static transform)
        """
        if char == 'w':
            pass
            # self.rotate_wheels_sync(0.1,1)
        elif char == 's':
            pass
            # self.rotate_wheels_sync(0.1,-1)
        else:
            self.init_body()

    def rotate_legs_sync(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = TransformPublisher('hexapod_node')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
