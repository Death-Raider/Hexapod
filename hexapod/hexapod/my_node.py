#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import time
import math

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
        self.key_input.keyListener(self.movement)
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
        print("body ready")

    def stand_body(self):
        for i in range(0,6):
            theta1,theta2 = self.rotation.inverse_kinematics((1,0), (0,0.5), 0.5, 1.0)
            self.set_leg(i,theta1, theta2, self.transform_manager_static)
    
    def set_leg(self, i, theta1, theta2, transform_manager):
        transform_manager.set_transform(
            'body',
            self.legs[i].joints[0],
            self.legs[i].joints_pos[0]+self.legs[i].joints_rot[0],
            self.get_timestamp()
        )
        transform_manager.broadcast_transform()
        self.legs[i].joints_angle[1] = [0,-theta1,0]
        transform_manager.set_transform(
            self.legs[i].joints[0],
            self.legs[i].joints[1],
            self.legs[i].joints_pos[1]+self.rotation.euler_to_quaternion(0,-theta1,0),
            self.get_timestamp()
        )
        transform_manager.broadcast_transform()
        self.legs[i].joints_angle[2] = [0,theta2+theta1,0]
        transform_manager.set_transform(
            self.legs[i].joints[1],
            self.legs[i].joints[2],
            self.legs[i].joints_pos[2]+self.rotation.euler_to_quaternion(0,-theta2+theta1,0),
            self.get_timestamp()
        )
        transform_manager.broadcast_transform()

    def move_leg(self, leg_index:int, stride_angle:float, smoothness:int):

        def leg_plane_x(angle:float,stride_angle:float):
            x = math.cos(stride_angle/2)/math.cos(angle - stride_angle/2)
            return x

        leg = self.legs[leg_index]
        joint_index = 0 # rotate at mount
        phi = stride_angle
        steps = smoothness
        max_height = 0.5
        # get foot and rotate it about joint_index by phi
        leg.get_absolute_joint_position()
        init_foot_pos = leg.joints_abs_pos[-1].copy()
        final_pos_foot = self.rotation.rotate_about_point(*leg.joints_abs_pos[-1],*leg.joints_abs_pos[joint_index],0,0,phi)
        # get the path that the foot should take
        path_pos = leg.get_leg_movement(init_foot_pos,final_pos_foot,steps,max_height)
        print(path_pos)
        for t in range(0, steps):
            x_1 =  leg_plane_x((t+1)*phi/steps, phi)
            start_point = [0,0.5] # x position for foot, height of movement
            plane_point = [x_1, path_pos[t][2] - path_pos[0][2]] # x position for foot, height of movement
            print("positions:", start_point, plane_point)
            theta1,theta2 = self.rotation.inverse_kinematics(plane_point,start_point,leg.lengths[1],leg.lengths[2])
            # print("angles:", theta1,theta2)
            self.set_leg(leg_index,theta1,theta2,self.transform_manager_dynamic)
            # update mount angle by phi, get foot
            leg.joints_angle[joint_index][2 if joint_index == 0 else 1] += phi/steps # only updated mount in z axis, other joints y axis
            leg.joints_rot[joint_index] = self.rotation.euler_to_quaternion(*leg.joints_angle[joint_index])
            time.sleep(0.01)

    def movement(self,char):
        if char == 'w':
            print("moved leg")
            self.move_leg(3,0.53,100)
            pass
        elif char == 's':
            self.move_leg(3,-0.53,100)
            print("moved leg")
            pass
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
