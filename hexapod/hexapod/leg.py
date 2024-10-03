# from hexapod.rotation import Rotation
from rotation import Rotation
import numpy as np
import math
import matplotlib.pyplot as plt

class Leg:
    def __init__(self,leg_id, leg_name):
        self.id = leg_id
        self.name = leg_name
        self.lengths = [0.25, 0.5, 1] # [leg_mount to joint_1, joint_1 to joint_2, joint_2 to feet]
        self.mount_dir = [1.0, 0.0, 0.0] # leg mounted in this direction [x,y,z]
        self.joints = list(map(lambda x: f"{self.name}_{x}", ['mount', 'joint_1', 'joint_2', 'feet']))
        self.joints_pos = [[0.0]*3 for i in self.joints]
        self.joints_rot = [[0.0, 0.0, 0.0, 1.0] for i in self.joints]
        self.joints_angle = [[0.0]*3 for i in self.joints]
        self.rotation = Rotation()
    
    def init_leg(self):
        # initially leg is flat in the mount direction
        data = []
        for i in range(1, len(self.joints)):
            self.joints_pos[i] = [self.lengths[i-1]*1.0, 0.0, 0.0]
            data.append({
                'parent_name' : self.joints[i-1],
                'child_name': self.joints[i],
                'child_pos' : self.joints_pos[i],
                'child_rot' :  self.joints_rot[i]
            })
        return data
    
    def get_absolute_joint_position(self):
        self.joints_abs_pos = [[0.0]*3 for i in self.joints_pos]
        self.joints_abs_angle = [[0.0]*3 for i in self.joints_pos]
        self.joints_abs_pos[0] = self.joints_pos[0].copy()
        self.joints_abs_angle[0] = self.joints_angle[0].copy()
        for i in range(1,len(self.joints_pos)):
            self.joints_abs_angle[i] = self.joints_angle[i].copy()
            self.joints_abs_angle[i][0] += self.joints_abs_angle[i-1][0]
            self.joints_abs_angle[i][1] += self.joints_abs_angle[i-1][1]
            self.joints_abs_angle[i][2] += self.joints_abs_angle[i-1][2]
            self.joints_abs_pos[i] = self.rotation.rotate_point(*self.joints_pos[i],*self.joints_abs_angle[i-1])
            self.joints_abs_pos[i][0] += self.joints_abs_pos[i-1][0]
            self.joints_abs_pos[i][1] += self.joints_abs_pos[i-1][1]
            self.joints_abs_pos[i][2] += self.joints_abs_pos[i-1][2]

    def get_leg_movement(self,start_foot_pos, final_foot_pos, stride_time):
        
        def movement_function(x:float, vec:list[float], max_height:float = 1)->float:
            mag = math.hypot(*vec)
            return -4*max_height*(x/mag)**2+4*max_height*x/mag # returns the height
        
        def projected_movement(x:float, y:float, vec:list[float])->float:
            mag = math.hypot(*vec)
            return movement_function( np.dot(vec,(x,y))/mag , vec)
        
        def planar_movement(t:float, vec:list[float])->tuple[float]:
            ratio = vec[1]/vec[0]  # y/x
            proj_mov = projected_movement(t, ratio*t, vec)
            return [t, ratio*t, proj_mov]
        
        diff_vec = [final_foot_pos[0] - start_foot_pos[0] , final_foot_pos[1] - start_foot_pos[1]]

        cond = diff_vec[0] < 0
        t_start = min(0,diff_vec[0])
        t_end = max(0,diff_vec[0])
        pos = []
        for t in np.linspace(t_start, t_end, stride_time):
            f_dash = planar_movement(t,diff_vec)
            curr_foot_position = np.sum([start_foot_pos,f_dash],axis=0)
            pos.append(curr_foot_position)
        pos = np.array(pos)
        if cond:
            pos = pos[::-1]
        return pos

# leg = Leg(0,'leg_test')
# leg.joints_pos = [
#     [0.0, 0.0, 1.0],
#     [1.0, 0.0, 0.0],
#     [0.0, 1.0, 0.0],
#     [0.0, -1.0, 0.0],
# ]
# leg.joints_angle = [
#     [0.0, 0.0, 1.57],
#     [0.0, 0.0, 0.0],
#     [0.0, 0.0, 1.57],
#     [0.0, -1.57, 0.0],
# ]
# leg.get_absolute_joint_position()
# print(leg.joints_abs_pos)
# pos = leg.get_leg_movement(np.radians(30), 30)
