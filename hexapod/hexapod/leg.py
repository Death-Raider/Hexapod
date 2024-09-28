from rotation import Rotation
import numpy as np
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
    



