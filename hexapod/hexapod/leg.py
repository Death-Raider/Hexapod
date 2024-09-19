from rotation import Rotation

class Leg:
    def __init__(self,leg_id, leg_name, leg_mount_name):
        self.id = leg_id
        self.name = leg_name
        self.lengths = [0.25,0.5,1] # [leg_mount to joint_1, joint_1 to joint_2, joint_2 to feet]
        self.mount_dir = [1,0,0] # leg mounted in this direction [x,y,z]
        self.joints = [leg_mount_name, 'joint_1', 'joint_2', 'feet']
        self.joints_pos = [[0]*3 for i in self.joints]
        self.joints_rot = [[0]*4 for i in self.joints]
        self.joints_angle = [[0]*3 for i in self.joints]
        self.rotation = Rotation()
    
    def init_leg(self):
        # initially leg is flat in the mount direction
        data = []
        for i in range(len(self.joints[:-1])):
            data.append({})
            data[i]['parent_name'] = self.joints[i]
            data[i]['child_name'] = self.joints[i+1]
            data[i]['child_pos'] = self.rotation.scale(self.lengths[i], *self.mount_dir)
            data[i]['child_rot'] = [0,0,0,1] # no rotation
        return data
    



