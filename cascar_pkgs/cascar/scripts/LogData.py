import numpy as np
import pickle

class LogData:
    left = np.zeros((0, 4))
    right = np.zeros((0, 4))
    qualisys = {}
    
    def __init__(self, filename):
        self.filename = filename

    def DumpLog(self):
        with open(self.filename, 'wb') as handle:
            dd = {'qualisys': self.qualisys}
            dd['car_ticks'] = {
                'fields': ['t', 'dt', 'vel', 'steer'],
                'left': self.left, 'right': self.right}                
            pickle.dump(dd, handle)
        
    def Clear(self):
        self.left = np.zeros((0, 4))
        self.right = np.zeros((0, 4))        

    def AppendTicksData(self, msg):
        if msg.wheel=="L":
            self.left = np.append(self.left,
                                  [[msg.t, msg.dt, msg.velocity, msg.steer]],
                                  axis=0)
        elif msg.wheel=="R":
            self.right = np.append(self.right,
                                   [[msg.t, msg.dt, msg.velocity, msg.steer]],
                                   axis=0)
        else:
            print("Incorrect format on log message")

    def AppendQualisysData(self, msg, time):
        body_name = msg.name
        if not self.qualisys.has_key(body_name):
            self.qualisys[body_name] = {'position': np.zeros((0, 3)),
                               'orientation': np.zeros((0, 4)),
                               't': np.zeros([])}
        self.qualisys[body_name]['position'] = np.append(
            self.qualisys[body_name]['position'],
            [[msg.position.x, msg.position.y, msg.position.z]],
            axis=0)
        self.qualisys[body_name]['orientation'] = np.append(
            self.qualisys[body_name]['orientation'],
            [[msg.orientation.x, msg.orientation.y,
              msg.orientation.z, msg.orientation.w]],
            axis=0)
        self.qualisys[body_name]['t'] = np.append(
            self.qualisys[body_name]['t'], time)
        
    def DataSize(self):
        n_qualisys = [len(self.qualisys[body_data]['t'])
                      for body_data in self.qualisys]
        n_ticks = [self.left.shape[0], self.right.shape[0]]
        return ((n_ticks, n_qualisys))
