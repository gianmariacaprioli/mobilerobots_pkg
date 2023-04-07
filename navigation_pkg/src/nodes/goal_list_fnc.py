#!/usr/bin/env python3

import random

class lista:

    def __init__(self):
        self.list = [[3.0,1.0,1.0],[5.0,3.0,1.0],[5.0,5.0,1.0]]
    
    def list_generator(self):
            self.list.append([5,float(random.randint(-8,8)),1])
            # goal_list[self.i][0] = 5
            # goal_list[self.i][1] = float(random.randint(-8,8))
            # goal_list[self.i][2] = 1

            return self.list
    
if __name__ == '__name__':
     adj=lista()
     adj.list_generator()