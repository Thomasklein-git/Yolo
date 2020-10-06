#!/usr/bin/env python3

import numpy as np
from scipy.spatial import distance as dist

class Object_handler():
    def __init__(self):
        classNum = 3
        self.Current = []
        self.Known = []
        self.Lost = []
        self.UID = []
        self.ClassID = np.zeros(classNum,dtype = int)

        # [class, ID, cx, cy, Start_x, Start_y, End_x, End_y, Score, Occlusion]
    def add(self,Objects):
        if len(Objects) > 0:
            for i in range(0,len(Objects)):
                Start_x = int(Objects[i,0])
                Start_y = int(Objects[i,1])
                End_x   = int(Objects[i,2])
                End_y   = int(Objects[i,3])
                Score   = Objects[i,4]
                Class   = int(Objects[i,5])
                Cx      = int((Start_x + End_x) / 2)
                Cy      = int((Start_y + End_y) / 2)
                Current = ([Class, Cx, Cy, Start_x, Start_y, End_x, End_y, Score])
                self.Current.append(Current)
        

    def merge(self):
        ############## Case 1 ##############
        # No New Objects are present
        # Add one to all Occlusion values
        if len(self.Current) == 0:
            if len(self.Known) > 0:
                for i in range(0,len(self.Known)):
                    self.Known[i][9] += 1
                    
        
        ############## Case 2 ##############
        # New Objects are present but no known objects are present
        if len(self.Known) == 0:
            # Known is empty add all new instances to Known
            for i in range(0,len(self.Current)):
                Object = self.Current[i]
                #UID = self.incUID           
                ID = self.incClassID(Object[0])
                Known = [Object[0], ID, Object[1], Object[2], Object[3], Object[4] ,Object[5] ,Object[6] ,Object[7], 0 ]
                self.Known.append(Known)
        ############## Case 3 ##############  
        # Find List of Unique Classes
        if len(self.Current) > 0:
            Unique_Classes = self.Unique_List([row[0] for row in self.Current])
            # For Loop over each Unique Class
            for x in Unique_Classes:
                for C
                
            
        
                
    #def incUID(self):
    #    self.UID += 1
    #    return self.UID
    
    def clear(self):
        self.Current = []
        
    
    def incClassID(self,Class):
        self.ClassID[Class] += 1
        return self.ClassID[Class]
    
    def Unique_List(self, List):  
        Unique_Entries = []
        for x in List: 
            if x not in Unique_Entries: 
                Unique_Entries.append(x)
        return Unique_Entries
        
            
        