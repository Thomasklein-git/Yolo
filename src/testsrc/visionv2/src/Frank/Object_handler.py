#!/usr/bin/env python3

import numpy as np
from scipy.spatial import distance as dist

class Object_handler():
    def __init__(self):
        classNum = 3
        self.OcclusionLimit = 50
        self.Current = []
        self.Known = []
        self.Lost = []
        self.UID = 0
        self.ClassID = np.zeros(classNum,dtype = int)

        # [UID, ID, class,  cx, cy, Start_x, Start_y, End_x, End_y, Score, Occlusion]
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
        self.merge()
        self.clear()

    def merge(self):
        ############## Case 1 ##############
        # Current == 0
        # Known == 0
        if len(self.Current) == 0:
            if len(self.Known) == 0:
                print("Case 1")

            ############## Case 2 ##############
            # Current == 0 
            # Known > 0
            # No New Objects are present
            # Add one to all Occlusion values
            else:
                #print("Case 2")
                for i in range(0,len(self.Known)):
                    self.Known[i][10] += 1

        ############## Case 3 ##############
        # Current > 0
        # Known == 0
        else:
            if len(self.Known) == 0:
                    #print("Case 3")
                    for i in range(0,len(self.Current)):
                        self.upgrade(self.Current[i])

        ############## Case 4 ##############
        # Current > 0
        # Known > 0
            else:
                #print("Case 4")
                Unique_Classes = self.Unique_List([row[0] for row in self.Current])
                # For Loop over each Unique Class
                Current_classes = [row[0] for row in self.Current] 
                Known_classes   = [row[2] for row in self.Known]
                for c in Unique_Classes:
                    Current_i = [i for i, x in enumerate(Current_classes) if c == x]
                    Known_i   = [i for i, x in enumerate(Known_classes) if c == x]
                    Current_C = []
                    Known_C   = []
                    for i in Current_i:
                        Current_C.append(self.Current[i][1:3])
                        
                    for i in Known_i:
                        Known_C.append(self.Known[i][3:5])

                    D = dist.cdist(np.array(Current_C), np.array(Known_C))
                    UsedRow = []
                    UsedCol = []
                    pairs = min(len(Current_i), len(Known_i))
                    for i in range(0,pairs):
                        D1 = np.where(D==D.min())
                        UsedRow.append(D1[0][0])
                        UsedCol.append(D1[1][0])
                        D[UsedRow[i]][0:len(Known_i)] = 1000
                        for j in range(0,len(Current_i)):
                            D[j][UsedCol[i]] = 1000
                    # Updating Known to match current pairs
                    for i in UsedRow:
                        for j in UsedCol:
                            Current_update = self.Current[Current_i[i]]
                            Known_update = self.Known[Known_i[j]][0]

                            self.update(Current_update,Known_update)
                    ## Adding new points not matched with a known point

                    if len(UsedRow) < len(Current_i):
                        New_Points = np.delete(Current_i,[UsedRow])
                        for i in New_Points:
                            self.upgrade(self.Current[i])
                    
                    if len(UsedCol) < len(Known_i):
                        Lost_Points = np.delete(Known_i,[UsedCol])
                        for i in Lost_Points:
                            self.Known[i][10] += 1        

    def upgrade(self,Current):     
        ID, UID = self.incID(Current[0])
        Known = [UID, ID, Current[0],  Current[1], Current[2], Current[3], Current[4] ,Current[5] ,Current[6] ,Current[7], 0 ]          
        self.Known.append(Known)

    def update(self,Current,Known_update):
        #Matching UID
        for i in range(0,len(self.Known)):
            if Known_update == self.Known[i][0]:
                Knownrow = i
        
        self.Known[Knownrow][3] = Current[1]
        self.Known[Knownrow][4] = Current[2]
        self.Known[Knownrow][5] = Current[3]
        self.Known[Knownrow][6] = Current[4]
        self.Known[Knownrow][7] = Current[5]
        self.Known[Knownrow][8] = Current[6]
        self.Known[Knownrow][9] = Current[7]
        self.Known[Knownrow][10] = 0

    def clear(self):
        self.Current = [] 
        Lost_UID = []
        for i in range(0,len(self.Known)):
            if self.Known[i][10] == self.OcclusionLimit:
                Lost_UID.append(self.Known[i][0])
        for i in Lost_UID:
            self.Remove(i)
    
    def incID(self,Class):
        UID = self.UID
        ClassID = self.ClassID[Class]
        self.UID += 1
        self.ClassID[Class] += 1
        return ClassID, UID
    
    def Unique_List(self, List):  
        Unique_Entries = []
        for x in List: 
            if x not in Unique_Entries: 
                Unique_Entries.append(x)
        return Unique_Entries
    
    def Remove(self,Lost_UID):
        indexes = []
        for i in range(0,len(self.Known)):
            if self.Known[i][0] == Lost_UID:
                Lost_P = self.Known[i]
                UID = self.Known[i][0]
                ID = self.Known[i][1]
                Class = self.Known[i][2]
                Lost = [UID, ID, Class]
                self.Lost.append(Lost)
                indexes.append(i)
        for index in sorted(indexes, reverse=True):
            del self.Known[index]

        