#!/usr/bin/env python3

import numpy as np
import math
from scipy.spatial import distance as dist
from scipy.optimize import linear_sum_assignment
from agfh import *

class Object_handler():
    def __init__(self,classNum):
        self.OcclusionLimit = 5
        self.Current = []
        self.Known = []
        self.Lost = []
        self.UID = 0
        self.ClassID = np.zeros(classNum,dtype = int)
        self.CurrentOrder = {"Class": 0, "cx": 1, "cy": 2, "Start_x": 3, "Start_y": 4, "End_x": 5, \
             "End_y": 6, "Score": 7, "Depth_X": 8, "Depth_Y": 9, "Depth_Z": 10, "Time": 11, \
                "Current_listing": 12} #Tilføjet time and vehicle XYZ
        self.KnownOrder   = {"UID":  0, "ID": 1, "Class": 2, "cx": 3, "cy": 4, "Start_x": 5, "Start_y": 6, \
             "End_x": 7, "End_y": 8, "Score": 9, "Occlusion": 10, "Depth_X": 11, "Depth_Y": 12, "Depth_Z": 13, \
             "Time": 14, "Current_listing": 15} #Tilføjet time and vehicle XYZ
        self.LostOrder    = {"UID":  0, "ID": 1, "Class": 2}
        self.Dynsta = np.zeros(classNum, dtype=bool) # Dynamic class true, Static class flass (everything is false)
        self.Dynsta[np.array([0])] = True # Make person and car (2) dynamic 
        self.static_V = 0.5 #1.8km/h
        self.dynamic_V = 10 #10.8km/h
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
                #Depth   = Objects[i,6]
                DX = Objects[i,6][0]
                DY = Objects[i,6][1]
                DZ = Objects[i,6][2]
                Time = Objects[i,7] #Tilføjet
                Current = ([Class, Cx, Cy, Start_x, Start_y, End_x, End_y, Score, DX, DY, DZ, Time, i]) #Tilføjet time and vehicle
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
                #print("I see nothing, I know nothing, I am nothing")

            ############## Case 2 ##############
            # Current == 0 
            # Known > 0
            # No New Objects are present
            # Add one to all Occlusion values
            else:
                print("Case 2")
                for i in range(0,len(self.Known)):
                    self.Known[i][self.KnownOrder.get("Occlusion")] += 1
                    self.Known[i][self.KnownOrder.get("Current_listing")] = float("nan")

        ############## Case 3 ##############
        # Current > 0
        # Known == 0
        else:
            if len(self.Known) == 0:
                    print("Case 3")
                    for i in range(0,len(self.Current)):
                        self.upgrade(self.Current[i])

        ############## Case 4 ##############
        # Current > 0
        # Known > 0
            else:
                print("Case 4")
                Unique_Classes = self.Unique_List([row[self.CurrentOrder.get("Class")] for row in self.Current])
                Unique_Known_Classes = self.Unique_List([row[self.KnownOrder.get("Class")] for row in self.Known])
                # For Loop over each Unique Class
                Current_classes = [row[self.CurrentOrder.get("Class")] for row in self.Current] 
                Known_classes   = [row[self.KnownOrder.get("Class")] for row in self.Known]
                for c in Unique_Classes:
                    Current_i = [i for i, x in enumerate(Current_classes) if c == x]
                    Known_i   = [i for i, x in enumerate(Known_classes) if c == x]
                    Current_D = []
                    Known_D = []
                    Current_Time = [] #tilføjet
                    Known_Time = [] #tilføjet
                    UsedRow = []
                    UsedCol = []

                    for i in Current_i:
                        Current_D.append([self.Current[i][self.CurrentOrder.get("Depth_X")],self.Current[i][self.CurrentOrder.get("Depth_Y")],self.Current[i][self.CurrentOrder.get("Depth_Z")]])
                        Current_Time=([self.Current[i][self.CurrentOrder.get("Time")]]) #tilføjet

                    for i in Known_i:
                        Known_D.append([self.Known[i][self.KnownOrder.get("Depth_X")],self.Known[i][self.KnownOrder.get("Depth_Y")],self.Known[i][self.KnownOrder.get("Depth_Z")]])      
                        Known_Time.append([self.Known[i][self.KnownOrder.get("Time")]]) #tilføjet

                    if len(Known_D) > 0:
                        D = dist.cdist(np.array(Current_D), np.array(Known_D))
                        UsedRow, UsedCol = linear_sum_assignment(D)

                        """
                        #print(np.min(D))
                        V_obj=np.array([]) # Velocity of object relative to Vehicle
                        for i in range(D.shape[1]):
                            V=D[:,i]/(np.array(Current_Time)-np.array(Known_Time[i]))
                            V_obj=np.append(V_obj,V)    
                        V_obj=np.reshape(V_obj,D.shape)
                        #print(D,"distance")
                        #print(V_obj, "Velocity object")
                        #print(len(V_obj))

                        if self.Dynsta[c]==False:
                            v_thres=self.static_V
                            #print(v_thres,"sta")
                        else:
                            v_thres=self.dynamic_V
                            #print(v_thres,"dyn")

                        pairs = min(len(Current_i), len(Known_i))
                        
                        
                        for i in range(0,pairs):
                            #V_obj1=np.where(V_obj==V_obj.min())
                            if V_obj.min()<=v_thres:# and D[V_obj1[0][0],V_obj1[1][0]]<3:# and D.min()<1:
                                D1 = np.where(D==D.min())   
                                UsedRow.append(D1[0][0])
                                UsedCol.append(D1[1][0])
                                D[UsedRow[i]][0:len(Known_i)] = 1000
                                V_obj[UsedRow[i]][0:len(Known_i)] = 1000
                                for j in range(0,len(Current_i)):
                                    D[j][UsedCol[i]] = 1000
                                    V_obj[j][UsedCol[i]] = 1000
                    """
                    
                       
                            
                    """
                        for i in range(0,pairs):
                            print(D, "D")
                            D1 = np.where(D==D.min())
                            UsedRow.append(D1[0][0])
                            UsedCol.append(D1[1][0])
                            D[UsedRow[i]][0:len(Known_i)] = 1000
                            for j in range(0,len(Current_i)):
                                D[j][UsedCol[i]] = 1000
                    """
                    # Updating Known to match current pairs
                    for i in range(len(UsedRow)):
                        Row = UsedRow[i]
                        Col = UsedCol[i]
                        Current_update = self.Current[Current_i[Row]]
                        Known_update = self.Known[Known_i[Col]][self.KnownOrder.get("UID")]
                        self.update(Current_update,Known_update)
                    """
                    for i in UsedRow:
                        for j in UsedCol:
                            Current_update = self.Current[Current_i[i]]
                            Known_update = self.Known[Known_i[j]][self.KnownOrder.get("UID")]
                            self.update(Current_update,Known_update)
                    """

                    # Adding new points not matched with a known points
                    if len(UsedRow) < len(Current_i):
                        New_Points = np.delete(Current_i,[UsedRow])
                        for i in New_Points:
                            self.upgrade(self.Current[i])

                    # Add Occlusion to lost objects
                    if len(UsedCol) < len(Known_i):
                        Lost_Points = np.delete(Known_i,[UsedCol])
                        for i in Lost_Points:
                            self.Known[i][self.KnownOrder.get("Occlusion")] += 1  
                            self.Known[i][self.KnownOrder.get("Current_listing")] = float("nan")
                
                
                # Add Occlusion to all classed not found
                Unseen_Classes = Unique_Known_Classes
                for i in Unique_Classes:
                    try:
                        Unseen_Classes.remove(i) 
                    except ValueError:
                        e = ValueError
                for i in Unseen_Classes:
                    for j in range(0,len(self.Known)):
                        if self.Known[j][self.KnownOrder.get("Class")] == i:
                            self.Known[j][self.KnownOrder.get("Occlusion")] += 1
                            self.Known[j][self.KnownOrder.get("Current_listing")] = float("nan")

    def upgrade(self,Current): #Tilføjet time and vehicle xyz
        ID, UID = self.incID(Current[self.CurrentOrder.get("Class")])
        Known = [UID, ID, Current[self.CurrentOrder.get("Class")], \
                Current[self.CurrentOrder.get("cx")], Current[self.CurrentOrder.get("cy")], \
                Current[self.CurrentOrder.get("Start_x")], Current[self.CurrentOrder.get("Start_y")], \
                Current[self.CurrentOrder.get("End_x")] ,Current[self.CurrentOrder.get("End_y")], \
                Current[self.CurrentOrder.get("Score")], 0, Current[self.CurrentOrder.get("Depth_X")], \
                Current[self.CurrentOrder.get("Depth_Y")], Current[self.CurrentOrder.get("Depth_Z")], \
                Current[self.CurrentOrder.get("Time")], \
                Current[self.CurrentOrder.get("Current_listing")]]   
        self.Known.append(Known)

    def update(self,Current,Known_update):
        #Matching UID
        for i in range(0,len(self.Known)):
            if Known_update == self.Known[i][self.KnownOrder.get("UID")]:
                Knownrow = i
        
        self.Known[Knownrow][self.KnownOrder.get("cx")] = Current[self.CurrentOrder.get("cx")]
        self.Known[Knownrow][self.KnownOrder.get("cy")] = Current[self.CurrentOrder.get("cy")]
        self.Known[Knownrow][self.KnownOrder.get("Start_x")] = Current[self.CurrentOrder.get("Start_x")]
        self.Known[Knownrow][self.KnownOrder.get("Start_y")] = Current[self.CurrentOrder.get("Start_y")]
        self.Known[Knownrow][self.KnownOrder.get("End_x")] = Current[self.CurrentOrder.get("End_x")]
        self.Known[Knownrow][self.KnownOrder.get("End_y")] = Current[self.CurrentOrder.get("End_y")]
        self.Known[Knownrow][self.KnownOrder.get("Score")] = Current[self.CurrentOrder.get("Score")]
        self.Known[Knownrow][self.KnownOrder.get("Depth_X")] = Current[self.CurrentOrder.get("Depth_X")]
        self.Known[Knownrow][self.KnownOrder.get("Depth_Y")] = Current[self.CurrentOrder.get("Depth_Y")]
        self.Known[Knownrow][self.KnownOrder.get("Depth_Z")] = Current[self.CurrentOrder.get("Depth_Z")]
        self.Known[Knownrow][self.KnownOrder.get("Occlusion")] = 0
        self.Known[Knownrow][self.KnownOrder.get("Time")] = Current[self.CurrentOrder.get("Time")] #Tilføjet
        self.Known[Knownrow][self.KnownOrder.get("Current_listing")] = Current[self.CurrentOrder.get("Current_listing")]

    def clear(self):
        self.Current = [] 
        Lost_UID = []
        for i in range(0,len(self.Known)):
            if self.Known[i][self.KnownOrder.get("Occlusion")] == self.OcclusionLimit:
                Lost_UID.append(self.Known[i][self.KnownOrder.get("UID")])
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
            if self.Known[i][self.KnownOrder.get("UID")] == Lost_UID:
                #Lost_P = self.Known[i]
                UID = self.Known[i][self.KnownOrder.get("UID")]
                ID = self.Known[i][self.KnownOrder.get("ID")]
                Class = self.Known[i][self.KnownOrder.get("Class")]
                Lost = [UID, ID, Class]
                self.Lost.append(Lost)
                indexes.append(i)
        for index in sorted(indexes, reverse=True):
            del self.Known[index]

