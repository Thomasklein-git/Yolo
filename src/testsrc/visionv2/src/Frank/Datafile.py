#!/usr/bin/env python3

import numpy as np
from random import randint
from Object_handler import Object_handler

c1 = float(randint(10,240))
c2 = float(randint(10,240))
c3 = float(randint(10,240))
c4 = float(randint(10,240))
c5 = float(randint(10,240))
c6 = float(randint(10,240))
c7 = float(randint(10,240))
Randombox = []
Randombox2 = []
Randombox3 = []
Randombox.append([c1,c1,c1+10,c1+10,0.8,1.0])
Randombox.append([c2,c2,c2+10,c2+10,0.8,0.0])
Randombox.append([c3,c3,c3+10,c3+10,0.7,1.0])
Randombox.append([c4,c4,c4+10,c4+10,0.7,2.0])

Randombox2.append([c5,c5,c5+10,c5+10,0.7,1.0])
Randombox2.append([c6,c6,c6+10,c6+10,0.7,0.0])
Randombox2.append([c7,c7,c7+10,c7+10,0.7,0.0])

#Randombox3.append([c7,c7,c7+10,c7+10,0.7,2.0])

Randombox=np.array(Randombox)
Randombox2=np.array(Randombox2)
Randombox3=np.array(Randombox3)
OH = Object_handler(3)

OH.add(Randombox)
OH.add(Randombox2)

