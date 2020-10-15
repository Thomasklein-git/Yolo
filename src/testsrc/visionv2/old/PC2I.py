#!/usr/bin/env python3
'''
This script can convert a point cloud data to coordinate positions, a dpeth image and an RGB image.
'''
import sys
import rospy
import cv2
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

import ctypes
import struct


class PC2I():
    def __init__(self):
        cloud_sub = rospy.Subscriber("/zed2/zed_node/point_cloud/cloud_registered",PointCloud2,self.callback_cloud)

    def callback_cloud(self,data):
        """
        fields = []
        fields.name =["x", "y", "z", "r"]
        name    = ["x", "y", "z", "r"]
        offset  = [0,4,8,12]
        datatype= [7,7,7,1]
        count   = [1,1,1,1]
        fields.name = name
        #data.fields.name = name
        #data.fields.offset = offset
        #data.fields.datatype = datatype
        #data.fields.count = count
        """
        
        
        #colors = {"name": "r", "offset": 12, "datatype": 1, "count": 1}
        #print(colors)
        datastack = pc2.read_points(data, skip_nans=False)#, field_names=("x", "y", "z", "rgb")
        for point in datastack:
            s = struct.pack('>f', point[3])
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000) >> 16
            g = (pack & 0x0000FF00) >> 8
            b = (pack & 0x000000FF)
            #img.append([r,b,g])

            #img[k,:,0] = (pack & 0x00FF0000) >> 16
            #img[k,:,1] = (pack & 0x0000FF00) >> 8
            #img[k,:,2] = (pack & 0x000000FF)
        print(r,g,b, "1")


        data.fields[0].name = "r"
        data.fields[0].datatype = 2
        data.fields[0].offset = 12
        data.fields[1].name = "b"
        data.fields[1].datatype = 2
        data.fields[1].offset = 13
        data.fields[2].name = "g"
        data.fields[2].datatype = 2
        data.fields[2].offset = 14
        data.fields[3].name = "a"
        data.fields[3].datatype = 2
        data.fields[3].offset = 15


        
        for point in datastack:
            r = point[0]
            g = point[1]
            b = point[2]
            a = point[3]
        print(r,g,b, "2")

        #x, y , z = self.get_xyz(datastack)


        #P = struct.unpack('fff',data.data[0:12])
        #P = struct.unpack('i',data.data[12:16])
        """
        print(P)

        
        for point in datastack:
            s = struct.pack('>f', point[3])
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000) >> 16
            g = (pack & 0x0000FF00) >> 8
            b = (pack & 0x000000FF)
            #img.append([r,b,g])

            img[k,:,0] = (pack & 0x00FF0000) >> 16
            img[k,:,1] = (pack & 0x0000FF00) >> 8
            img[k,:,2] = (pack & 0x000000FF)
            k += 1

        #print(data.data[0:16])
        #data.data
        #print(len(data.data))

        P = data.data[3]
        print(P)
        
        #x = np.array(data.data[0:4], dtype=np.uint8)
        #x_f = x.veiw(dtype=np.float32)
        #print(x_f)
        #for point in datastack
        #    x = point[0]
        
        #for pc in datastack:
            #print(pc)
        k = 0
        img = np.zeros((921600,1,4),dtype=np.int8)
        W = 1280
        H = 720
            #k = 0
        for rgb in datastack:
            s = struct.pack('>f', rgb[3])
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000) >> 16
            g = (pack & 0x0000FF00) >> 8
            b = (pack & 0x000000FF)
            #img.append([r,b,g])
            img[k,:,3] = (pack & 0xFF000000) >> 24
            img[k,:,0] = (pack & 0x00FF0000) >> 16
            img[k,:,1] = (pack & 0x0000FF00) >> 8
            img[k,:,2] = (pack & 0x000000FF)
            k += 1
        img = np.asarray(img).astype(np.int8)
        img = img.reshape(720,1280,4)
        r = img[:,:,0]
        print(r.shape)
        cv2.imshow("r", img)
        cv2.waitKey(3)
        

    #def get_rgb(self):
    #    return r, g, b

    def get_xyz(self,datastack):
        for point in datastack:

        return x, y ,z
    """

def main(args):
    PC = PC2I()
    
    rospy.init_node('PC2I', anonymous=True)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows

if __name__ =='__main__':
    main(sys.argv)