from collections import Counter
import numpy as np
import cv2
from sklearn.cluster import KMeans, DBSCAN
from sklearn.preprocessing import StandardScaler
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
import rospy
import tf2_ros
import tf2_geometry_msgs
from tf import TransformListener


def Give_boundingbox_coor_class(bboxes):
    x1=[] # Top left x-coor
    y1=[] # Top left y-coor
    x2=[] # Bottom right x-coor
    y2=[] # Bottom right y-coor
    Score=[] # Class probabilities times objectness score
    C=[] # Class
    for i in range(len(bboxes)):
        boundingbox=bboxes[i]
        boundingbox_int=boundingbox.astype(int)
        x1.append(boundingbox_int[0])
        y1.append(boundingbox_int[1])
        x2.append(boundingbox_int[2])
        y2.append(boundingbox_int[3])
        Score.append(boundingbox[4])
        C.append(boundingbox_int[5])
    return x1, y1, x2, y2, Score, C

def PC_dataxyz_to_PC_image(data,Org_img_height=720,Org_img_width=1280):
    """
    Extract xyz coordinates from point cloud and transform them an image

    data: pointcloud from ZED2 
    Org_img_height: Height of image defined in ZED2
    Org_img_width: Wigth of image defined in ZED2
    """
    pc = pc2.read_points(data, skip_nans=False, field_names=("x", "y", "z"))
    pc_list = []
    for p in pc:
        pc_list.append([p[0],p[1],p[2]]) # Extract x,y and z coordinates from point cloud
    pc_list=np.array(pc_list, dtype='float32') # cv2.getRectSubPix need float32 (Sub_pointcloud function)
    PC_image=pc_list.reshape((Org_img_height,Org_img_width,3)) # Transform list into image
    return pc_list, PC_image

def Sub_pointcloud(PC_image, bboxes):   
    """
    Extract sub images with the size of bounding box from point cloud image

    PC_image: Point cloud image [h,w,3]
    bboxes: Bounding boxes from yolov3
    """
    PC_image_bbox_sub_series = []
    x1, y1, x2, y2, _, _ = Give_boundingbox_coor_class(bboxes)
    for i in range(len(bboxes)):
        patch=(int(x2[i]-x1[i]),int(y2[i]-y1[i])) # gives width and height of bbox
        center=(int(x1[i]+patch[0]/2),int(y1[i]+patch[1]/2)) # gives center coodintes of bbox global
        PC_image_bbox_sub = cv2.getRectSubPix(PC_image,patch,center) # Extract bbox in depth image
        PC_image_bbox_sub_series.append(PC_image_bbox_sub)

    return PC_image_bbox_sub_series

def NormalizeData(data):
    return (data - np.nanmin(data)) / (np.nanmax(data) - np.nanmin(data))

def DBSCAN_pointcloud(img, bboxes, seg_plot=True, eps=0.046, procent=0.0011):
    labels_series = []
    avg_depth_series = []
    segmented_img_color_series = []
    xyzcoord_series =[]
    for i in range(len(bboxes)):
        imgre=img[i].reshape((-1,3)) # Flatten the image (pixel,3)
        min_samples=int(len(imgre)*procent)
        imgre_wo_nan = imgre[~np.isnan(imgre).any(axis=1)] # remove rows with nan
        imgre_wo_nan_inf = imgre_wo_nan[~np.isinf(imgre_wo_nan).any(axis=1)] # remove rows with inf
        # Perform kmeans with xyz data
        imgre_scale = StandardScaler().fit_transform(imgre_wo_nan_inf)
        DBSCAN_scale = DBSCAN(eps=eps, min_samples=min_samples).fit(imgre_scale)
        # Calculation of average depth
        label=DBSCAN_scale.labels_
        label_for_plot=np.array(label) #[label,]
        label=np.transpose(np.asarray([label])) # In order to concatenate shape[label,1]
        Sort=Counter(label.flatten()).most_common() 
        label_max=Sort[0][0]

        xcoord=[]
        ycoord=[]
        zcoord=[]
        for xyz in range(len(imgre_wo_nan_inf)):
            xcoord.append(imgre_wo_nan_inf[xyz][0])
            ycoord.append(imgre_wo_nan_inf[xyz][1])
            zcoord.append(imgre_wo_nan_inf[xyz][2])
        xcoord=np.transpose(np.array([xcoord])) # shape[xcoord,1] 
        ycoord=np.transpose(np.array([ycoord])) # shape[ycoord,1] 
        zcoord=np.transpose(np.array([zcoord])) # shape[zcoord,1]

        a=np.concatenate((label,xcoord,ycoord,zcoord),axis=1) # Put label xcoord,ycoord,zcoord side by side shape[pixel,4]
        x_clust=[] # x coordinates from a which is at label_max
        y_clust=[] # y coordinates from a which is at label_max
        z_clust=[] # z coordinates from a which is at label_max
        for len_a in range(len(a)):
            if a[len_a,0]==label_max:
                x_clust.append(a[len_a,1])
                y_clust.append(a[len_a,2])
                z_clust.append(a[len_a,3])
        min_x=np.min(x_clust)
        avg_y=np.mean(y_clust)
        avg_z=np.mean(z_clust)
        
        """
        a=np.concatenate((label,xcoord),axis=1) # Put label and xcoord (depth) side by side shape[pixel,2]
        b=[] # Depth values from a which is at label_max
        for len_a in range(len(a)):
            if a[len_a,0]==label_max:
                b.append(a[len_a,1])
        
        min_x=np.min(b) # find minimum depth in object
        min_index=np.where(xcoord[:,0]==min_x) # Find index of min x in xcoord
        y_for_min_x = ycoord[min_index[0][0],0] # get y from min_index
        z_for_min_x = zcoord[min_index[0][0],0] # get z from min_index
            
        xyzcoord=[min_x,y_for_min_x,z_for_min_x]
        """
        
        
        avg_depth=np.mean(x_clust)
        xyzcoord=[avg_depth,avg_y,avg_z]
        #xyzcoord=[min_x,avg_y,avg_z]

        xyzcoord_series.append(xyzcoord)
        avg_depth_series.append(avg_depth)
        
        if seg_plot==True:
            depth_distance=[] 
            for depth_coord in range(len(imgre)):
                depth_distance.append(imgre[depth_coord][0])
            depth_distance=np.array(depth_distance)
            
            labels = np.array(np.empty(len(depth_distance))) # Initialize labels array with the length of nr pixels in img
            labels[:] = np.nan # All index are nan
            labels[np.isfinite(depth_distance)]=label_for_plot
            labels = np.where(np.isnan(labels), 1000 ,labels) # Set nan to 1000
            labels = np.where(np.isinf(labels), 1000 ,labels) # Set +- inf to 1000
            for clust in range(-1,len(Sort)-1,1): # Set all clusters which are not the largest cluster to 2000
                if clust != label_max:
                    labels = np.where(labels==clust,2000,labels)
            
            labels_img=np.array(labels.reshape(img[i].shape[0],img[i].shape[1],1)) # Convert labels into img one channel
            labels_img_3c=cv2.merge((labels_img,labels_img,labels_img)) # Convert label_img into three channels

            #Change labels_img_3c so ch 2 is max clust, ch3 is rest for segmented image
            seg1=np.where((labels_img_3c[:,:,0]==label_max))
            seg3=np.where((labels_img_3c[:,:,0]==2000))
            segnaninf=np.where((labels_img_3c[:,:,0]==1000))

            segmented_img=labels_img_3c
            segmented_img[seg1]=(0,1,0)
            segmented_img[seg3]=(0,0,1)
            segmented_img[segnaninf]=(0,0,0)
            
            # Create segmented image
            segmented_img_color=segmented_img*255 # Tranfer into RGB
            segmented_img_color[segnaninf]=(255,255,255) #Makes nan and inf white
            segmented_img_color=segmented_img_color.astype("uint8") # Change type
            segmented_img_color_series.append(segmented_img_color)


            tf_labels = np.where((labels == label_max), True, False)
            labels_series.append(tf_labels)
            
            

    return avg_depth_series, segmented_img_color_series, xyzcoord_series, labels_series

def k_means_pointcloud(img, bboxes, PC=True, seg_plot=True, k=3,max_iter=1000,tol=1e-4):
    """
    Performs kmeans on point cloud in order to segment object

    INPUT:
    seg_plot=True output segmentation image (works only for k=3)
    seg_plot=False does not output segmentation image 
    PC=True use xyz from point cloud
    PC=False use x from point cloud (depth image)
    img = PC_image_bbox_sub_series
    img is a point cloud image with three channels corresponding to x, y and z coordinate for each pixel.
    It must have the shape of [h,w,3]
    Coordinate system according to ZED2

    OUTPUT:

    """
    if PC==True:
        avg_depth_series = []
        segmented_img_color_series = []
        xyzcoord_series =[]
        for i in range(len(bboxes)):
            imgre=img[i].reshape((-1,3)) # Flatten the image (pixel,3)
            imgre_wo_nan = imgre[~np.isnan(imgre).any(axis=1)] # remove rows with nan
            imgre_wo_nan_inf = imgre_wo_nan[~np.isinf(imgre_wo_nan).any(axis=1)] # remove rows with inf
            # Perform kmeans with xyz data
            imgre_scale = StandardScaler().fit_transform(imgre_wo_nan_inf)
            KM_scale = KMeans(n_clusters=k, max_iter=max_iter, tol=tol, random_state=0).fit(imgre_scale)
            # Calculation of average depth
            label=KM_scale.labels_
            label_for_plot=np.array(label) #[label,]
            label=np.transpose(np.asarray([label])) # In order to concatenate shape[label,1]
            Sort=Counter(label.flatten()).most_common() 
            label_max=Sort[0][0]
            # Extraxt depth data from point cloud
            xcoord=[] #depth
            ycoord=[]
            zcoord=[]
            for xyz in range(len(imgre_wo_nan_inf)):
                xcoord.append(imgre_wo_nan_inf[xyz][0])
                ycoord.append(imgre_wo_nan_inf[xyz][1])
                zcoord.append(imgre_wo_nan_inf[xyz][2])
            xcoord=np.transpose(np.array([xcoord])) # shape[xcoord,1] 
            ycoord=np.transpose(np.array([ycoord])) # shape[ycoord,1] 
            zcoord=np.transpose(np.array([zcoord])) # shape[zcoord,1]

            a=np.concatenate((label,xcoord),axis=1) # Put label and xcoord (depth) side by side shape[pixel,2]
            b=[] # Depth values from a which is at label_max
            for len_a in range(len(a)):
                if a[len_a,0]==label_max:
                    b.append(a[len_a,1])
            
            min_x=np.min(b) # find minimum depth
            min_index=np.where(xcoord[:,0]==min_x) # Find index of min x in xcoord
            y_for_min_x = ycoord[min_index[0][0],0] # get y from min_index
            z_for_min_x = zcoord[min_index[0][0],0] # get z from min_index
            
            xyzcoord=[min_x,y_for_min_x,z_for_min_x]
            xyzcoord_series.append(xyzcoord)

            avg_depth=np.mean(b)
            avg_depth_series.append(avg_depth)
            
            #%% For plotting segmented image (work only for k=3)
            if seg_plot==True:
                depth_distance=[] 
                for depth_coord in range(len(imgre)):
                    depth_distance.append(imgre[depth_coord][0])
                depth_distance=np.array(depth_distance)

                labels = np.array(np.empty(len(depth_distance))) # Initialize labels array with the length of nr pixels in img
                labels[:] = np.nan # All index are nan
                labels[np.invert(np.isnan(depth_distance))]=label_for_plot
                labels = np.where(np.isnan(labels), k ,labels) # set nan to category 3
                labels = np.where(np.isinf(labels), k ,labels) # set +- inf to category 3
                labels_img=np.array(labels.reshape(img[i].shape[0],img[i].shape[1],1)) # Convert labels into img one channel
                labels_img_3c=cv2.merge((labels_img,labels_img,labels_img)) # Convert label_img into three channels

                #Change labels_img_3c so ch 1 is seg1, ch2 is seg2 and ch3 is seg3 for segmented image
                seg1=np.where((labels_img_3c[:,:,0]==0))# & (labels_img_3c[:,:,1]==0) & (labels_img_3c[:,:,2]==0))
                seg2=np.where((labels_img_3c[:,:,0]==1))# & (labels_img_3c[:,:,1]==1) & (labels_img_3c[:,:,2]==1))
                seg3=np.where((labels_img_3c[:,:,0]==2))# & (labels_img_3c[:,:,1]==2) & (labels_img_3c[:,:,2]==2))
                segnaninf=np.where((labels_img_3c[:,:,0]==3))# & (labels_img_3c[:,:,1]==3) & (labels_img_3c[:,:,2]==3))

                segmented_img=labels_img_3c
                segmented_img[seg1]=(1,0,0)
                segmented_img[seg2]=(0,1,0)
                segmented_img[seg3]=(0,0,1)
                segmented_img[segnaninf]=(0,0,0)
            
                # Create segmented image with depth data 
                #depth_distance_img=np.array(depth_distance.reshape(img[i].shape[0],img[i].shape[1],1))

                #segmented_img_3c=segmented_img*depth_distance_img
                #segmented_img_3c_norm=NormalizeData(segmented_img_3c) # Normalize segmented image
                segmented_img_color=segmented_img*255 # Tranfer into RGB
                segmented_img_color[segnaninf]=(255,255,255) #Makes nan and inf white
                segmented_img_color=segmented_img_color.astype("uint8")
                segmented_img_color_series.append(segmented_img_color)
    
    elif PC==False:
        avg_depth_series = []
        segmented_img_color_series = []
        xyzcoord_series =[]
        for i in range(len(bboxes)):
            imgre=img[i].reshape((-1,3)) # Flatten the image (pixel,3)
            imgre_wo_nan = imgre[~np.isnan(imgre).any(axis=1)] # remove rows with nan
            imgre_wo_nan_inf = imgre_wo_nan[~np.isinf(imgre_wo_nan).any(axis=1)] # remove rows with inf
            # Extraxt depth data from point cloud
            xcoord=[] 
            for x in range(len(imgre_wo_nan_inf)):
                xcoord.append(imgre_wo_nan_inf[x][0])
            xcoord=np.transpose(np.array([xcoord])) # shape[xcoord,1]
            # Perform kmeans with x data 
            imgre_scale = StandardScaler().fit_transform(xcoord)
            KM_scale = KMeans(n_clusters=k, max_iter=max_iter, tol=tol, random_state=0).fit(imgre_scale)
            # Calculation of average depth
            label=KM_scale.labels_
            label_for_plot=np.array(label) #[label,]
            label=np.transpose(np.asarray([label])) # In order to concatenate shape[label,1]
            Sort=Counter(label.flatten()).most_common() 
            label_max=Sort[0][0]

            a=np.concatenate((label,xcoord),axis=1) # Put label and xcoord (depth) side by side shape[pixel,2]
            b=[] # Depth values from a which is at label_max
            for len_a in range(len(a)):
                if a[len_a,0]==label_max:
                    b.append(a[len_a,1])
            avg_depth=np.mean(b)
            avg_depth_series.append(avg_depth)

            #%% For plotting segmented image (work only for k=3)
            if seg_plot==True:
                depth_distance=[] 
                for depth_coord in range(len(imgre)):
                    depth_distance.append(imgre[depth_coord][0])
                depth_distance=np.array(depth_distance)

                labels = np.array(np.empty(len(depth_distance))) # Initialize labels array with the length of nr pixels in img
                labels[:] = np.nan # All index are nan
                labels[np.invert(np.isnan(depth_distance))]=label_for_plot
                labels = np.where(np.isnan(labels), k ,labels) # set nan to category 3
                labels = np.where(np.isinf(labels), k ,labels) # set +- inf to category 3
                labels_img=np.array(labels.reshape(img[i].shape[0],img[i].shape[1],1)) # Convert labels into img one channel
                labels_img_3c=cv2.merge((labels_img,labels_img,labels_img)) # Convert label_img into three channels

                #Change labels_img_3c so ch 1 is seg1, ch2 is seg2 and ch3 is seg3 for segmented image
                seg1=np.where((labels_img_3c[:,:,0]==0))# & (labels_img_3c[:,:,1]==0) & (labels_img_3c[:,:,2]==0))
                seg2=np.where((labels_img_3c[:,:,0]==1))# & (labels_img_3c[:,:,1]==1) & (labels_img_3c[:,:,2]==1))
                seg3=np.where((labels_img_3c[:,:,0]==2))# & (labels_img_3c[:,:,1]==2) & (labels_img_3c[:,:,2]==2))
                segnaninf=np.where((labels_img_3c[:,:,0]==3))# & (labels_img_3c[:,:,1]==3) & (labels_img_3c[:,:,2]==3))

                segmented_img=labels_img_3c
                segmented_img[seg1]=(1,0,0)
                segmented_img[seg2]=(0,1,0)
                segmented_img[seg3]=(0,0,1)
                segmented_img[segnaninf]=(0,0,0)
            
                # Create segmented image with depth data 
                depth_distance_img=np.array(depth_distance.reshape(img[i].shape[0],img[i].shape[1],1))

                segmented_img_3c=segmented_img*depth_distance_img
                segmented_img_3c_norm=NormalizeData(segmented_img_3c) # Normalize segmented image
                segmented_img_color=segmented_img_3c_norm*255 # Tranfer into RGB
                segmented_img_color[segnaninf]=(255,255,255) #Makes nan and ing white
                segmented_img_color=segmented_img_color.astype("uint8")
                segmented_img_color_series.append(segmented_img_color)
    
    else:
        print("PC must be True or False")
        avg_depth_series=np.nan
        segmented_img_color_series = []
        xyzcoord_series = np.nan

    return avg_depth_series, segmented_img_color_series, xyzcoord_series

def PC_reduc(Target,TargetOrder, pc_list, cloud):
    if Target == None:
        Reduced_PC2 = cloud
    else:
        Start_x = Target[TargetOrder("Start_x")]
        Start_y = Target[TargetOrder("Start_y")]
        End_x   = Target[TargetOrder("End_x")]
        End_y   = Target[TargetOrder("End_y")]

        bbox_i = []
        for y in range(Start_y,End_y):
            bbox_i += list(range((y*672+Start_x)*3,(y*672+End_x+1)*3))
        #print(len(bbox_i), "bbox_i")
        pc_list = np.delete(pc_list, bbox_i)
        pc_list = pc_list.reshape(int(len(pc_list)/3),3)
        pc_list= pc_list[~np.isnan(pc_list).any(axis=1)]
        pc_list= pc_list[~np.isinf(pc_list).any(axis=1)]
        header = cloud.header
        Reduced_PC2 = pc2.create_cloud_xyz32(header, pc_list)
    return Reduced_PC2

def PC_reduc_seg(bbox, Segmented_labels ,pc_list,cloud):
    if bbox == []:
        reduced_PC2 = cloud
    else:
        """
        Start_x = Target[TargetOrder("Start_x")]
        Start_y = Target[TargetOrder("Start_y")]
        End_x   = Target[TargetOrder("End_x")]
        End_y   = Target[TargetOrder("End_y")]

        """
        Start_x = bbox[0]
        End_x   = bbox[2]
        Start_y = bbox[1]
        End_y   = bbox[3]

        bbox_i = []
        Seg_index = 0
        for y in range(Start_y,End_y):
            for x in range(Start_x,End_x):
                pc_index = (y*672+x)*3
                if Segmented_labels[Seg_index] == True:
                    for i in [0,1,2]:
                        bbox_i.append(pc_index+i)
                Seg_index += 1
        #print(pc_list.shape)
        pc_list = np.delete(pc_list, bbox_i)
        #print(pc_list.shape)
        pc_list = pc_list.reshape(int(len(pc_list)/3),3)
        pc_list= pc_list[~np.isnan(pc_list).any(axis=1)]
        pc_list= pc_list[~np.isinf(pc_list).any(axis=1)]
        header = cloud.header
        Reduced_PC2 = pc2.create_cloud_xyz32(header, pc_list)
    return Reduced_PC2

def Waypoint_planter(Target, TargetOrder, Frame_id, Time):
    Pose = PoseStamped()
    Pose.header.stamp = Time
    Pose.header.frame_id = Frame_id #"zed2_left_camera_frame"
    Pose.pose.position.x = Target[TargetOrder("Depth_X")]
    Pose.pose.position.y = Target[TargetOrder("Depth_Y")]
    Pose.pose.position.z = Target[TargetOrder("Depth_Z")]
    Pose.pose.orientation.x = float(0)
    Pose.pose.orientation.y = float(0)
    Pose.pose.orientation.z = float(0)
    Pose.pose.orientation.w = float(1)
    return Pose
    
def Choose_target(OH, Target_class):
    # Find the UID associated with the first found target from Target_class
    fp = True
    Target_Found = False
    Target_UID = []
    for Target in OH.Known:
        if Target[OH.KnownOrder.get("Class")] == Target_class and fp == True:
            Target_UID = Target[OH.KnownOrder.get("UID")]
            Target_Found = True
            fp == False
    return Target_UID, Target_Found
    
def Find_target(OH, Target_UID):
    Index = []
    Listing = []
    #Occlusion = []
    
    for i in range(0,len(OH.Known)):
        if OH.Known[i][OH.KnownOrder.get("UID")] == Target_UID:
            Index = i
            Occlusion = OH.Known[i][OH.KnownOrder.get("Occlusion")]
            break
    if Index == []:
        print("Target is Lost")
    else:
        if Occlusion > 0:
            print("Target was occluded {} frames ago".format(Occlusion))
        else:
           Listing = OH.Known[Index][OH.KnownOrder.get("Current_listing")] 
    return Listing

def Unique_in_List(List):  
    Unique_Entries = []
    for x in List: 
        if x not in Unique_Entries: 
            Unique_Entries.append(x)
    return Unique_Entries

def Create_PoseStamped_msg(coordinates, coordinate_frame, Time):
    Pose = PoseStamped()
    Pose.header.stamp = Time
    Pose.header.frame_id = coordinate_frame
    Pose.pose.position.x = coordinates[0]
    Pose.pose.position.y = coordinates[1]
    Pose.pose.position.z = coordinates[2]
    Pose.pose.orientation.x = float(0)
    Pose.pose.orientation.y = float(0)
    Pose.pose.orientation.z = float(0)
    Pose.pose.orientation.w = float(1)
    
    return Pose

def Transform_Pose_between_frames(coordinates,Current_frame,Target_frame,Time):
    """
    Input:
    coordinates=[x, y, z]
    Current frame = name of current frame (format: string)
    Target frame = name of target frame (format: string)
    Time = Timestamp for PoseStamped msg (format: float)
    """
    Pose = Create_PoseStamped_msg(coordinates,Current_frame,Time)
    # Transform pose to target frame
    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    transform = tf_buffer.lookup_transform(Target_frame, Pose.header.frame_id, rospy.Time(0), rospy.Duration(20.0))
    Trans_Pose = tf2_geometry_msgs.do_transform_pose(Pose, transform)
    
    return Trans_Pose

def Transform_Coordinates_between_frames(xyzcoord_series, Current_frame,Target_frame,Time):
    xyzcoord_trans_series = []
    for i in range(len(xyzcoord_series)):
        Trans_pose = Transform_Pose_between_frames(xyzcoord_series[i], Current_frame, Target_frame, Time)	
        Trans_coord = [Trans_pose.pose.position.x,Trans_pose.pose.position.y,Trans_pose.pose.position.z]
        xyzcoord_trans_series.append(Trans_coord)
    
    return xyzcoord_trans_series

def get_new_orientation(Waypoint_old, Waypoint_new, vec_old_new, Points=True):
    #https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
    if Points == True:
        Waypoint_old_xyz = np.array([Waypoint_old.pose.position.x,Waypoint_old.pose.position.y,Waypoint_old.pose.position.z])
        Waypoint_new_xyz = np.array([Waypoint_new.pose.position.x,Waypoint_new.pose.position.y,Waypoint_new.pose.position.z])
        vec_old_new = Waypoint_new_xyz-Waypoint_old_xyz

    Vec_ref = np.array([1,0,0]) 
    Complex_ele = np.cross(Vec_ref,vec_old_new) # X, Y, Z vector part
    if np.dot(Vec_ref,vec_old_new/np.linalg.norm(vec_old_new)) < -0.999999 and np.linalg.norm(Complex_ele) == 0:
        q_norm = np.array([0,0,1,0])
    else:
        Real_ele = np.sqrt((np.linalg.norm(Vec_ref)**2)*(np.linalg.norm(vec_old_new)**2))+np.dot(Vec_ref,vec_old_new) # W scalar part
        q=np.append(Complex_ele,Real_ele)
        q_norm=q/np.linalg.norm(q)
    
    return q_norm

def read_class_names(class_file_name):
    # loads class name from a file
    names = {}
    with open(class_file_name, 'r') as data:
        for ID, name in enumerate(data):
            names[ID] = name.strip('\n')
    return names

def cal_pose_stop(pose_goal,pose_vehicle,distance_keep):
    vec_v2g=pose_goal-pose_vehicle #vec from vehicle to goal
    vec_v2g_stop=vec_v2g/np.linalg.norm(vec_v2g)*distance_keep #vector from goal to stop
    pose_goal_stop=pose_goal-vec_v2g_stop
    return pose_goal_stop
