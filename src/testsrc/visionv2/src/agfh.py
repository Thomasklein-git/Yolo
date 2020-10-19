from collections import Counter
import numpy as np
import cv2
from sklearn.cluster import KMeans, DBSCAN
from sklearn.preprocessing import StandardScaler
import sensor_msgs.point_cloud2 as pc2


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
    return PC_image

def Sub_pointcloud(PC_image, bboxes):   
    """
    Extract sub images with the size of bounding box from point cloud image

    PC_image: Point cloud image [w,h,3]
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

def k_means_pointcloud(img, bboxes, PC=True, k=3,max_iter=1000,tol=1e-4):
    """
    Performs kmeans on point cloud in order to segment object

    INPUT:
    PC=True use xzy from point cloud
    PC=False use x from point cloud (depth image)
    img = PC_image_bbox_sub_series
    img is a point cloud image with three channels corresponding to x, y and z coordinate for each pixel.
    It must have the shape of [w,h,3]
    Coordinate system according to ZED2

    OUTPUT:
    """
    #https://stackoverflow.com/questions/5124376/convert-nan-value-to-zero?fbclid=IwAR3YAEuY_Iw3BoHqI7-BtsomqIyE0tTikTkB9znfihU0EBdqprjKeHHrjW0
    if PC==True:
        avg_depth_series = []
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
            xcoord=[] 
            for x in range(len(imgre_wo_nan_inf)):
                xcoord.append(imgre_wo_nan_inf[x][0])
            xcoord=np.transpose(np.array([xcoord])) # shape[xcoord,1] 

            a=np.concatenate((label,xcoord),axis=1) # Put label and xcoord (depth) side by side shape[pixel,2]
            b=[] # Depth values from a which is at label_max
            for len_a in range(len(a)):
                if a[len_a,0]==label_max:
                    b.append(a[len_a,1])
            avg_depth=np.mean(b)
            avg_depth_series.append(avg_depth)
            
            # For plotting segmented image 
            depth_distance=[] 
            for depth_coord in range(len(imgre)):
                depth_distance.append(imgre[depth_coord][0])
            depth_distance=np.array(depth_distance)

            labels = np.array(np.empty(len(depth_distance))) # Initialize labels array with the length of nr pixels in img
            labels[:] = np.nan # All index are nan
            labels[np.invert(np.isnan(depth_distance))]=label_for_plot
            labels = np.where(np.isnan(labels), k ,labels)

            labels_img=np.array(labels.reshape(img[i].shape[0],img[i].shape[1],1)) # Convert labels into img one channel
            labels_img_3c=cv2.merge((labels_img,labels_img,labels_img)) # Convert label_img into three channels
            #print(labels_img_3c)

            #Change labels_img_3c so ch 1 is seg1, ch2 is seg2 and ch3 is seg3 for segmented image
            seg1=np.where((labels_img_3c[:,:,0]==0) & (labels_img_3c[:,:,1]==0) & (labels_img_3c[:,:,2]==0))
            seg2=np.where((labels_img_3c[:,:,0]==1) & (labels_img_3c[:,:,1]==1) & (labels_img_3c[:,:,2]==1))
            seg3=np.where((labels_img_3c[:,:,0]==2) & (labels_img_3c[:,:,1]==2) & (labels_img_3c[:,:,2]==2))
            segnan=np.where((labels_img_3c[:,:,0]==3) & (labels_img_3c[:,:,1]==3) & (labels_img_3c[:,:,2]==3))

            segmented_img=labels_img_3c
            segmented_img[seg1]=(1,0,0)
            segmented_img[seg2]=(0,1,0)
            segmented_img[seg3]=(0,0,1)
            segmented_img[segnan]=(0,0,0)
            #print(segmented_img) 
            #print(np.min(segmented_img),"min",np.max(segmented_img),"max")
            
            # Create segmented image with depth data 
            depth_distance_img=np.array(depth_distance.reshape(img[i].shape[0],img[i].shape[1],1))
            #print(depth_distance_img.shape)
            segmented_img1=cv2.extractChannel(segmented_img,0)*depth_distance_img[:,:,0]
            segmented_img2=cv2.extractChannel(segmented_img,1)*depth_distance_img[:,:,0]
            segmented_img3=cv2.extractChannel(segmented_img,2)*depth_distance_img[:,:,0]
            #print(np.nanmin(segmented_img1),"min1",np.nanmin(segmented_img2),"min2",np.nanmin(segmented_img3),"min3")
            #print(np.nanmax(segmented_img1),"max1",np.nanmax(segmented_img2),"max2",np.nanmax(segmented_img3),"max3")
            segmented_img_3c=cv2.merge((segmented_img1,segmented_img2,segmented_img3))
            #print(np.nanmin(segmented_img_3c),"min",np.nanmax(segmented_img_3c),"max")
            segmented_img_3c_norm=NormalizeData(segmented_img_3c) # Normalize segmented image
            #print(np.nanmin(segmented_img_3c_norm),"min",np.nanmax(segmented_img_3c_norm),"max")
            segmented_img_color=segmented_img_3c_norm*255 # Tranfer into RGB
            segmented_img_color[segnan]=(255,255,255) #Makes nan white
            segmented_img_color=segmented_img_color.astype("uint8")
            
            """
            seghej=segmented_img*depth_distance_img
            seghej_norm=NormalizeData(seghej)

            seghej_farver=seghej_norm*255
            seghej_farver[segnan]=(255,255,255)
            seghej_farver=seghej_farver.astype("uint8")
            
            print(np.nanmin(seghej))
            print(np.nanmax(seghej))
            print(seghej_norm)
            print(np.nanmax(seghej_norm),np.nanmin(seghej_norm))
            print(seghej_farver)
            print(np.max(seghej_farver),np.min(seghej_farver))
            """
            
                  
            #print(labels_img_3c.shape)
            #segmented_img=labels_img_3c
            #segmented_img_test=np.where(0,255,cv2.extractChannel(segmented_img,0))
            #print(segmented_img_test.shape)
            #labels_norm=NormalizeData(labels)*255
            #depth_plot_img=np.array(labels_norm.reshape(img[i].shape[0],img[i].shape[1],1),dtype=np.uint8)
            #depth_plot_img_3c=cv2.merge((depth_plot_img,depth_plot_img,depth_plot_img))
            #hej=cv2.extractChannel(depth_plot_img_3c,0)
            #print(depth_plot_img.shape)
            #print(depth_plot_img_3c.shape)
            #print(hej.shape)
            #print(labels,"labels")
            #print(labels_plot,"norm")
            #label_test=np.array(labels.reshape(img[i].shape[0],img[i].shape[1],1))
            #label_test=np.where(0,[0,0,255],labels)
            #print(img[i].shape,"img")
            #print(label_test.shape,"label_test")
            #print(label_for_plot.shape,"label")
            #print(depth_plot.shape, "depth_plot")
            #print(labels.shape, "labels")
            #print(label_for_plot)
            #print(labels)
            #print(max(labels),"max")
            #print(min(labels),"min")

            """
            depth_plot_img=depth_plot.reshape((img[i].shape[0],img[i].shape[1],1))
            print(depth_plot_img.shape)
            print(img[i].shape)
            """
            depth_plot_img=segmented_img_color

    elif PC==False:
        avg_depth_series = []
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
    else:
        print("PC must be True or False")
        avg_depth_series=np.nan

    return avg_depth_series, depth_plot_img

def Simple_Pinhole(P,D):
    '''
    Simple Pinhole Model to calculate physical position based on depth and pixel coordinates 
    for Zed2 left camera FullHD. Numeric.
    Assumed no rotation or translation
    #https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
    Parameters:
        P: Position (Pixel)
        D: Depth (m)
        c: Pricipal point
        f: Focal length
        
    fx=529.085
    fy=528.38
    cx=645.815
    cy=376.6125
    k1=-0.0396842
    k2=0.00917941
    k3=-0.0047467
    p1=0.00010877
    p2=0.000124303
    '''
    f = [529.085,528.38]
    c = [645.815,376.6125]
    R = [[0, 0, 1],[0, 1, 0], [-1 , 0, 0]] # Rotation around y axis

    xm = (P[0]-c[0])/f[0]
    ym = (P[1]-c[1])/f[1]

    x = xm*D
    y = ym*D

    #Lc = [[x],[y],[d]]

    #Gc = Lc*R 
    #print(Gc)   
    
    return x, y , D