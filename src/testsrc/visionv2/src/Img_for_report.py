import cv2
import numpy as np
import tensorflow as tf
from yolov3.configs import *
from yolov3.yolov3 import *
from yolov3.utils import *

show=True
write=False
yolo = Load_Yolo_model()

image_path="./Images_for_report/YOLO_TEST_IMG.png" #Input images that you want to analyze 

original_image = cv2.imread(image_path)
original_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)
original_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)

# Preprocess image
image_preproces = image_preprocess(np.copy(original_image), [YOLO_INPUT_SIZE, YOLO_INPUT_SIZE]) #[416,416,3] float64 (Normalize all pixels between 0 and 1)
image_preproces_1 = image_preproces[np.newaxis, ...].astype(np.float32) #[1,416,416,3] float32

# Predict image
pred_bbox = yolo.predict(image_preproces_1)
pred_bbox = [tf.reshape(x, (-1, tf.shape(x)[-1])) for x in pred_bbox]
pred_bbox = tf.concat(pred_bbox, axis=0) # shape [10647,85] float32

# Post process bboxes
#bboxes_all = postprocess_boxes(pred_bbox, original_image, YOLO_INPUT_SIZE, score_threshold=0) #tres=0 lets 6832bbox through
bboxes_0_05 = postprocess_boxes(pred_bbox, original_image, YOLO_INPUT_SIZE, score_threshold=0.05) 
#bboxes_0_3 = postprocess_boxes(pred_bbox, original_image, YOLO_INPUT_SIZE, score_threshold=0.3) 

# nms bboxes
bboxes_0_05_nms = nms(bboxes_0_05, iou_threshold=0.45, method='nms')
#bboxes_0_3_nms = nms(bboxes_0_3, iou_threshold=0.45, method='nms')

# Draw bboxes on original image
#image_all = draw_bbox(original_image, bboxes_all, CLASSES=YOLO_COCO_CLASSES, show_label=False)#, rectangle_colors=(255,0,0)
#image_0_05 = draw_bbox(original_image, bboxes_0_05, CLASSES=YOLO_COCO_CLASSES, show_label=False)
#image_0_3 = draw_bbox(original_image, bboxes_0_3, CLASSES=YOLO_COCO_CLASSES, show_label=False)
image_0_05_nms = draw_bbox(original_image, bboxes_0_05_nms, CLASSES=YOLO_COCO_CLASSES, show_label=False)


if show==True:
    cv2.imshow("original image",original_image)
    cv2.imshow("image preprocess",image_preproces)
    
    #cv2.imshow("All prediction bboxes",image_all)
    #cv2.imshow("postprocess_0_05",image_0_05)
    #cv2.imshow("postprocess_0_3",image_0_3)
    
    cv2.imshow("postprocess_0_05_with_nms",image_0_05_nms)
    
    # Load and hold the image
    cv2.waitKey(0)
    # To close the window after the required kill value was provided
    cv2.destroyAllWindows()
if write==True:
    #cv2.imwrite("Vis_image_preprocess.png",(image_preproces*255).astype(np.uint8))
    #cv2.imwrite("Vis_yolo_bbox_prediction.png",image_all)
    #cv2.imwrite("Vis_c_times_p_0_05.png",image_0_05)
    #cv2.imwrite("Vis_c_times_p_0_3.png",image_0_3)
    cv2.imwrite("Vis_c_times_p_0_05_with_nms.png",image_0_05_nms)
    



