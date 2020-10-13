import cv2
img=cv2.imread("depth20.png",-1)
cv2.imshow("depth_original",img)
cv2.waitKey(3)
#cv2.destroyAllWindows