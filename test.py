


#import cv2

 
#opencv DNN
#net = cv2.dnn.readNet()

#cap = cv2.VideoCapture(0)

#while True:
 # ret, frame = cap.read()
  #cv2.imshow("frame", frame)
  #cv2.waitKey(1)






#import cv2 
#import numpy as np 
#from tracker  import *




#tracker =EuclideanDistTracker()


# need pass of source video file 
#cap = cv2.VideoCapture() 


# object detection from stable camera 
#object_detector = cv2.createBackgroundSubtractorMOG2(history=100, varThreshold=40 )

#while True: 
  #ret, frame = cap.read()
 # height , width ,_ = frame.shape 
   
   # Extract region of interest  
  #roi = frame[340: 720,500: 800]
  
  # to give a blackbaground # object detection  
  #mask = object_detector.apply(roi)
  #_,mask = cv2.threshold(mask, 254, 255, cv2.INTER_NEAREST)
  #contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  #detections = []
  #for cnt in contours:
    
    # calculate area and remove small elements 
   # area = cv2.contourArea(cnt)
    #if area >100:
     # x, y, w, h = cv2.boundingRect(cnt)

      #detections.append([x,y, w, h ])
      
  # 2. object tracking 
  #boxes_ids = tracker.update(detections)
  #for box_id in boxes_ids:
   # x, y, w, h, id =box_id
    
    #cv2.putText(roi,str(id), (x, y - 15 ), cv2.FONT_HERSHEY_PLAIN, 2, (255,0, 0), 2)
   # cv2.rectangle(roi,(x,y), (x + w, y +h), (0, 255,0), 3 )

  
  
  #print(detections)
  
 # cv2.imshow("roi",roi)
 # cv2.imshow("frame", frame)
 # cv2.imshow("mask", mask)
 # 
 # key = cv2.waitKey(30)
  #if key == 27: 
 #   break
  
  #cap.release()






# if the video open 
#if (video.isOpened()):
 # print ("Good")
 #fps = video.get(cv2.CAP_PROP_FPS)
 #frame_count= video.get (cv2.CAP_PROP_FRAME_COUNT)
 #width = video.get(cv2.CAP_PROP_FRAME_WIDTH)
 #height  = video.get(cv2.CAP_PROP_FRAME_HEIGHT)
 #print(fps)
 #print(frame_count)
 #print(width)
 #print(height)
 #frame_size =(width, height)
 
 # to write the video 
 #video_out = cv2.VideoWriter("webcam_gray.avi", cv2.VideoWriter_fourcc("M", "3", "P", "G"), 
 #                           24, 
 #                          frame_size )
 
#else:
  #print("Not Good ")

#while (video.isOpened()):
 # ret, frame = video.read()
  #if ret == True:
   # cv2.imshow("video Frame" , frame)
    #key = cv2.waitKey(20)
    #if key == ord("q"):
     # break
    #else: 
     # break
  
# to release the object 
#video.release()





#import cv2 
#import numpy as np 

#image = 150 * np.ones((400,600,3), dtype = np.uint8) 

#cv2.ellipse(image, (300,200),(150, 150 ), 0, 100, 360, (0, 0, 255), -1 )
#cv2.ellipse(image, (300,200),(150, 150), 0, 0, 180, (255, 255, 255), -1 )

#cv2.line(image, (155, 200),(445, 200), (0, 0, 0), 15)

#cv2.circle(image, (300, 200),50, (0, 0, 0 ), -1)
#cv2.circle(image, (300, 200),30, (255, 255, 255 ), -1)

#cv2.rectangle(image, (150, 20),(450,450), (0, 255, 255), 5)
#cv2.rectangle(image, (150, 50),(450,20), (0, 255, 255), -1)
#cv2.putText(image, "pokeball",(160,45), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 3  )

#cv2.imshow("image", image)
#cv2.waitKey(0)
#cv2.destroyAllWindows()










# object detection 
#######
#import cv2 
#import cvlib as cv 
#from cvlib.object_detection import draw_bbox
#from gtts import gTTs
#from playsound import playsound

# acsses to cammera 

#video = cv2.VideoCapture(1)

# to use video capture 
#while True: 
 # ret, frame = video.read()
  #bbox, label, conf = cv.detect_common_objects(frame)
  #output_image = draw_bbox(frame, bbox, label, conf)
  
  
  #cv2.imshow("Object detection", output_image)
  #if cv2.waitKey(1) & 0xFF == ord ("q"):
  # break 




import cv2

cam = cv2.VideoCapture(0)

cv2.namedWindow("test")

img_counter = 0


while True:
   ret,frem = cam.read()
    
   if not ret:
       print("failed")
       break
cv2.imshow("test", frem)
     
k = cv2.waitkey(1)
    
if k%256 ==27: 
  print("Escape hit,  closing app")

  
elif k%256 == 32:
       img_name  = "opencv_frem_{}.png",format(img_counter)
       cv2.imwrite(img_name,frem)
       print("screenshot taken")
       img_counter += 1
        

cam.release()
cam.destroyAllWindows()