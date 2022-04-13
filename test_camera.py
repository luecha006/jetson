# import cv2
# print(cv2.__version__)
# width=1280
# height=750
# flip=2
# camSet='nvarguscamerasrc sensor-id=0 ee-mode=1 ee-strength=0 tnr-mode=2 tnr-strength=1 wbmode=0 ! video/x-raw(memory:NVMM), width=3264, height=2464, framerate=21/1,format=NV12 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(width)+', height='+str(height)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! videobalance contrast=1.3 brightness=0 saturation=1.6 ! appsink '
# # camSet='nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=3264, height=2464, framerate=21/1,format=NV12 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(width)+', height='+str(height)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
# # camSet ='v4l2src device=/dev/video1 ! video/x-raw,width='+str(width)+',height='+str(height)+',framerate=20/1 ! videoconvert ! appsink'
# cam=cv2.VideoCapture(camSet)
# while True:
#     _, frame = cam.read()
#     cv2.imshow('myCam',frame)
#     cv2.moveWindow('myCam',0,0)
#     if cv2.waitKey(1)==ord('q'):
#         break
# cam.release()
# cv2.destroyAllWindows()


#---------------------------------

# import cv2

# dispW=800
# dispH=800
# flip=2
# camSet='nvarguscamerasrc sensor-id=0 tnr-mode=2 tnr-strength=1 saturation=1 ee-mode=1 ee-strength=0 gainrange=16 aeantibanding=2  exposurecompensation=0 wbmode=4 !  video/x-raw(memory:NVMM), width=800, height=800, format=NV12, framerate=21/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! videobalance contrast=1.5 brightness=-0.1 saturation=1.5 ! appsink'
# #  exposurecompensation=1  ปรับความสว่าง -2-2
# cam = cv2.VideoCapture(camSet)

# while True:
#     ret, frame = cam.read()
#     print('frame not ',frame.shape)
#     # image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB )
#     # frame = cv2.resize(frame, (650,720))
#     print('frame resize ',frame.shape)
#     cv2.imshow('frame', frame)
#     if cv2.waitKey(1) == ord('q'):
#         break
    
# cam.release()
# cv2.destroyAllWindows()



import cv2
print(cv2.__version__)
cam=cv2.VideoCapture('/dev/video1')
while True:
    _, frame = cam.read()
    frame = cv2.resize(frame, (650,720))
    image_predict = frame[140:-130, 80:-80]
    # image_predict = frame[150:-220, 120:-120]
    # print(image_predict.shape)
    
    cv2.imshow('image_predict',image_predict)
    cv2.moveWindow('myCam',0,0)
    if cv2.waitKey(1)==ord('q'):
        break
cam.release()
cv2.destroyAllWindows()

