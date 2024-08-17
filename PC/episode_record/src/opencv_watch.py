import cv2

# 打开摄像头
cap_1 = cv2.VideoCapture('/dev/video2')
cap_2 = cv2.VideoCapture('/dev/video4')

# 检查摄像头是否成功打开
if not cap_1.isOpened():
    print("无法打开摄像头")
    exit()

# 不断从摄像头读取视频帧
while True:
    # 读取一帧视频
    ret_1, frame_1 = cap_1.read()
    ret_2, frame_2 = cap_2.read()


    # 显示视频帧
    cv2.imshow('inhand', frame_1)
    retval, buf = cv2.imencode('.jpg', frame_1, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
    print(buf.shape)

    cv2.imshow('overview', frame_2)
    retval, buf = cv2.imencode('.jpg', frame_2, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
    print(buf.shape)

    # 按下 'q' 键退出循环
    if cv2.waitKey(1) == ord('q'):
        break

# 释放摄像头
cap_1.release()
cap_2.release()

# 关闭所有OpenCV窗口
cv2.destroyAllWindows()
