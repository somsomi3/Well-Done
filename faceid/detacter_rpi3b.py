import cv2
import numpy as np
import os

recognizer = cv2.face.LBPHFaceRecognizer_create()
recognizer.read('trainer/trainer.yml')
detector = cv2.CascadeClassifier("./haarcascade_frontalface_default.xml")
font = cv2.FONT_HERSHEY_SIMPLEX

# 이름 캐시 구현
def get_names_from_files():
    names = ['unknown'] * 200
    userdata_path = 'userdata'
    if os.path.exists(userdata_path):
        for filename in os.listdir(userdata_path):
            if filename.startswith('User_'):
                # User_asdf_이름_숫자.jpg 형식에서 이름 추출
                parts = filename.split('_')
                if len(parts) >= 3:
                    user_id = parts[1]  # asdf
                    user_name = parts[2]  # 이름
                    # 파일 번호를 인덱스로 사용
                    file_number = int(parts[3].split('.')[0])  # 숫자
                    names[file_number] = f"{user_id}_{user_name}"
    return names
names = get_names_from_files()  # 이전 코드의 함수 사용

# 카메라 설정 최적화
cap = cv2.VideoCapture(0)
cap.set(3, 640)  # 해상도를 낮춤 (1280 -> 640)
cap.set(4, 480)  # 해상도를 낮춤 (720 -> 480)

# 처리 프레임 조절을 위한 변수
frame_count = 0
process_every_n_frames = 2  # 2프레임마다 한 번씩 처리

while True:
    ret, img = cap.read()
    frame_count += 1
    
    # 특정 프레임마다만 처리
    if frame_count % process_every_n_frames == 0:
        # 이미지 크기 축소
        small_img = cv2.resize(img, (0,0), fx=0.5, fy=0.5)
        gray = cv2.cvtColor(small_img, cv2.COLOR_BGR2GRAY)
        
        faces = detector.detectMultiScale(
            gray,
            scaleFactor = 1.1,  # 성능 최적화
            minNeighbors = 4,   # 성능 최적화
            minSize = (30, 30)  # 최소 얼굴 크기 증가
        )
        
        # 원본 이미지 크기로 좌표 변환
        faces = [(int(x*2), int(y*2), int(w*2), int(h*2)) for (x,y,w,h) in faces]
        
        for (x,y,w,h) in faces:
            cv2.rectangle(img, (x,y), (x+w,y+h), (255,255,255), 2)
            
            # ROI 추출
            roi_gray = gray[int(y/2):int((y+h)/2), int(x/2):int((x+w)/2)]
            
            try:
                # 얼굴 인식
                id, confidence = recognizer.predict(roi_gray)
                
                if confidence < 100:
                    id = names[id] if id < len(names) else "unknown"
                    confidence = " {0}%".format(round(100 - confidence))
                else:
                    id = "unknown"
                    confidence = " {0}%".format(round(100 - confidence))
                
                # 텍스트 표시
                cv2.putText(img, str(id), (x+5,y-5), font, 0.8, (0,255,0), 2)
                cv2.putText(img, str(confidence), (x+5,y+h-5), font, 0.8, (0,255,0), 2)
            
            except Exception as e:
                print(f"Error during recognition: {e}")
    
    cv2.imshow('camera',img)
    k = cv2.waitKey(1) & 0xff
    if k == 27:
        break

print("\nExiting Program.")
cap.release()
cv2.destroyAllWindows()