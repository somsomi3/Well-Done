import cv2
import numpy as np
import os
import mediapipe as mp

# OpenCV 얼굴 인식 설정
recognizer = cv2.face.LBPHFaceRecognizer_create()
recognizer.read('trainer/trainer.yml')
detector = cv2.CascadeClassifier("./haarcascade_frontalface_default.xml")
font = cv2.FONT_HERSHEY_SIMPLEX

# MediaPipe 손 인식 설정
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=2,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)
mp_draw = mp.solutions.drawing_utils

# 파일명에서 이름 추출
def get_names_from_files():
    names = ['unknown'] * 200
    userdata_path = 'userdata'
    if os.path.exists(userdata_path):
        for filename in os.listdir(userdata_path):
            if filename.startswith('User_'):
                parts = filename.split('_')
                if len(parts) >= 3:
                    user_id = parts[1]
                    user_name = parts[2]
                    file_number = int(parts[3].split('.')[0])
                    names[file_number] = f"{user_id}_{user_name}"
    return names

# 손의 좌우 방향 감지
def detect_hand_direction(hand_landmarks, image_width):
    # 손바닥 중심점의 x 좌표
    palm_x = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].x * image_width
    # 화면 중앙을 기준으로 방향 판단
    if palm_x < image_width / 2:
        return "Left"
    else:
        return "Right"

names = get_names_from_files()
confidence_threshold = 60

cap = cv2.VideoCapture(0)
cap.set(3, 1280)
cap.set(4, 720)

while True:
    ret, img = cap.read()
    if not ret:
        break
        
    # 손 인식을 위한 이미지 변환
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    hand_results = hands.process(img_rgb)
    
    # 얼굴 인식
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = detector.detectMultiScale(
        gray,
        scaleFactor=1.2,
        minNeighbors=5,
        minSize=(20, 20)
    )
    
    # 얼굴 인식 결과 표시
    for (x,y,w,h) in faces:
        cv2.rectangle(img, (x,y), (x+w,y+h), (255,255,255), 2)
        id, confidence = recognizer.predict(gray[y:y+h,x:x+w])
        
        if confidence < confidence_threshold and id < len(names):
            id = names[id]
        else:
            id = "unknown"
        
        confidence_display = " {0}%".format(round(100 - confidence))
        cv2.putText(img, str(id), (x+5,y-5), font, 1, (0,255,0), 2)
        cv2.putText(img, str(confidence_display), (x+5,y+h-5), font, 1, (0,255,0), 2)
    
    # 손 인식 결과 표시
    if hand_results.multi_hand_landmarks:
        for hand_landmarks in hand_results.multi_hand_landmarks:
            # 손 landmarks 그리기
            mp_draw.draw_landmarks(img, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            
            # 손의 방향 감지 및 표시
            direction = detect_hand_direction(hand_landmarks, img.shape[1])
            
            # 손바닥 중심점 좌표 계산
            palm_center = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
            px = int(palm_center.x * img.shape[1])
            py = int(palm_center.y * img.shape[0])
            
            # 방향 텍스트 표시
            cv2.putText(img, f"Hand: {direction}", (px-50, py-20), 
                       font, 1, (255, 0, 0), 2)
    
    cv2.imshow('Face and Hand Detection', img)
    
    k = cv2.waitKey(1) & 0xff
    if k == 27:
        break

print("\nExiting Program.")
cap.release()
cv2.destroyAllWindows()
hands.close()