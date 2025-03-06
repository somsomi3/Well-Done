import numpy as np
import cv2
import os
from PIL import Image  # 대체 저장 방법을 위해 추가

# 현재 작업 디렉토리 출력
current_dir = os.getcwd()
print("현재 작업 디렉토리:", current_dir)

# userdata 폴더 생성 (절대 경로 사용)
save_dir = os.path.join(current_dir, "userdata")
if not os.path.exists(save_dir):
    os.makedirs(save_dir)
print("저장 경로:", save_dir)

#데이터베이스에 올릴  이름과 id를 입력받음
user_name = input("Please write your name : ")
user_id = input("Please write your id : ")
detected_data = cv2.CascadeClassifier('./haarcascade_frontalface_default.xml')
cap = cv2.VideoCapture(0)
cap.set(3,1280)
cap.set(4,720)
count = 0
interrupt_flag = 0

while True:
    ret, img = cap.read()
    if not ret:
        print("Failed to capture image")
        continue
        
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = detected_data.detectMultiScale(
        gray,
        scaleFactor=1.2,
        minNeighbors=9,
        minSize=(20, 20)
    )
    
    for (x,y,w,h) in faces:
        cv2.rectangle(img,(x,y),(x+w,y+h),(255,255,255),2)
        count+=1
        face_img = gray[y:y+h, x:x+w]
        
        # 이미지가 유효한지 확인
        if face_img.size == 0:
            print("Error: Empty image")
            continue
            
        print(f"Face image shape: {face_img.shape}")
        print(f"Face image dtype: {face_img.dtype}")
        print(f"Face image min/max values: {face_img.min()}/{face_img.max()}")
        
        # 방법 1: cv2.imwrite로 시도
        save_path = os.path.join(save_dir, f"User_{user_id}_{user_name}_{count}.jpg")
        success = cv2.imwrite(save_path, face_img)
        
        if not success:
            print("cv2.imwrite failed, trying PIL...")
            # 방법 2: PIL을 사용하여 저장 시도
            try:
                pil_image = Image.fromarray(face_img)
                pil_image.save(save_path)
                print("PIL save successful")
            except Exception as e:
                print(f"PIL save failed: {e}")
                
                # 방법 3: 다른 형식으로 저장 시도
                try:
                    cv2.imwrite(save_path.replace('.jpg', '.png'), face_img)
                    print("Saved as PNG instead")
                except Exception as e:
                    print(f"PNG save failed: {e}")
                    
                    # 방법 4: 임시 디렉토리에 저장 시도
                    try:
                        temp_path = os.path.join(os.environ.get('TEMP', '/tmp'), f"User_{user_id}_{user_name}_{count}.jpg")
                        cv2.imwrite(temp_path, face_img)
                        print(f"Saved to temp directory: {temp_path}")
                    except Exception as e:
                        print(f"Temp save failed: {e}")
        
        # 파일 존재 확인
        if os.path.exists(save_path):
            print(f"File exists at: {save_path}")
            print(f"File size: {os.path.getsize(save_path)} bytes")
        else:
            print("Error: File was not saved!")
        
    cv2.imshow('image', img)
    k = cv2.waitKey(50) & 0xff
    
    if k == 27:
        interrupt_flag = 1
        break
    elif count >= 100:
        break

if interrupt_flag == 1:
    print("\nFinish by interrupt ESC.\n")
else:
    print("\nComplete to save data.\n")

# 저장된 파일 목록 출력
print("\nSaved files in userdata directory:")
if os.path.exists(save_dir):
    files = os.listdir(save_dir)
    for file in files:
        file_path = os.path.join(save_dir, file)
        print(f"- {file} ({os.path.getsize(file_path)} bytes)")
else:
    print("userdata directory does not exist!")

cap.release()
cv2.destroyAllWindows()