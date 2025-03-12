import cv2
import numpy as np
from PIL import Image
import os

path = 'userdata'
detector = cv2.CascadeClassifier("./haarcascade_frontalface_default.xml")
recognizer = cv2.face.LBPHFaceRecognizer_create()

def getImagesAndLabels(path):
    imagePaths = [os.path.join(path,file) for file in os.listdir(path)]
    faceSamples = []
    ids = []
    
    for imagePath in imagePaths:
        PIL_img = Image.open(imagePath).convert('L')
        img_numpy = np.array(PIL_img,'uint8')
        
        # 모든 이미지에 대해 ID를 1로 설정
        current_id = 1
        
        faces = detector.detectMultiScale(
            img_numpy,
            scaleFactor=1.2,
            minNeighbors=5,
            minSize=(20, 20)
        )
        
        for (x,y,w,h) in faces:
            faceSamples.append(img_numpy[y:y+h,x:x+w])
            ids.append(current_id)
            
            # 디버깅을 위한 출력
            print(f"Processing image: {imagePath}")
            print(f"Found face at: x={x}, y={y}, w={w}, h={h}")
    
    return faceSamples, ids

print("\nStarting training...")
faces,ids = getImagesAndLabels(path)

if len(faces) == 0:
    print("No faces found in the training data!")
else:
    print(f"Found {len(faces)} faces for training")
    recognizer.train(faces, np.array(ids))
    
    if not os.path.exists('trainer'):
        os.makedirs('trainer')
    
    recognizer.write('trainer/trainer.yml')
    print(f"\nTraining completed. {len(np.unique(ids))} faces trained.")    