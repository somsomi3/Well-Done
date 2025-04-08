from data_augmentiation import create_dataset
import tensorflow as tf
import os
import numpy as np
import xml.etree.ElementTree as ET
from xml.dom import minidom

# 증강된 이미지 저장 디렉토리
output_img_dir = 'augmented_images'
output_xml_dir = 'augmented_annotations'
os.makedirs(output_img_dir, exist_ok=True)
os.makedirs(output_xml_dir, exist_ok=True)

# 이미지와 XML 파일이 있는 디렉토리 경로 지정
image_dir = 'model_image'
xml_dir = 'model_data_set'

# 데이터셋 생성
dataset = create_dataset(image_dir, xml_dir, batch_size=8)

# XML 파일 생성 함수
def create_xml_annotation(filename, img_width, img_height, boxes, labels, folder="augmented"):
    root = ET.Element("annotation")
    
    # 기본 정보 추가
    ET.SubElement(root, "folder").text = folder
    ET.SubElement(root, "filename").text = filename
    ET.SubElement(root, "path").text = f"/{folder}/{filename}"
    
    source = ET.SubElement(root, "source")
    ET.SubElement(source, "database").text = "Unspecified"
    
    size = ET.SubElement(root, "size")
    ET.SubElement(size, "width").text = str(img_width)
    ET.SubElement(size, "height").text = str(img_height)
    ET.SubElement(size, "depth").text = "3"
    
    # 객체 정보 추가
    for i, (box, label) in enumerate(zip(boxes, labels)):
        obj = ET.SubElement(root, "object")
        ET.SubElement(obj, "name").text = label
        ET.SubElement(obj, "pose").text = "Unspecified"
        ET.SubElement(obj, "truncated").text = "0"
        ET.SubElement(obj, "difficult").text = "0"
        
        bbox = ET.SubElement(obj, "bndbox")
        ET.SubElement(bbox, "xmin").text = str(int(box[1]))  # xmin
        ET.SubElement(bbox, "ymin").text = str(int(box[0]))  # ymin
        ET.SubElement(bbox, "xmax").text = str(int(box[3]))  # xmax
        ET.SubElement(bbox, "ymax").text = str(int(box[2]))  # ymax
    
    # XML 문자열로 변환하고 들여쓰기 추가
    xml_str = minidom.parseString(ET.tostring(root)).toprettyxml(indent="  ")
    return xml_str

# 증강된 데이터를 파일로 저장
for i, (images, boxes, labels) in enumerate(dataset):
    for j in range(images.shape[0]):
        # 이미지 저장
        img = images[j].numpy()
        img = (img * 255).astype(np.uint8)  # [0,1] -> [0,255]
        img_filename = f"aug_{i}_{j}.png"
        img_path = os.path.join(output_img_dir, img_filename)
        tf.io.write_file(
            img_path,
            tf.io.encode_png(tf.convert_to_tensor(img))
        )
        
        # XML 형식으로 바운딩 박스 정보 저장
        box = boxes[j].numpy()
        label = [l.decode('utf-8') for l in labels[j].numpy()]
        
        xml_content = create_xml_annotation(
            img_filename, 
            img.shape[1],  # width
            img.shape[0],  # height
            box, 
            label
        )
        
        xml_filename = f"aug_{i}_{j}.xml"
        xml_path = os.path.join(output_xml_dir, xml_filename)
        with open(xml_path, 'w') as f:
            f.write(xml_content)
    
    print(f"배치 {i}의 증강 데이터가 저장되었습니다.")

# 데이터셋 순회 및 증강된 이미지 확인
for images, boxes, labels in dataset.take(1):
    print("Augmented images shape:", images.shape)
    print("Augmented boxes shape:", boxes.shape)
    print("Augmented labels:", labels)
