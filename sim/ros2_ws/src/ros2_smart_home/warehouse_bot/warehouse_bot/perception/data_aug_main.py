from data_augmentiation import parse_xml
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
target_size = (300, 300)  # 목표 이미지 크기

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

# 바운딩 박스 조정 함수
def resize_boxes(boxes, original_size, target_size):
    height_ratio = target_size[0] / original_size[0]
    width_ratio = target_size[1] / original_size[1]
    
    resized_boxes = np.zeros_like(boxes)
    resized_boxes[:, 0] = boxes[:, 0] * height_ratio  # ymin
    resized_boxes[:, 1] = boxes[:, 1] * width_ratio   # xmin
    resized_boxes[:, 2] = boxes[:, 2] * height_ratio  # ymax
    resized_boxes[:, 3] = boxes[:, 3] * width_ratio   # xmax
    
    return resized_boxes

# 좌우 반전 시 바운딩 박스 조정
def flip_boxes(boxes, width):
    flipped_boxes = np.zeros_like(boxes)
    flipped_boxes[:, 0] = boxes[:, 0]                # ymin
    flipped_boxes[:, 1] = width - boxes[:, 3]        # xmin = width - xmax
    flipped_boxes[:, 2] = boxes[:, 2]                # ymax
    flipped_boxes[:, 3] = width - boxes[:, 1]        # xmax = width - xmin
    return flipped_boxes

# 90도 회전 시 바운딩 박스 조정
def rotate90_boxes(boxes, width, height):
    rotated_boxes = np.zeros_like(boxes)
    rotated_boxes[:, 0] = boxes[:, 1]                # ymin = xmin
    rotated_boxes[:, 1] = height - boxes[:, 2]       # xmin = height - ymax
    rotated_boxes[:, 2] = boxes[:, 3]                # ymax = xmax
    rotated_boxes[:, 3] = height - boxes[:, 0]       # xmax = height - ymin
    return rotated_boxes

# 유효한 이미지-XML 쌍 찾기
image_paths = []
xml_paths = []
for f in os.listdir(image_dir):
    if f.endswith(('.jpg', '.png')):
        img_path = os.path.join(image_dir, f)
        xml_name = os.path.splitext(f)[0] + '.xml'
        xml_path = os.path.join(xml_dir, xml_name)
        
        # 두 파일이 모두 존재하는 경우만 추가
        if os.path.exists(xml_path):
            image_paths.append(img_path)
            xml_paths.append(xml_path)

if not image_paths:
    raise ValueError("유효한 이미지-XML 쌍을 찾을 수 없습니다.")

print(f"처리할 이미지 수: {len(image_paths)}")

# 각 이미지에 대해 증강 수행
for idx, (img_path, xml_path) in enumerate(zip(image_paths, xml_paths)):
    print(f"이미지 {idx+1}/{len(image_paths)} 처리 중: {os.path.basename(img_path)}")
    
    # 이미지 로드 및 전처리
    img = tf.io.read_file(img_path)
    img = tf.image.decode_png(img, channels=3)
    
    # 원본 크기 저장
    original_height = img.shape[0]
    original_width = img.shape[1]
    original_size = (original_height, original_width)
    
    # XML에서 바운딩 박스 파싱
    boxes, labels = parse_xml(xml_path)
    
    # 1. 원본 이미지 (리사이징만 적용)
    img_resized = tf.image.resize(img, target_size)
    img_resized = tf.cast(img_resized * 255, tf.uint8)
    
    # 바운딩 박스 조정
    boxes_resized = resize_boxes(boxes, original_size, target_size)
    
    # 저장
    original_filename = f"orig_{idx}.png"
    tf.io.write_file(
        os.path.join(output_img_dir, original_filename),
        tf.io.encode_png(img_resized)
    )
    
    # XML 저장
    xml_content = create_xml_annotation(
        original_filename,
        target_size[1],  # width
        target_size[0],  # height
        boxes_resized,
        labels
    )
    with open(os.path.join(output_xml_dir, f"orig_{idx}.xml"), 'w') as f:
        f.write(xml_content)
    
    # 2. 밝기 증강
    img_brightness = tf.image.adjust_brightness(img_resized, delta=0.1)
    img_brightness = tf.cast(tf.clip_by_value(img_brightness, 0, 255), tf.uint8)
    
    brightness_filename = f"bright_{idx}.png"
    tf.io.write_file(
        os.path.join(output_img_dir, brightness_filename),
        tf.io.encode_png(img_brightness)
    )
    
    # XML 저장 (바운딩 박스는 변하지 않음)
    xml_content = create_xml_annotation(
        brightness_filename,
        target_size[1],
        target_size[0],
        boxes_resized,
        labels
    )
    with open(os.path.join(output_xml_dir, f"bright_{idx}.xml"), 'w') as f:
        f.write(xml_content)
    
    # 3. 대비 증강
    img_contrast = tf.image.adjust_contrast(img_resized, 1.2)
    img_contrast = tf.cast(tf.clip_by_value(img_contrast, 0, 255), tf.uint8)
    
    contrast_filename = f"contrast_{idx}.png"
    tf.io.write_file(
        os.path.join(output_img_dir, contrast_filename),
        tf.io.encode_png(img_contrast)
    )
    
    # XML 저장 (바운딩 박스는 변하지 않음)
    xml_content = create_xml_annotation(
        contrast_filename,
        target_size[1],
        target_size[0],
        boxes_resized,
        labels
    )
    with open(os.path.join(output_xml_dir, f"contrast_{idx}.xml"), 'w') as f:
        f.write(xml_content)
    
    # 4. 좌우 반전
    img_flipped = tf.image.flip_left_right(img_resized)
    
    flipped_filename = f"flip_{idx}.png"
    tf.io.write_file(
        os.path.join(output_img_dir, flipped_filename),
        tf.io.encode_png(img_flipped)
    )
    
    # 바운딩 박스 좌우 반전
    boxes_flipped = flip_boxes(boxes_resized, target_size[1])
    
    # XML 저장
    xml_content = create_xml_annotation(
        flipped_filename,
        target_size[1],
        target_size[0],
        boxes_flipped,
        labels
    )
    with open(os.path.join(output_xml_dir, f"flip_{idx}.xml"), 'w') as f:
        f.write(xml_content)
    
    # 5. 90도 회전
    img_rotated = tf.image.rot90(img_resized)
    
    rotated_filename = f"rot90_{idx}.png"
    tf.io.write_file(
        os.path.join(output_img_dir, rotated_filename),
        tf.io.encode_png(img_rotated)
    )
    
    # 바운딩 박스 회전
    boxes_rotated = rotate90_boxes(boxes_resized, target_size[1], target_size[0])
    
    # XML 저장 (주의: 회전 후에는 이미지 크기가 바뀔 수 있음)
    xml_content = create_xml_annotation(
        rotated_filename,
        target_size[0],  # 회전 후 width = 원래 height
        target_size[1],  # 회전 후 height = 원래 width
        boxes_rotated,
        labels
    )
    with open(os.path.join(output_xml_dir, f"rot90_{idx}.xml"), 'w') as f:
        f.write(xml_content)

print(f"모든 이미지 증강 완료. 총 {len(image_paths) * 5}개 이미지 생성됨")
print(f"- 원본 리사이징: {len(image_paths)}개")
print(f"- 밝기 증강: {len(image_paths)}개")
print(f"- 대비 증강: {len(image_paths)}개")
print(f"- 좌우 반전: {len(image_paths)}개")
print(f"- 90도 회전: {len(image_paths)}개")
