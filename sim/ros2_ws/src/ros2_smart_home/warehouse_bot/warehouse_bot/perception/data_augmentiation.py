import tensorflow as tf
import tensorflow_addons as tfa
import xml.etree.ElementTree as ET
import numpy as np
import os
import tensorflow.compat.v1 as tf

global xml_path
xml_path = 'warehouse_bot\perception\model_data_set'
def parse_xml(xml_path):
    """XML 파일에서 바운딩 박스 정보 추출"""
    tree = ET.parse(xml_path)
    root = tree.getroot()
    
    boxes = []
    labels = []
    
    for obj in root.findall('object'):
        label = obj.find('name').text
        bbox = obj.find('bndbox')
        xmin = float(bbox.find('xmin').text)
        ymin = float(bbox.find('ymin').text)
        xmax = float(bbox.find('xmax').text)
        ymax = float(bbox.find('ymax').text)
        
        boxes.append([ymin, xmin, ymax, xmax])
        labels.append(label)
    
    return np.array(boxes, dtype=np.float32), labels

def transform_bbox(bbox, image_shape, transformation_matrix):
    """바운딩 박스 좌표를 변환 행렬에 따라 변환"""
    h, w = image_shape[:2]
    
    # 바운딩 박스의 모서리 좌표 추출 [ymin, xmin, ymax, xmax]
    ymin, xmin, ymax, xmax = bbox
    
    # 모서리 좌표를 동차 좌표로 변환
    corners = tf.constant([
        [xmin, ymin, 1.0],
        [xmax, ymin, 1.0],
        [xmin, ymax, 1.0],
        [xmax, ymax, 1.0]
    ])
    
    # 변환 행렬 적용
    new_corners = tf.matmul(corners, tf.transpose(transformation_matrix))
    
    # 새 모서리 좌표에서 바운딩 박스 계산
    new_xmin = tf.reduce_min(new_corners[:, 0])
    new_ymin = tf.reduce_min(new_corners[:, 1])
    new_xmax = tf.reduce_max(new_corners[:, 0])
    new_ymax = tf.reduce_max(new_corners[:, 1])
    
    # 이미지 경계 내로 클리핑
    new_xmin = tf.clip_by_value(new_xmin, 0, w)
    new_ymin = tf.clip_by_value(new_ymin, 0, h)
    new_xmax = tf.clip_by_value(new_xmax, 0, w)
    new_ymax = tf.clip_by_value(new_ymax, 0, h)
    
    return tf.stack([new_ymin, new_xmin, new_ymax, new_xmax])

def augment_with_labels(image, boxes, labels):
    """이미지와 바운딩 박스를 함께 증강"""
    # 이미지 크기 저장
    image_shape = tf.shape(image)
    h, w = image_shape[0], image_shape[1]
    
    # 1. 랜덤 밝기 조정 (바운딩 박스에 영향 없음)
    if tf.random.uniform(()) > 0.5:
        image = tf.image.random_brightness(image, max_delta=0.2)
    
    # 2. 랜덤 대비 조정 (바운딩 박스에 영향 없음)
    if tf.random.uniform(()) > 0.5:
        image = tf.image.random_contrast(image, lower=0.8, upper=1.2)
    
    # 3. 랜덤 좌우 반전 (바운딩 박스도 함께 반전)
    if tf.random.uniform(()) > 0.5:
        image = tf.image.flip_left_right(image)
        # 바운딩 박스 좌우 반전
        flipped_boxes = tf.stack([
            boxes[:, 0],  # ymin 유지
            w - boxes[:, 3],  # xmin = w - xmax
            boxes[:, 2],  # ymax 유지
            w - boxes[:, 1]   # xmax = w - xmin
        ], axis=1)
        boxes = flipped_boxes
    
    # 4. 랜덤 회전 (바운딩 박스도 함께 회전)
    if tf.random.uniform(()) > 0.7:  # 회전은 덜 자주 적용
        angle = tf.random.uniform([], minval=-15, maxval=15, dtype=tf.float32)
        # 회전 변환 행렬 계산
        radian = angle * np.pi / 180
        rotation_matrix = tfa.image.transform_ops.angles_to_projective_transforms(
            radian, tf.cast(h, tf.float32), tf.cast(w, tf.float32)
        )
        
        # 이미지 회전
        image = tfa.image.transform(image, rotation_matrix, interpolation='BILINEAR')
        
        # 각 바운딩 박스 회전
        new_boxes = []
        for box in boxes:
            new_box = transform_bbox(box, (h, w), rotation_matrix)
            new_boxes.append(new_box)
        
        boxes = tf.stack(new_boxes)
    
    return image, boxes, labels

# 데이터셋 생성 예시
def create_dataset(image_dir, xml_dir, batch_size=8):
    image_paths = [os.path.join(image_dir, f) for f in os.listdir(image_dir) if f.endswith(('.jpg', '.png'))]
    xml_paths = [os.path.join(xml_dir, os.path.splitext(os.path.basename(img))[0] + '.xml') for img in image_paths]
    
    # 이미지와 XML 파일 로드하는 함수
    def load_data(img_path, xml_path):
        img = tf.io.read_file(img_path)
        img = tf.image.decode_image(img, channels=3)
        img = tf.image.convert_image_dtype(img, tf.float32)
        
        # XML에서 바운딩 박스 정보 추출
        boxes, labels = tf.py_function(
            parse_xml, [xml_path], [tf.float32, tf.string]
        )
        
        return img, boxes, labels
    
    # 데이터셋 생성
    dataset = tf.data.Dataset.from_tensor_slices((image_paths, xml_paths))
    dataset = dataset.map(load_data, num_parallel_calls=tf.data.AUTOTUNE)
    
    # 증강 적용
    dataset = dataset.map(augment_with_labels, num_parallel_calls=tf.data.AUTOTUNE)
    
    # 배치 처리 및 최적화
    dataset = dataset.batch(batch_size).prefetch(tf.data.AUTOTUNE)
    
    return dataset
