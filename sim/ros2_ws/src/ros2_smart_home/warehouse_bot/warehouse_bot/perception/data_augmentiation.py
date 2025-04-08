import tensorflow as tf
# import tensorflow_addons as tfa
import xml.etree.ElementTree as ET
import numpy as np
import os
import tensorflow.compat.v1 as tf1

global xml_path
xml_path = 'model_data_set'
def parse_xml(xml_path):
    """XML 파일에서 바운딩 박스 정보 추출"""
    # 텐서를 문자열로 변환
    if isinstance(xml_path, tf.Tensor):
        xml_path = xml_path.numpy().decode('utf-8', errors='ignore')
    
    # 빈 바운딩 박스와 라벨 반환 (오류 발생 시)
    empty_boxes = np.zeros((0, 4), dtype=np.float32)
    empty_labels = []
    
    try:
        # 파일 존재 확인
        if not os.path.exists(xml_path):
            print(f"파일이 존재하지 않음: {xml_path}")
            return empty_boxes, empty_labels
            
        # 바이너리 모드로 파일 열기
        with open(xml_path, 'rb') as f:
            content = f.read()
            
        # XML 파싱
        tree = ET.fromstring(content)
        
        boxes = []
        labels = []
        
        for obj in tree.findall('object'):
            label = obj.find('name').text
            bbox = obj.find('bndbox')
            xmin = float(bbox.find('xmin').text)
            ymin = float(bbox.find('ymin').text)
            xmax = float(bbox.find('xmax').text)
            ymax = float(bbox.find('ymax').text)
            
            boxes.append([ymin, xmin, ymax, xmax])
            labels.append(label)
        
        return np.array(boxes, dtype=np.float32), labels
    except Exception as e:
        print(f"XML 파싱 오류: {e}, 경로: {xml_path}")
        return empty_boxes, empty_labels

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
    h_float = tf.cast(h, tf.float32)
    w_float = tf.cast(w, tf.float32)
    
    # 1. 랜덤 밝기 조정 (바운딩 박스에 영향 없음)
    image = tf.cond(
        tf.random.uniform(()) > 0.5,
        lambda: tf.image.random_brightness(image, max_delta=0.2),
        lambda: image
    )
    
    # 2. 랜덤 대비 조정 (바운딩 박스에 영향 없음)
    image = tf.cond(
        tf.random.uniform(()) > 0.5,
        lambda: tf.image.random_contrast(image, lower=0.8, upper=1.2),
        lambda: image
    )
    
    # 3. 랜덤 좌우 반전 (바운딩 박스도 함께 반전)
    def flip_image_and_boxes():
        flipped_image = tf.image.flip_left_right(image)
        # 바운딩 박스 좌우 반전
        flipped_boxes = tf.stack([
            boxes[:, 0],  # ymin 유지
            w_float - boxes[:, 3],  # xmin = w - xmax
            boxes[:, 2],  # ymax 유지
            w_float - boxes[:, 1]   # xmax = w - xmin
        ], axis=1)
        return flipped_image, flipped_boxes
    
    image, boxes = tf.cond(
        tf.random.uniform(()) > 0.5,
        flip_image_and_boxes,
        lambda: (image, boxes)
    )
    
    # 4. 회전은 제거하거나 단순화
    # 복잡한 반복문 대신 단순 90도 회전만 적용
    def rotate_image_and_boxes():
        rotated_image = tf.image.rot90(image)
        # 90도 회전 시 바운딩 박스 변환
        rotated_boxes = tf.stack([
            w_float - boxes[:, 3],  # ymin = w - xmax
            boxes[:, 0],            # xmin = ymin
            w_float - boxes[:, 1],  # ymax = w - xmin
            boxes[:, 2]             # xmax = ymax
        ], axis=1)
        return rotated_image, rotated_boxes
    
    image, boxes = tf.cond(
        tf.random.uniform(()) > 0.7,
        rotate_image_and_boxes,
        lambda: (image, boxes)
    )
    
    return image, boxes, labels

# 이미지와 XML 파일 로드하는 함수
def load_data(img_path, xml_path, target_size=(300, 300)):
    img = tf.io.read_file(img_path)
    # decode_image 대신 decode_png 사용
    img = tf.image.decode_png(img, channels=3)
    img = tf.image.convert_image_dtype(img, tf.float32)
    
    # 원본 이미지 크기 저장
    original_height = tf.shape(img)[0]
    original_width = tf.shape(img)[1]
    
    # XML에서 바운딩 박스 파싱
    boxes, labels = tf.py_function(
        parse_xml, [xml_path], [tf.float32, tf.string]
    )
    
    # 형태 정보 설정
    boxes.set_shape([None, 4])
    labels.set_shape([None])
    
    # 이미지 리사이징과 바운딩 박스 조정
    img_resized = tf.image.resize(img, target_size)
    
    # 리사이징 비율 계산
    height_ratio = tf.cast(target_size[0], tf.float32) / tf.cast(original_height, tf.float32)
    width_ratio = tf.cast(target_size[1], tf.float32) / tf.cast(original_width, tf.float32)
    
    # 바운딩 박스 좌표 조정 [ymin, xmin, ymax, xmax]
    boxes_resized = tf.stack([
        boxes[:, 0] * height_ratio,  # ymin
        boxes[:, 1] * width_ratio,   # xmin
        boxes[:, 2] * height_ratio,  # ymax
        boxes[:, 3] * width_ratio    # xmax
    ], axis=1)
    
    return img_resized, boxes_resized, labels



# 데이터셋 생성 예시
def create_dataset(image_dir, xml_dir, batch_size=8):
    # 파일 경로 확인
    if not os.path.exists(image_dir):
        raise ValueError(f"이미지 디렉토리가 존재하지 않음: {image_dir}")
    if not os.path.exists(xml_dir):
        raise ValueError(f"XML 디렉토리가 존재하지 않음: {xml_dir}")
    
    # 유효한 이미지 파일만 선택
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
    
    # 나머지 코드는 동일
    dataset = tf.data.Dataset.from_tensor_slices((image_paths, xml_paths))
    dataset = dataset.map(load_data, num_parallel_calls=tf.data.AUTOTUNE)
    dataset = dataset.map(augment_with_labels, num_parallel_calls=tf.data.AUTOTUNE)
    dataset = dataset.batch(batch_size).prefetch(tf.data.AUTOTUNE)
    
    return dataset

