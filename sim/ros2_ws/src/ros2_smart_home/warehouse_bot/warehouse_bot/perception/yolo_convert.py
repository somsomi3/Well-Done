# yolo_convert.py
import os
import shutil
import random
import xml.etree.ElementTree as ET
import yaml

def convert_xml_to_yolo(xml_dir, images_dir, output_dir):
    """XML 디렉토리를 YOLO 형식으로 변환"""
    os.makedirs(output_dir, exist_ok=True)
    
    # 모든 클래스 이름 수집
    all_classes = set()
    for xml_file in os.listdir(xml_dir):
        if not xml_file.endswith('.xml'):
            continue
        
        tree = ET.parse(os.path.join(xml_dir, xml_file))
        root = tree.getroot()
        
        for obj in root.findall('object'):
            class_name = obj.find('name').text
            all_classes.add(class_name)
    
    # 클래스 매핑 생성
    class_map = {name: idx for idx, name in enumerate(sorted(all_classes))}
    print(f"발견된 클래스: {class_map}")
    
    # 이미지별 어노테이션 수집
    image_annotations = {}
    for xml_file in os.listdir(xml_dir):
        if not xml_file.endswith('.xml'):
            continue
        
        tree = ET.parse(os.path.join(xml_dir, xml_file))
        root = tree.getroot()
        
        image_name = root.find('filename').text
        size = root.find('size')
        width = int(size.find('width').text)
        height = int(size.find('height').text)
        
        if image_name not in image_annotations:
            image_annotations[image_name] = []
        
        for obj in root.findall('object'):
            class_name = obj.find('name').text
            class_id = class_map[class_name]
            
            bndbox = obj.find('bndbox')
            xmin = float(bndbox.find('xmin').text)
            ymin = float(bndbox.find('ymin').text)
            xmax = float(bndbox.find('xmax').text)
            ymax = float(bndbox.find('ymax').text)
            
            # 바운딩 박스 좌표 정규화
            x_center = (xmin + xmax) / 2 / width
            y_center = (ymin + ymax) / 2 / height
            box_width = (xmax - xmin) / width
            box_height = (ymax - ymin) / height
            
            # YOLO 형식으로 추가
            image_annotations[image_name].append(f"{class_id} {x_center:.6f} {y_center:.6f} {box_width:.6f} {box_height:.6f}")
    
    # 학습/검증 데이터 분할 (80/20)
    train_dir = os.path.join(output_dir, 'train')
    val_dir = os.path.join(output_dir, 'val')
    train_img_dir = os.path.join(train_dir, 'images')
    train_lbl_dir = os.path.join(train_dir, 'labels')
    val_img_dir = os.path.join(val_dir, 'images')
    val_lbl_dir = os.path.join(val_dir, 'labels')
    
    for dir_path in [train_img_dir, train_lbl_dir, val_img_dir, val_lbl_dir]:
        os.makedirs(dir_path, exist_ok=True)
    
    # 이미지 및 라벨 파일 생성
    all_images = list(image_annotations.keys())
    random.shuffle(all_images)
    split_idx = int(len(all_images) * 0.8)
    
    train_images = all_images[:split_idx]
    val_images = all_images[split_idx:]
    
    print(f"총 이미지 수: {len(all_images)}")
    print(f"학습 이미지 수: {len(train_images)}")
    print(f"검증 이미지 수: {len(val_images)}")
    
    # 훈련 데이터 복사 및 라벨 생성
    for img_name in train_images:
        # 이미지 복사
        src_img = os.path.join(images_dir, img_name)
        dst_img = os.path.join(train_img_dir, img_name)
        shutil.copy2(src_img, dst_img)
        
        # 라벨 파일 생성
        txt_name = os.path.splitext(img_name)[0] + '.txt'
        with open(os.path.join(train_lbl_dir, txt_name), 'w') as f:
            f.write('\n'.join(image_annotations[img_name]))
    
    # 검증 데이터 복사 및 라벨 생성
    for img_name in val_images:
        # 이미지 복사
        src_img = os.path.join(images_dir, img_name)
        dst_img = os.path.join(val_img_dir, img_name)
        shutil.copy2(src_img, dst_img)
        
        # 라벨 파일 생성
        txt_name = os.path.splitext(img_name)[0] + '.txt'
        with open(os.path.join(val_lbl_dir, txt_name), 'w') as f:
            f.write('\n'.join(image_annotations[img_name]))
    
    return train_dir, val_dir, list(class_map.keys()), class_map

def create_yaml_file(output_dir, class_names, class_map, yaml_path):
    """YOLO 학습을 위한 데이터 구성 YAML 파일 생성"""
    # 절대 경로를 상대 경로로 변환
    train_path = os.path.join(output_dir, 'train/images')
    val_path = os.path.join(output_dir, 'val/images')
    
    yaml_content = {
        'train': train_path,
        'val': val_path,
        'nc': len(class_names),
        'names': class_names
    }
    
    with open(yaml_path, 'w') as f:
        yaml.dump(yaml_content, f, default_flow_style=False)
    
    print(f"YAML 파일이 생성되었습니다: {yaml_path}")
    print(f"클래스 매핑: {class_map}")

# 메인 실행 코드
if __name__ == "__main__":
    # 파일 경로 설정
    xml_dir = 'augmented_annotations'
    images_dir = 'augmented'
    output_dir = 'yolov8_dataset'
    yaml_path = 'exported_model/data.yaml'
    
    # XML에서 YOLO 형식으로 변환
    train_dir, val_dir, class_names, class_map = convert_xml_to_yolo(
        xml_dir=xml_dir,
        images_dir=images_dir,
        output_dir=output_dir
    )
    
    # YAML 파일 생성
    create_yaml_file(output_dir, class_names, class_map, yaml_path)
    
    print("\n변환 완료!")
    print(f"1. YOLO 데이터셋 경로: {output_dir}")
    print(f"2. 학습 이미지 경로: {os.path.join(output_dir, 'train/images')}")
    print(f"3. 검증 이미지 경로: {os.path.join(output_dir, 'val/images')}")
    print(f"4. YAML 파일 경로: {yaml_path}")
    print("\nYOLOv8 학습 명령어 예시:")
    print("from ultralytics import YOLO")
    print("model = YOLO('yolov8n.pt')")
    print(f"model.train(data='{yaml_path}', epochs=100, imgsz=640)")
