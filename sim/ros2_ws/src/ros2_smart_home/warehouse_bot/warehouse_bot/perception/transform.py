#!/usr/bin/env python
import os
import glob
import pandas as pd
import xml.etree.ElementTree as ET

def xml_to_csv(xml_folder):
    """
    xml_folder: XML 파일들이 존재하는 폴더 (여기서는 "augmented_annotations")
    """
    xml_list = []
    # xml 폴더 내의 모든 .xml 파일 읽기
    for xml_file in glob.glob(os.path.join(xml_folder, '*.xml')):
        tree = ET.parse(xml_file)
        root = tree.getroot()

        # XML 파일 내 <filename> 태그의 텍스트를 읽어옵니다.
        filename = root.find('filename').text
        
        # <size> 태그 내부의 <width>와 <height> 값을 읽습니다.
        size = root.find('size')
        width = int(size.find('width').text)
        height = int(size.find('height').text)
        
        # image 내 객체(object)가 여러 개 있을 경우 for문으로 순회
        for member in root.findall('object'):
            object_class = member.find('name').text  # 객체 이름
            bndbox = member.find('bndbox')
            xmin = int(bndbox.find('xmin').text)
            ymin = int(bndbox.find('ymin').text)
            xmax = int(bndbox.find('xmax').text)
            ymax = int(bndbox.find('ymax').text)
            xml_list.append((filename, width, height, object_class, xmin, ymin, xmax, ymax))
    
    column_name = ['filename', 'width', 'height', 'class', 'xmin', 'ymin', 'xmax', 'ymax']
    xml_df = pd.DataFrame(xml_list, columns=column_name)
    return xml_df

if __name__ == "__main__":
    # augmented_annotations 폴더에 있는 XML 파일들을 CSV로 변환
    xml_df = xml_to_csv("augmented_annotations")
    
    # CSV 파일을 저장할 폴더(model_data_set)가 없으면 생성
    output_folder = "model_data_augmented_set"
    os.makedirs(output_folder, exist_ok=True)
    
    csv_output_path = os.path.join(output_folder, "annotations.csv")
    xml_df.to_csv(csv_output_path, index=False)
    print("CSV 파일이 {} 에 저장되었습니다.".format(csv_output_path))
