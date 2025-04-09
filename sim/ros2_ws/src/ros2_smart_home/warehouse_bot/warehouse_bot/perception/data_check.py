import cv2
import matplotlib.pyplot as plt
from xml.etree.ElementTree import parse

def parse_xml(xml_path):
    tree = parse(xml_path)
    root = tree.getroot()
    boxes = []
    for obj in root.findall('object'):
        bbox = obj.find('bndbox')
        xmin = int(bbox.find('xmin').text)
        ymin = int(bbox.find('ymin').text)
        xmax = int(bbox.find('xmax').text)
        ymax = int(bbox.find('ymax').text)
        boxes.append((xmin, ymin, xmax, ymax))
    return boxes

def visualize_augmented_data(image_path, xml_path):
    img = cv2.imread(image_path)
    boxes = parse_xml(xml_path)
    
    for (xmin, ymin, xmax, ymax) in boxes:
        cv2.rectangle(img, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
    
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    plt.show()

# 증강된 데이터 샘플 확인
image_sample = "augmented/aug_36_3.png"
xml_sample = "augmented_annotations/aug_36_3.xml"
visualize_augmented_data(image_sample, xml_sample)
