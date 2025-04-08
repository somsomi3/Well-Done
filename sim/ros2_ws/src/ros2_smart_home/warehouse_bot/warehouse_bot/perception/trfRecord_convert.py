#!/usr/bin/env python
import os
import io
import pandas as pd
import tensorflow as tf
from collections import namedtuple
from object_detection.utils import dataset_util

# CSV에 기록된 클래스 이름을 정수 ID로 매핑하는 함수입니다.
def class_text_to_int(row_label):
    
    if row_label == "Rack":
        return 1
    else:
        return None

# CSV 파일을 이미지 파일별로 그룹화합니다.
def split(df, group):
    data = namedtuple('data', ['filename', 'object'])
    grouped = df.groupby(group)
    return [data(filename, grouped.get_group(x))
            for filename, x in zip(grouped.groups.keys(), grouped.groups)]

# 하나의 이미지에 대한 tf.train.Example을 생성합니다.
def create_tf_example(group, images_path):
    image_path = os.path.join(images_path, group.filename)
    with tf.io.gfile.GFile(image_path, 'rb') as fid:
        encoded_png = fid.read()
    filename = group.filename.encode('utf8')
    image_format = b'png'  # 이미지 형식이 png라고 가정합니다.

    # CSV에서 image 크기는 각 행마다 중복되어 있으므로 첫 번째 행의 값을 사용합니다.
    width = int(group.object.iloc[0]['width'])
    height = int(group.object.iloc[0]['height'])

    xmins = []  # bounding box의 왼쪽 좌표 (정규화된 값)
    xmaxs = []  # bounding box의 오른쪽 좌표 (정규화)
    ymins = []  # bounding box의 위쪽 좌표 (정규화)
    ymaxs = []  # bounding box의 아래쪽 좌표 (정규화)
    classes_text = []
    classes = []

    for index, row in group.object.iterrows():
        xmins.append(row['xmin'] / width)
        xmaxs.append(row['xmax'] / width)
        ymins.append(row['ymin'] / height)
        ymaxs.append(row['ymax'] / height)
        classes_text.append(row['class'].encode('utf8'))
        classes.append(class_text_to_int(row['class']))

    tf_example = tf.train.Example(features=tf.train.Features(feature={
        'image/height': dataset_util.int64_feature(height),
        'image/width': dataset_util.int64_feature(width),
        'image/filename': dataset_util.bytes_feature(filename),
        'image/source_id': dataset_util.bytes_feature(filename),
        'image/encoded': dataset_util.bytes_feature(encoded_png),
        'image/format': dataset_util.bytes_feature(image_format),
        'image/object/bbox/xmin': dataset_util.float_list_feature(xmins),
        'image/object/bbox/xmax': dataset_util.float_list_feature(xmaxs),
        'image/object/bbox/ymin': dataset_util.float_list_feature(ymins),
        'image/object/bbox/ymax': dataset_util.float_list_feature(ymaxs),
        'image/object/class/text': dataset_util.bytes_list_feature(classes_text),
        'image/object/class/label': dataset_util.int64_list_feature(classes),
    }))
    return tf_example

def main():
    # CSV 파일 경로, 이미지가 저장된 폴더, 그리고 출력할 TFRecord 파일 경로를 지정합니다.
    csv_input = "model_data_augmented_set/annotations.csv"
    images_path = "augmented"
    output_path = "model_data_augmented_set/train.record"

    writer = tf.io.TFRecordWriter(output_path)
    examples = pd.read_csv(csv_input)
    grouped = split(examples, 'filename')
    for group in grouped:
        tf_example = create_tf_example(group, images_path)
        writer.write(tf_example.SerializeToString())
    writer.close()
    print("TFRecord 파일이 '{}' 에 생성되었습니다.".format(output_path))

if __name__ == '__main__':
    main()
