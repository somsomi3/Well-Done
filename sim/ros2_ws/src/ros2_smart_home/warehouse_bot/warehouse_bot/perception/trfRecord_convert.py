import os
import io
import pandas as pd
import tensorflow as tf
from collections import namedtuple
from object_detection.utils import dataset_util

def class_text_to_int(row_label):
    if row_label == "palette":
        return 1
    elif row_label == "couquedasse":
        return 2
    elif row_label == "moncher":
        return 3
    else:
        return None  # 오류 방지를 위해 알 수 없는 클래스 처리



def split(df, group):
    data = namedtuple('data', ['filename', 'object'])
    grouped = df.groupby(group)
    return [data(filename, grouped.get_group(x))
            for filename, x in zip(grouped.groups.keys(), grouped.groups)]

def create_tf_example(group, images_path):
    image_path = os.path.join(images_path, group.filename)
    with tf.io.gfile.GFile(image_path, 'rb') as fid:
        encoded_png = fid.read()
    filename = group.filename.encode('utf8')
    image_format = b'png'  # 이미지 형식이 png라고 가정합니다.
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
    
    # 데이터 섞기 및 분할
    examples = pd.read_csv(csv_input)
    examples = examples.sample(frac=1).reset_index(drop=True)  # 랜덤 셔플

    # 80% 학습, 20% 테스트 분할
    split_index = int(0.8 * len(examples))
    train_examples = examples[:split_index]
    test_examples = examples[split_index:]

    # 트레인/테스트 TFRecord 별도 생성
    train_output = "model_data_augmented_set/train.record"
    test_output = "model_data_augmented_set/test.record"
    
    # 트레인 데이터 작성
    writer_train = tf.io.TFRecordWriter(train_output)
    grouped_train = split(train_examples, 'filename')
    for group in grouped_train:
        tf_example = create_tf_example(group, images_path)
        writer_train.write(tf_example.SerializeToString())
    writer_train.close()

    # 테스트 데이터 작성
    writer_test = tf.io.TFRecordWriter(test_output)
    grouped_test = split(test_examples, 'filename')
    for group in grouped_test:
        tf_example = create_tf_example(group, images_path)
        writer_test.write(tf_example.SerializeToString())
    writer_test.close()


    print("Train TFRecord 파일이 '{}' 에 생성되었습니다.".format(train_output))
    print("Test TFRecord 파일이 '{}' 에 생성되었습니다.".format(test_output))

if __name__ == '__main__':
    main()
