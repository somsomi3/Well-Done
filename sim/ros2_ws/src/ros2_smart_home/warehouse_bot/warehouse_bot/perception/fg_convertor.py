import tensorflow as tf

# SavedModel 로드 (Raw String 사용)
model = tf.saved_model.load(r"C:\Users\SSAFY\Desktop\project\S12P21E102\sim\ros2_ws\src\ros2_smart_home\warehouse_bot\warehouse_bot\perception\model_weights\exported_model\saved_model")

# Frozen Graph 변환
from tensorflow.python.framework.convert_to_constants import convert_variables_to_constants_v2
frozen_func = convert_variables_to_constants_v2(
    model.signatures['serving_default'],
    lower_control_flow=False
)

# .pb 파일 저장 (정방향 슬래시 사용)
tf.io.write_graph(
    frozen_func.graph,
    "C:/Users/SSAFY/Desktop/project/S12P21E102/sim/ros2_ws/src/ros2_smart_home/warehouse_bot/warehouse_bot/perception/model_weights/exported_model/frozen_model",
    "frozen_inference_graph.pb",
    as_text=False
)

