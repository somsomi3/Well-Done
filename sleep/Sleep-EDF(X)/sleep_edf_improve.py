# 데이터 증강
from tensorflow.keras.preprocessing.sequence import TimeseriesGenerator

def augment_data(data, labels):
    """데이터 증강"""
    augmented_data = []
    augmented_labels = []
    
    for i in range(len(data)):
        # 원본 데이터
        augmented_data.append(data[i])
        augmented_labels.append(labels[i])
        
        # 노이즈 추가
        noise = np.random.normal(0, 0.1, data[i].shape)
        augmented_data.append(data[i] + noise)
        augmented_labels.append(labels[i])
        
        # 시간 이동
        shift = np.random.randint(-10, 10)
        augmented_data.append(np.roll(data[i], shift))
        augmented_labels.append(labels[i])
    
    return np.array(augmented_data), np.array(augmented_labels)

# 교차 검증
from sklearn.model_selection import KFold

def cross_validate(X, y, n_splits=5):
    """교차 검증"""
    kf = KFold(n_splits=n_splits, shuffle=True)
    scores = []
    
    for fold, (train_idx, val_idx) in enumerate(kf.split(X)):
        X_train, X_val = X[train_idx], X[val_idx]
        y_train, y_val = y[train_idx], y[val_idx]
        
        # 모델 학습
        model = SleepStageClassifier()
        history = model.train(X_train, y_train, (X_val, y_val))
        
        # 성능 평가
        score = model.model.evaluate(X_val, y_val)
        scores.append(score[1])  # accuracy
        
    return np.mean(scores), np.std(scores)