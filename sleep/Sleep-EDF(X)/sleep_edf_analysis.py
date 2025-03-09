import numpy as np
import pandas as pd
import mne
import glob
from scipy import signal
from sklearn.model_selection import train_test_split
from tensorflow.keras.utils import to_categorical

class SleepEDFProcessor:
    def __init__(self, data_path):
        self.data_path = data_path
        self.sampling_rate = 100  # Sleep-EDF의 기본 샘플링 레이트
        self.epoch_length = 30  # 30초 에폭
        
    def load_edf_files(self):
        """EDF 파일 로드"""
        edf_files = glob.glob(f"{self.data_path}/*.edf")
        return edf_files
    
    def preprocess_single_file(self, edf_file):
        """단일 EDF 파일 전처리"""
        # EDF 파일 로드
        raw = mne.io.read_raw_edf(edf_file, preload=True)
        
        # 관심 있는 채널 선택 (EEG, EOG 등)
        channels_of_interest = ['EEG Fpz-Cz', 'EEG Pz-Oz', 'EOG horizontal']
        raw.pick_channels(channels_of_interest)
        
        # 데이터 필터링
        raw.filter(l_freq=0.3, h_freq=45)
        
        # 에폭으로 분할
        data = raw.get_data()
        epochs = []
        
        for i in range(0, data.shape[1], self.sampling_rate * self.epoch_length):
            epoch = data[:, i:i + self.sampling_rate * self.epoch_length]
            if epoch.shape[1] == self.sampling_rate * self.epoch_length:
                epochs.append(epoch)
                
        return np.array(epochs)

    def extract_features(self, epoch):
        """특성 추출"""
        features = []
        
        for channel in range(epoch.shape[0]):
            # 시간 도메인 특성
            features.extend([
                np.mean(epoch[channel]),
                np.std(epoch[channel]),
                np.max(epoch[channel]),
                np.min(epoch[channel])
            ])
            
            # 주파수 도메인 특성
            freqs, psd = signal.welch(epoch[channel], 
                                    fs=self.sampling_rate,
                                    nperseg=self.sampling_rate)
            
            # 주파수 대역별 파워
            delta = np.mean(psd[(freqs >= 0.5) & (freqs <= 4)])
            theta = np.mean(psd[(freqs >= 4) & (freqs <= 8)])
            alpha = np.mean(psd[(freqs >= 8) & (freqs <= 13)])
            beta = np.mean(psd[(freqs >= 13) & (freqs <= 30)])
            
            features.extend([delta, theta, alpha, beta])
            
        return np.array(features)

class SleepStageClassifier:
    def __init__(self):
        self.model = self._build_model()
        
    def _build_model(self):
        """CNN 모델 구축"""
        from tensorflow.keras.models import Sequential
        from tensorflow.keras.layers import Conv1D, MaxPooling1D, Dense, Dropout, Flatten
        
        model = Sequential([
            Conv1D(64, 3, activation='relu', input_shape=(None, 1)),
            MaxPooling1D(2),
            Conv1D(128, 3, activation='relu'),
            MaxPooling1D(2),
            Conv1D(128, 3, activation='relu'),
            MaxPooling1D(2),
            Flatten(),
            Dense(128, activation='relu'),
            Dropout(0.5),
            Dense(5, activation='softmax')  # 5개 수면 단계
        ])
        
        model.compile(optimizer='adam',
                     loss='categorical_crossentropy',
                     metrics=['accuracy'])
        
        return model
    
    def train(self, X_train, y_train, validation_data, epochs=50):
        """모델 학습"""
        history = self.model.fit(
            X_train, y_train,
            validation_data=validation_data,
            epochs=epochs,
            batch_size=32
        )
        return history

def main():
    # 데이터 처리기 초기화
    processor = SleepEDFProcessor("path_to_sleep_edf_data")
    
    # 데이터 로드 및 전처리
    edf_files = processor.load_edf_files()
    
    all_features = []
    all_labels = []
    
    for edf_file in edf_files:
        # 데이터 전처리
        epochs = processor.preprocess_single_file(edf_file)
        
        # 특성 추출
        for epoch in epochs:
            features = processor.extract_features(epoch)
            all_features.append(features)
            
        # 레이블 로드 (실제 구현에서는 해당 파일의 레이블을 로드해야 함)
        # 여기서는 예시로 더미 레이블 생성
        labels = np.random.randint(0, 5, size=len(epochs))
        all_labels.extend(labels)
    
    # 데이터 분할
    X_train, X_test, y_train, y_test = train_test_split(
        np.array(all_features),
        np.array(all_labels),
        test_size=0.2,
        random_state=42
    )
    
    # 레이블 원-핫 인코딩
    y_train = to_categorical(y_train)
    y_test = to_categorical(y_test)
    
    # 모델 학습
    classifier = SleepStageClassifier()
    history = classifier.train(
        X_train, y_train,
        validation_data=(X_test, y_test)
    )
    
    return classifier, history

if __name__ == "__main__":
    classifier, history = main()