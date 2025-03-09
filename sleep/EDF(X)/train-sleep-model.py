
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import numpy as np
import mne

# 1. 데이터 로딩 및 전처리
class SleepEDFDataset(Dataset):
    def __init__(self, edf_files, hypnogram_files):
        self.edf_files = edf_files
        self.hypnogram_files = hypnogram_files

    def __len__(self):
        return len(self.edf_files)

    def __getitem__(self, idx):
        # EDF 파일 로드
        raw = mne.io.read_raw_edf(self.edf_files[idx], preload=True)
        
        # 필요한 채널 선택 (예: EEG Fpz-Cz, EOG horizontal)
        raw.pick_channels(['EEG Fpz-Cz', 'EOG horizontal'])
        
        # 데이터 추출 및 리샘플링
        data = raw.get_data()
        data = mne.filter.resample(data, down=raw.info['sfreq'] / 100)  # 100Hz로 리샘플링
        
        # 30초 에폭으로 분할
        epoch_length = 30 * 100  # 30초 * 100Hz
        epochs = [data[:, i:i+epoch_length] for i in range(0, data.shape[1], epoch_length)]
        
        # hypnogram 로드 및 레이블 생성
        hypnogram = np.loadtxt(self.hypnogram_files[idx])
        labels = hypnogram[:len(epochs)]
        
        return torch.FloatTensor(epochs), torch.LongTensor(labels)

# 2. 모델 정의
class SleepEDFModel(nn.Module):
    def __init__(self, input_channels=2, sequence_length=3000, num_classes=5):
        super().__init__()
        self.conv1 = nn.Conv1d(input_channels, 64, kernel_size=50, stride=6)
        self.conv2 = nn.Conv1d(64, 128, kernel_size=8, stride=1)
        self.conv3 = nn.Conv1d(128, 128, kernel_size=8, stride=1)
        self.conv4 = nn.Conv1d(128, 128, kernel_size=8, stride=1)
        
        self.fc1 = nn.Linear(128 * 82, 1024)
        self.fc2 = nn.Linear(1024, num_classes)
        
    def forward(self, x):
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = F.relu(self.conv4(x))
        
        x = x.view(x.size(0), -1)
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        return x

# SleepAnalyzer 클래스 수정
class SleepAnalyzer:
    def __init__(self, device='cuda'):
        self.device = device
        self.model = SleepEDFModel().to(device)
    
    def analyze_sleep_session(self, eeg_data):
        self.model.eval()
        with torch.no_grad():
            predictions = self.model(eeg_data)
            predictions = F.softmax(predictions, dim=-1)
        return predictions.cpu().numpy()
# 3. 학습 루프
def train_model(train_edf_files, train_hypnogram_files):
    # 데이터셋 및 DataLoader 생성
    train_dataset = SleepEDFDataset(train_edf_files, train_hypnogram_files)
    train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True)

    # 모델 초기화
    sleep_analyzer = SleepAnalyzer(device='cuda')
    model = sleep_analyzer.model

    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=0.001)

    num_epochs = 50
    for epoch in range(num_epochs):
        model.train()
        for batch_data, batch_labels in train_loader:
            batch_data, batch_labels = batch_data.to(sleep_analyzer.device), batch_labels.to(sleep_analyzer.device)
            
            optimizer.zero_grad()
            outputs = model(batch_data)
            loss = criterion(outputs, batch_labels)
            loss.backward()
            optimizer.step()
        
        print(f"Epoch {epoch+1}/{num_epochs}, Loss: {loss.item()}")

    # 모델 저장
    torch.save(model.state_dict(), "sleep_edf_model.pth")

if __name__ == "__main__":
    # 데이터 경로 설정 (실제 파일 경로로 수정 필요)
    train_edf_files = ['path/to/edf1.edf', 'path/to/edf2.edf', ...]
    train_hypnogram_files = ['path/to/hypno1.txt', 'path/to/hypno2.txt', ...]

    # 모델 학습 실행
    train_model(train_edf_files, train_hypnogram_files)