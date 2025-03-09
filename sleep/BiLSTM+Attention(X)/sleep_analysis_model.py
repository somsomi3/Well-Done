import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader
import numpy as np

class SleepTransformerModel(nn.Module):
    def __init__(self, input_dim, num_classes, d_model=128, nhead=8, 
                 num_layers=6, dropout=0.1):
        super().__init__()
        
        # 입력 특성을 임베딩으로 변환
        self.input_projection = nn.Linear(input_dim, d_model)
        
        # Positional Encoding
        self.pos_encoder = PositionalEncoding(d_model, dropout)
        
        # Transformer Encoder
        encoder_layers = nn.TransformerEncoderLayer(
            d_model=d_model,
            nhead=nhead,
            dim_feedforward=d_model*4,
            dropout=dropout
        )
        self.transformer_encoder = nn.TransformerEncoder(
            encoder_layers,
            num_layers=num_layers
        )
        
        # 출력 레이어
        self.output_layer = nn.Linear(d_model, num_classes)
        
    def forward(self, x):
        # x shape: (batch, sequence_length, input_dim)
        x = self.input_projection(x)
        x = self.pos_encoder(x)
        x = self.transformer_encoder(x)
        return self.output_layer(x)

class BiLSTMAttentionModel(nn.Module):
    def __init__(self, input_dim, hidden_dim, num_classes, num_layers=2, dropout=0.1):
        super().__init__()
        
        self.lstm = nn.LSTM(
            input_size=input_dim,
            hidden_size=hidden_dim,
            num_layers=num_layers,
            bidirectional=True,
            dropout=dropout,
            batch_first=True
        )
        
        self.attention = MultiHeadAttention(hidden_dim*2, 8)
        self.classifier = nn.Linear(hidden_dim*2, num_classes)
        
    def forward(self, x):
        # LSTM 처리
        lstm_out, _ = self.lstm(x)
        
        # Attention 적용
        attended, _ = self.attention(lstm_out, lstm_out, lstm_out)
        
        # 분류
        output = self.classifier(attended)
        return output

class SleepAnalyzer:
    def __init__(self, model_type='transformer', device='cuda'):
        self.device = device
        self.model_type = model_type
        self.model = self._create_model()
        self.feature_extractor = AudioFeatureExtractor()
        
    def _create_model(self):
        if self.model_type == 'transformer':
            return SleepTransformerModel(
                input_dim=128,  # MFCC + 추가 특성
                num_classes=6,  # 수면 단계 수
                d_model=256,
                nhead=8,
                num_layers=6
            ).to(self.device)
        else:
            return BiLSTMAttentionModel(
                input_dim=128,
                hidden_dim=256,
                num_classes=6
            ).to(self.device)
    
    def analyze_sleep_session(self, audio_data, sr):
        """전체 수면 세션 분석"""
        # 특성 추출
        features = self.feature_extractor.extract_features(audio_data, sr)
        
        # 데이터 준비
        features_tensor = torch.FloatTensor(features).unsqueeze(0).to(self.device)
        
        # 예측
        with torch.no_grad():
            predictions = self.model(features_tensor)
            predictions = F.softmax(predictions, dim=-1)
            
        return predictions.cpu().numpy()

class AudioFeatureExtractor:
    def __init__(self):
        self.n_mfcc = 40
        self.frame_length = 2048
        self.hop_length = 512
        
    def extract_features(self, audio, sr):
        """고급 오디오 특성 추출"""
        features = []
        
        # MFCC
        mfccs = librosa.feature.mfcc(
            y=audio,
            sr=sr,
            n_mfcc=self.n_mfcc,
            n_fft=self.frame_length,
            hop_length=self.hop_length
        )
        
        # Spectral Centroid
        spectral_centroids = librosa.feature.spectral_centroid(
            y=audio,
            sr=sr,
            n_fft=self.frame_length,
            hop_length=self.hop_length
        )
        
        # Spectral Rolloff
        spectral_rolloff = librosa.feature.spectral_rolloff(
            y=audio,
            sr=sr,
            n_fft=self.frame_length,
            hop_length=self.hop_length
        )
        
        # Zero Crossing Rate
        zcr = librosa.feature.zero_crossing_rate(
            audio,
            frame_length=self.frame_length,
            hop_length=self.hop_length
        )
        
        # RMS Energy
        rms = librosa.feature.rms(
            y=audio,
            frame_length=self.frame_length,
            hop_length=self.hop_length
        )
        
        # 특성 결합
        features = np.concatenate([
            mfccs,
            spectral_centroids,
            spectral_rolloff,
            zcr,
            rms
        ], axis=0)
        
        return features.T
