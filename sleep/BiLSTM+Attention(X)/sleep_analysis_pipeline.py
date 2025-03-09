class SleepAnalysisPipeline:
    def __init__(self):
        self.analyzer = SleepAnalyzer(model_type='transformer')
        self.segment_duration = 30  # 30초 단위 분석
        
    def process_sleep_recording(self, audio_file):
        """수면 녹음 처리"""
        # 오디오 로드
        audio, sr = librosa.load(audio_file, sr=22050)
        
        # 수면 분석
        predictions = self.analyzer.analyze_sleep_session(audio, sr)
        
        # 결과 후처리
        processed_results = self.post_process_predictions(predictions)
        
        return self.generate_sleep_report(processed_results)
    
    def post_process_predictions(self, predictions):
        """예측 결과 후처리"""
        # 중간값 필터링으로 노이즈 제거
        smoothed_predictions = self.median_filter(predictions, kernel_size=5)
        
        # 상태 전이 보정
        corrected_predictions = self.correct_state_transitions(smoothed_predictions)
        
        return corrected_predictions
    
    def generate_sleep_report(self, results):
        """수면 보고서 생성"""
        report = {
            'sleep_stages': self.analyze_sleep_stages(results),
            'sleep_quality': self.calculate_sleep_quality(results),
            'sleep_events': self.detect_sleep_events(results),
            'statistics': self.calculate_statistics(results)
        }
        
        return report

# 사용 예시
def analyze_sleep_session(audio_file_path):
    pipeline = SleepAnalysisPipeline()
    report = pipeline.process_sleep_recording(audio_file_path)
    
    # 결과 시각화
    visualize_sleep_report(report)