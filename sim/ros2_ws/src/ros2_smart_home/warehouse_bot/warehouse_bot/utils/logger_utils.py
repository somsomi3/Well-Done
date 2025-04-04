# warehouse_bot/utils/logger_utils.py

from datetime import datetime

# 전역 로그 출력 설정 (전체 토글)
GLOBAL_LOG_ENABLED = True

# 파일별 로그 출력 설정 (파일 이름: True/False)
LOGGING_CONFIG = {
    # 예시
    "mapper": False,
    "mapping": False,
    "auto_mapping": True,
    # 필요한 파일마다 추가
}


def print_log(level, logger, msg, file_tag="default"):
    """시간 포함 로그 출력 함수"""
    if not GLOBAL_LOG_ENABLED:
        return
    if not LOGGING_CONFIG.get(file_tag, True):  # 해당 파일 로그 비활성화 시
        return
    if logger is None:
        return

    timestamp = datetime.now().strftime("[%Y-%m-%d %H:%M:%S]")
    full_msg = f"{timestamp} {msg}"

    if level == "info":
        logger.info(full_msg)
    elif level == "warn":
        logger.warn(full_msg)
    elif level == "error":
        logger.error(full_msg)
