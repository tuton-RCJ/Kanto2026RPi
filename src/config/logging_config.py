"""
ロギング設定モジュール
プロジェクト全体で一貫したログ出力を行うための設定を提供
"""

import logging
import logging.handlers
import os
from datetime import datetime

# ログディレクトリの作成
LOG_DIR = os.path.join(os.path.dirname(__file__), "..", "logs")
if not os.path.exists(LOG_DIR):
    os.makedirs(LOG_DIR)

# ログレベル
LOG_LEVEL = logging.DEBUG

# ログフォーマット設定
LOG_FORMAT = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
LOG_FORMAT_DETAILED = (
    "%(asctime)s - %(name)s - [%(filename)s:%(lineno)d] - %(levelname)s - %(message)s"
)

# ログファイル名（タイムスタンプ付き）
LOG_TIMESTAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
LOG_FILE = os.path.join(LOG_DIR, f"robot_{LOG_TIMESTAMP}.log")


def setup_logging():
    """
    ロギングを初期化する
    コンソール出力とファイル出力の両方を設定
    """
    # ルートロガーの設定
    root_logger = logging.getLogger()
    root_logger.setLevel(LOG_LEVEL)

    # 既存のハンドラをクリア
    root_logger.handlers.clear()

    # コンソールハンドラの設定
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_formatter = logging.Formatter(LOG_FORMAT)
    console_handler.setFormatter(console_formatter)
    root_logger.addHandler(console_handler)

    # ファイルハンドラの設定（ローテーション機能付き）
    file_handler = logging.handlers.RotatingFileHandler(
        filename=LOG_FILE,
        maxBytes=10 * 1024 * 1024,  # 10MB
        backupCount=5,  # 最大5世代まで保持
        encoding="utf-8",
    )
    file_handler.setLevel(LOG_LEVEL)
    file_formatter = logging.Formatter(LOG_FORMAT_DETAILED)
    file_handler.setFormatter(file_formatter)
    root_logger.addHandler(file_handler)

    root_logger.info(f"Logging initialized. Log file: {LOG_FILE}")


def get_logger(name: str) -> logging.Logger:
    """
    モジュール別のロガーを取得する

    Args:
        name: モジュール名（通常は __name__ を指定）

    Returns:
        logging.Logger: 設定済みのロガーインスタンス
    """
    return logging.getLogger(name)
