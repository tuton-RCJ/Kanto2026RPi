import serial
import time

class TSD10:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=0.1):
        """
        TSD10センサークラスの初期化
        :param port: シリアルポートのパス (環境に合わせて変更してください)
        :param baudrate: ボーレート (TSD10の仕様に合わせて調整してください)
        """
        self._ser = serial.Serial(port, baudrate, timeout=timeout)
        self._state = 0
        self._distL = 0
        self._distH = 0
        self._distance = 0

    def start(self):
        """センサーの動作開始（計測開始コマンド送信）"""
        cmd = bytes([0x5A, 0x0A, 0x02, 0x02, 0x00, 0xF1])
        self._ser.write(cmd)

    def stop(self):
        """センサーの動作停止"""
        cmd = bytes([0x5A, 0x0A, 0x02, 0x00, 0x00, 0xF3])
        self._ser.write(cmd)
            
    def update(self) -> bool:
        """
        非同期でシリアルデータを読み取り、最新の距離を更新する
        :return: 新しいデータを受信して更新された場合は True、そうでない場合は False
        """
         # 1. まず現在バッファに溜まっている古いデータをすべてクリア
        if self._ser.in_waiting > 0:
            self._ser.reset_input_buffer()
        
        # 状態リセット
        self._state = 0
        
        # 2. 最新のデータが届くまでわずかに待機しつつ、1フレーム分(4バイト)を同期的に読み取る
        # タイムアウト（デフォルト0.1秒）を設定しているため、フリーズはしません
        start_time = time.time()
        while (time.time() - start_time) < 0.05:  # 最大100ms待つ
            if self._ser.in_waiting > 0:
                b = ord(self._ser.read(1))

                if self._state == 0:
                    if b == 0x5C:  # 最新のヘッダーを見つけた
                        self._state = 1
                elif self._state == 1:
                    self._distL = b
                    self._state = 2
                elif self._state == 2:
                    self._distH = b
                    self._state = 3
                elif self._state == 3:
                    self._state = 0
                    if self._verify_checksum(self._distL, self._distH, b):
                        self._distance = self._distL | (self._distH << 8)
                        return True
                    else:
                        return False
        return False

    def get_distance(self) -> int:
        """最新の距離を取得 (mm) / 65535は測定範囲外エラー"""
        return self._distance

    def _verify_checksum(self, l: int, h: int, check: int) -> bool:
        """チェックサムの検証"""
        # Pythonは型上限がないため、& 0xFF で8ビット（uint8_t相当）にマスクします
        total_sum = (l + h) & 0xFF
        # ビット反転（~）を行い、再度8ビットにマスク
        inverted = (~total_sum) & 0xFF
        return inverted == check

    def close(self):
        """シリアルポートを閉じる"""
        if self._ser and self._ser.is_open:
            self._ser.close()