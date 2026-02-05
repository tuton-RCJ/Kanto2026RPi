import cv2
import numpy as np
from picamera2 import Picamera2, Preview
from libcamera import controls
import time
import threading
import atexit

current_frame = None
save_requested = False


class CameraColorDetector:
    def __init__(self):
        self.picam2 = Picamera2()
        self.picam2.start_preview(Preview.NULL)
        self.picam2.start()
        self.picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
        self._configure_camera()
        print("--- 設定完了 ---")

    @staticmethod
    def judge_color(image_bgr):
        # すでにBGRに変換された画像を受け取って判定する
        h, w, _ = image_bgr.shape
        roi_h, roi_w = int(h * 0.2), int(w * 0.2)
        start_y, start_x = (h // 2) -40 - (roi_h // 2), (w // 2) - (roi_w // 2)
        roi = image_bgr[start_y:start_y+roi_h, start_x:start_x+roi_w]

        # BGRからHSVへ
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        avg_hsv = cv2.mean(hsv_roi)
        avg_h, avg_s, avg_v = avg_hsv[0], avg_hsv[1], avg_hsv[2]
        #print(avg_h,avg_s,avg_v)
        if avg_h < 120:
            if avg_v < 200:  return "BLACK"
        
        return "UNKNOWN"

    def _configure_camera(self):
        # メタデータの取得
        props = self.picam2.capture_metadata()

        # 1. 露出とゲインの固定
        exp = props.get('ExposureTime')
        gain = props.get('AnalogueGain')

        if exp and gain:
            exp=30000
            gain=1.0
            self.picam2.set_controls({
                "AeEnable": False,
                "ExposureTime": exp,
                "AnalogueGain": gain
            })
            print(f"露出固定: {exp}us, ゲイン: {gain}")

        # 2. ホワイトバランスの固定 (ColourGains または ColorGains) 104, 233, 171
        # libcameraでは 'ColourGains' (uあり) が一般的です
        awb_gains = props.get('ColourGains') or props.get('ColorGains')

        if awb_gains:
            awb_gains=(1.2,3.8)
            self.picam2.set_controls({
                "AwbEnable": False,
                "ColourGains": awb_gains
            })
            print(f"ホワイトバランス固定: {awb_gains}")
        else:
            # 万が一メタデータから取れない場合は、現在のAwbModeを固定するだけでも効果があります
            self.picam2.set_controls({"AwbMode": controls.AwbModeEnum.Disabled})
            print("ホワイトバランスを現在の状態でロックしました（Disabled）")

    def detectTileColor(self):
        frame_rgb = self.picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        return self.judge_color(frame_bgr)

    def stop(self):
        self.picam2.stop()


def judge_color(image_bgr):
    return CameraColorDetector.judge_color(image_bgr)


_default_detector = None


def _get_default_detector():
    global _default_detector
    if _default_detector is None:
        _default_detector = CameraColorDetector()
        atexit.register(_default_detector.stop)
    return _default_detector


def detectTileColor():
    return _get_default_detector().detectTileColor()

_get_default_detector()