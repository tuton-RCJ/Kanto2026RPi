import cv2
import numpy as np
from picamera2 import Picamera2, Preview
from libcamera import controls
import time
import threading

current_frame = None
save_requested = False

def judge_color(image_bgr):
    # すでにBGRに変換された画像を受け取って判定する
    h, w, _ = image_bgr.shape
    roi_h, roi_w = int(h * 0.2), int(w * 0.2)
    start_y, start_x = (h // 2) - (roi_h // 2), (w // 2) - (roi_w // 2)
    roi = image_bgr[start_y:start_y+roi_h, start_x:start_x+roi_w]

    # BGRからHSVへ
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    avg_hsv = cv2.mean(hsv_roi)
    avg_h, avg_s, avg_v = avg_hsv[0], avg_hsv[1], avg_hsv[2]

    if avg_s < 50:
        if avg_v > 150: return "WHITE"
        if avg_v < 60:  return "BLACK"
        return "GRAY"

    if (avg_h < 10 or avg_h > 160):
        return "RED"
    elif (100 < avg_h < 140):
        return "BLUE"
    
    return "UNKNOWN"

def input_thread():
    global save_requested
    while True:
        input() # Enter入力を待機
        save_requested = True

# --- メイン処理 ---
picam2 = Picamera2()
picam2.start_preview(Preview.NULL)
picam2.start()
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})

threading.Thread(target=input_thread, daemon=True).start()

    
    
try:
    print("判定中... (Enterで保存 / CTRL+Cで終了)")
    while True:
        # 1. RGB形式で取得
        frame_rgb = picam2.capture_array()
        
        # 2. RGBをBGRに変換（ここが重要！）
        # frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        color = "color"
        current_color = color
        save_requested=True
        # 保存リクエストがあった場合
        if save_requested:
            # timestamp = time.strftime("%Y%m%d-%H%M%S")
            filename = f"result.jpg"
            
            # # 画像に判定結果のテキストを書き込む
            # output_img = frame_bgr.copy()
            # cv2.putText(output_img, f"Detected: {color}", (50, 50), 
            #             cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            # # 中央の判定エリアを枠で囲む
            # h, w, _ = frame_bgr.shape
            # cv2.rectangle(output_img, (int(w*0.4), int(h*0.4)), (int(w*0.6), int(h*0.6)), (0, 255, 0), 2)
            
            # cv2.imwrite(filename, output_img)
            print(f"★画像を保存しました: {filename} (判定: {color})")
            
            save_requested = False # リセット


        # 更新周期を見るために現在時刻を表示
        print(f"\r現在の時刻: {time.perf_counter()} ")
        print(f"\r現在の判定: {current_color} ")
        
        

except KeyboardInterrupt:
    print("\n終了します")
finally:
    picam2.stop()