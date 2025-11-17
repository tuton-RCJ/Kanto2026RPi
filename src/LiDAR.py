import os
import ydlidar
import time
import matplotlib.pyplot as plt
import numpy as np
RMAX = 32.0


class lidar:
    def __init__(self, port="/dev/ttyAMA4"):
        ydlidar.os_init()
        ports = ydlidar.lidarPortList()
        self.port = port

        self.laser = ydlidar.CYdLidar()
        self.laser.setlidaropt(ydlidar.LidarPropSerialPort, self.port)
        self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 230400)
        self.laser.setlidaropt(
            ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        self.laser.setlidaropt(ydlidar.LidarPropDeviceType,
                               ydlidar.YDLIDAR_TYPE_SERIAL)
        self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
        self.laser.setlidaropt(ydlidar.LidarPropSampleRate, 4)
        self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
        self.laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
        self.laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
        self.laser.setlidaropt(ydlidar.LidarPropMaxRange, 16.0)
        self.laser.setlidaropt(ydlidar.LidarPropMinRange, 0.02)
        self.laser.setlidaropt(ydlidar.LidarPropIntenstiy, True)
        self.laser.initialize()
        return

    def turn_on(self):
        return self.laser.turnOn()

    def turn_off(self):
        return self.laser.turnOff()

    def close_port(self):
        return self.laser.disconnecting()

    def scan(self):
        scan = ydlidar.LaserScan()
        r = self.laser.doProcessSimple(scan)
        return scan.points if r else None

    def update(self):
        pts = self.scan()
        angles = np.array([-p.angle + np.pi for p in pts], dtype=np.float32)
        ranges = np.array([p.range for p in pts], dtype=np.float32)
        intensities = np.array([p.intensity for p in pts], dtype=np.float32)
        self.points = np.stack((angles, ranges, intensities), axis=1)

    def export_scan_plot(self):
        # --- プロット設定 ---
        fig = plt.figure()
        ax = plt.subplot(polar=True)
        ax.set_rmax(RMAX)
        ax.grid(True)

        # 散布図描画
        sc = ax.scatter(self.points[:, 0], self.points[:, 1], c=self.points[:, 2],
                        cmap='hsv', alpha=0.95)

        # 保存
        plt.savefig("lidar_scan.png", dpi=300)
        print("✅ Saved LIDAR scan to lidar_scan.png")
        print("Scan received [", self.points.shape[0], "] points")
        plt.close()
