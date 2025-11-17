import Explorer
from LiDAR import lidar
import STM
import time
import numpy as np


lidar_ = lidar(port="/dev/ttyAMA4")
lidar_.turn_on()
def get_dist(target_angle, angle_range=15):
    
    target_rad = np.deg2rad(target_angle)
    range_rad = np.deg2rad(angle_range)

    nearby_points = []
    for point in lidar_.points:
        angle_diff = abs((point[0] % (2 * np.pi)) - (target_rad % (2 * np.pi)))
        angle_diff = min(angle_diff, 2 * np.pi - angle_diff) 
        
        if angle_diff <= range_rad:
            nearby_points.append(point[1] * np.cos(point[0] - target_rad))
    
    return max(nearby_points)
# def get_dist(angle):  # this function is expected to use after executing lidar.update()
#     rad_a = np.deg2rad(angle)
#     # print(rad_a)
#     rad_a %= np.pi * 2
#     # print(rad_a)
#     min_abs = float('inf')
#     min_dist = -1
#     min_index = -1
#     for i, d in enumerate(lidar_.points):
#         if abs(d[0] % (np.pi*2) - rad_a) < min_abs:
#             min_dist = d[1]
#             min_abs = abs(d[0] - rad_a)
#             min_index = i
#     # print(i, lidar.points[i])
#     return min_dist


Explorer_ = Explorer.Explorer(
    moveForwardFunc=lambda: moveForward(),
    turnFunc=lambda angle: turn(angle),
    getTileInfoFunc=lambda: getTileInfo()
)

robot_pos = (0, 0)
robot_absolute_dir = 0  # North


def getTileInfo():
    info = {}
    for direction in [0, 90, 180, 270]:
        info[direction] = "wall" if get_dist(
            (direction-robot_absolute_dir) % 360) < 0.25 else "empty"
    return info




def moveForward():
    STM.drive(60, 0)
    time.sleep(1.2)
    STM.stop()
    global robot_pos
    if robot_absolute_dir == 0:
        robot_pos = (robot_pos[0]+1, robot_pos[1])
    elif robot_absolute_dir == 90:
        robot_pos = (robot_pos[0], robot_pos[1] + 1)
    elif robot_absolute_dir == 180:
        robot_pos = (robot_pos[0] - 1, robot_pos[1])
    elif robot_absolute_dir == 270:
        robot_pos = (robot_pos[0], robot_pos[1] - 1)
    else:
        print("Error: Invalid robot_absolute_dir")


def turn(angle):
    # print(angle)
    # return
    turnSpeed = 60
    turn90time = 0.7
    global robot_absolute_dir
    robot_absolute_dir = (robot_absolute_dir + angle) % 360
    if angle == 90:
        STM.drive(turnSpeed, -100)
        time.sleep(turn90time)
        STM.stop()
    elif angle == -90:
        STM.drive(turnSpeed, 100)
        time.sleep(turn90time)
        STM.stop()
    else:
        print("Error: Invalid turn angle")

if __name__ == "__main__":
    try:
        # turn(90)
        while True:
            STM.read_stm2()
            continue
            # turn(90)
            # continue
            lidar_.update()
            print(getTileInfo())
            time.sleep(0.4)
            if Explorer_.ExploreStep():
                break
            # time.sleep(3)
            # if get_dist(0)>0.4:
            #     STM.drive(40,0)
            # else:
            #     STM.stop()
            # lidar_.export_scan_plot()
    except KeyboardInterrupt:
        pass
    finally:
        lidar_.turn_off()
        STM.stop()
        lidar_.close_port()
        STM.closeSTMPort()