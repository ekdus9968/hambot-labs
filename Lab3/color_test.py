import time
import numpy as np
from robot_systems.robot import HamBot

# -------------------------------
# 색상 테스트용
# -------------------------------
TARGET_COLOR = (255, 220, 0)   # 기존 목표 노란색
TOLERANCE = 50                 # ± 허용 오차
SAMPLE_X = 320                 # 테스트할 X 좌표 (프레임 중앙 등)
SAMPLE_Y = 240                 # 테스트할 Y 좌표


def main():
    try:
        bot = HamBot(lidar_enabled=True, camera_enabled=True)
        
        time.sleep(1)  # 카메라 초기화 대기

        print("Camera color test started. Press Ctrl+C to stop.")
        
        while True:
            while True:
                scan = bot.get_range_image()
                if scan is not None and len(scan) > 0:
                    center_idx = len(scan) // 2
                    print(f"Front distance: {scan[center_idx]:.3f} m")
                else:
                    print("No LiDAR data received")
                
                forward_distance = np.min(scan[175:185]) /600
                
                if np.isnan(forward_distance) or np.isinf(forward_distance):
                    forward_distance = 9999.999999
                
                frame = bot.camera.get_frame()  # get_image() → get_frame() 변경
                if frame is None:
                    continue

                H, W = frame.shape[:2]

                # 지정 좌표 픽셀 값 확인
                x = min(max(SAMPLE_X, 0), W-1)
                y = min(max(SAMPLE_Y, 0), H-1)
                pixel_color = frame[y, x]  # (R, G, B)
                
                r_diff = abs(int(pixel_color[0]) - TARGET_COLOR[0])
                g_diff = abs(int(pixel_color[1]) - TARGET_COLOR[1])
                b_diff = abs(int(pixel_color[2]) - TARGET_COLOR[2])
                
                within_tolerance = r_diff <= TOLERANCE and g_diff <= TOLERANCE and b_diff <= TOLERANCE
                print(f"Forward: {forward_distance:.3f}")
                print(f"Pixel ({x},{y}) RGB: {pixel_color}, Diff: (R:{r_diff}, G:{g_diff}, B:{b_diff}), Match: {within_tolerance}")

                time.sleep(0.2)


    except KeyboardInterrupt:
        print("Test stopped by user.")
    finally:
        bot.camera.stop()
        print("Camera disabled.")

if __name__ == "__main__":
    main()

"""Distance from the object 200
Pixel (320,240) RGB: [255   0 173], Diff: (R:0, G:220, B:173), Match: False
Pixel (320,240) RGB: [255   0 163], Diff: (R:0, G:220, B:163), Match: False
Pixel (320,240) RGB: [255   0 171], Diff: (R:0, G:220, B:171), Match: False
Pixel (320,240) RGB: [254   0 170], Diff: (R:1, G:220, B:170), Match: False
Pixel (320,240) RGB: [255   0 169], Diff: (R:0, G:220, B:169), Match: False
Pixel (320,240) RGB: [255   0 169], Diff: (R:0, G:220, B:169), Match: False
Pixel (320,240) RGB: [255   0 167], Diff: (R:0, G:220, B:167), Match: False
Pixel (320,240) RGB: [255   0 165], Diff: (R:0, G:220, B:165), Match: False
Pixel (320,240) RGB: [255   0 171], Diff: (R:0, G:220, B:171), Match: False
Pixel (320,240) RGB: [255   0 168], Diff: (R:0, G:220, B:168), Match: False
Pixel (320,240) RGB: [255   0 171], Diff: (R:0, G:220, B:171), Match: False
Pixel (320,240) RGB: [255   0 170], Diff: (R:0, G:220, B:170), Match: False
Pixel (320,240) RGB: [255   0 169], Diff: (R:0, G:220, B:169), Match: False

Distance = 1200:
Pixel (320,240) RGB: [171 141 188], Diff: (R:84, G:79, B:188), Match: False
Pixel (320,240) RGB: [171 136 183], Diff: (R:84, G:84, B:183), Match: False
Pixel (320,240) RGB: [170 149 187], Diff: (R:85, G:71, B:187), Match: False
Pixel (320,240) RGB: [168 144 183], Diff: (R:87, G:76, B:183), Match: False
Pixel (320,240) RGB: [169 141 182], Diff: (R:86, G:79, B:182), Match: False
Pixel (320,240) RGB: [167 138 183], Diff: (R:88, G:82, B:183), Match: False
Pixel (320,240) RGB: [175 147 189], Diff: (R:80, G:73, B:189), Match: False
Pixel (320,240) RGB: [174 153 188], Diff: (R:81, G:67, B:188), Match: False
Pixel (320,240) RGB: [166 143 179], Diff: (R:89, G:77, B:179), Match: False
Pixel (320,240) RGB: [169 143 185], Diff: (R:86, G:77, B:185), Match: False
Pixel (320,240) RGB: [166 140 186], Diff: (R:89, G:80, B:186), Match: False
Pixel (320,240) RGB: [170 148 185], Diff: (R:85, G:72, B:185), Match: False
Pixel (320,240) RGB: [172 145 189], Diff: (R:83, G:75, B:189), Match: False
Pixel (320,240) RGB: [171 143 185], Diff: (R:84, G:77, B:185), Match: False
Pixel (320,240) RGB: [170 145 183], Diff: (R:85, G:75, B:183), Match: False
Pixel (320,240) RGB: [171 147 186], Diff: (R:84, G:73, B:186), Match: False
Pixel (320,240) RGB: [171 139 184], Diff: (R:84, G:81, B:184), Match: False
Pixel (320,240) RGB: [174 142 188], Diff: (R:81, G:78, B:188), Match: False
Pixel (320,240) RGB: [171 140 188], Diff: (R:84, G:80, B:188), Match: False
Pixel (320,240) RGB: [172 141 191], Diff: (R:83, G:79, B:191), Match: False
^CShutdown signal received. Stopping motors...

D = 2400
Pixel (320,240) RGB: [175 168 151], Diff: (R:80, G:52, B:151), Match: False
Pixel (320,240) RGB: [186 181 146], Diff: (R:69, G:39, B:146), Match: False
Pixel (320,240) RGB: [181 175 151], Diff: (R:74, G:45, B:151), Match: False
Pixel (320,240) RGB: [182 178 157], Diff: (R:73, G:42, B:157), Match: False
Pixel (320,240) RGB: [185 177 161], Diff: (R:70, G:43, B:161), Match: False
Pixel (320,240) RGB: [175 174 148], Diff: (R:80, G:46, B:148), Match: False
Pixel (320,240) RGB: [187 183 156], Diff: (R:68, G:37, B:156), Match: False
Pixel (320,240) RGB: [178 171 158], Diff: (R:77, G:49, B:158), Match: False
Pixel (320,240) RGB: [193 183 156], Diff: (R:62, G:37, B:156), Match: False
Pixel (320,240) RGB: [175 167 150], Diff: (R:80, G:53, B:150), Match: False
Pixel (320,240) RGB: [182 178 157], Diff: (R:73, G:42, B:157), Match: False
Pixel (320,240) RGB: [190 187 157], Diff: (R:65, G:33, B:157), Match: False
Pixel (320,240) RGB: [191 182 165], Diff: (R:64, G:38, B:165), Match: False
Pixel (320,240) RGB: [175 171 149], Diff: (R:80, G:49, B:149), Match: False
Pixel (320,240) RGB: [188 182 155], Diff: (R:67, G:38, B:155), Match: False
Pixel (320,240) RGB: [184 178 158], Diff: (R:71, G:42, B:158), Match: False
Pixel (320,240) RGB: [180 166 150], Diff: (R:75, G:54, B:150), Match: False
Pixel (320,240) RGB: [178 169 153], Diff: (R:77, G:51, B:153), Match: False
Pixel (320,240) RGB: [183 178 159], Diff: (R:72, G:42, B:159), Match: False
Pixel (320,240) RGB: [191 184 166], Diff: (R:64, G:36, B:166), Match: False
Pixel (320,240) RGB: [185 172 162], Diff: (R:70, G:48, B:162), Match: False
Pixel (320,240) RGB: [193 188 161], Diff: (R:62, G:3 이면 어떻게 색깔 잡으면 좋을까? ㅖPINK = (?, ?, 0)   # attempting to make yellow
color_tolerance  = ?
"""