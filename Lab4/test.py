"""
필요한 function:
360 돌면서 4개의 landmark detect 하고 4개중 하나의 색상이 발견되면 그 위치와 시작의 각도에서 얼마나 돌았는지
저장하고 계속 회전
color_list= [orange, green, blue, pink]
detected_list = [4][2] // 4 colors with ecah distance and delta
def turn_to_detect():
    current_heading = self.bot.get_heading()
    if current_heading is None:
        print("[DEBUG] Warning: current_heading is None, skipping turn")
        return False
    error = abs((current_heading - target_angle + 360) % 360 - 180)
        if error > 180:
            error = 360 - error  # 항상 최소 각도
     print(f"[DEBUG] Turning left only - Current: {current_heading:.2f}, Target: {target_angle:.2f}, Error: {error:.2f}")
    if error < 3:  # ±3° 안이면 멈춤
            self.bot.set_left_motor_speed(0.0)
            self.bot.set_right_motor_speed(0.0)
            print("[DEBUG] Reached target heading, motors stopped")
            return True
        else:
            # HamBot 모터 범위 내 고정 속도
            fixed_speed = 4.0
            self.bot.set_left_motor_speed(-fixed_speed)   # 왼쪽 모터 뒤
            self.bot.set_right_motor_speed(fixed_speed)   # 오른쪽 모터 앞으로
            frame = bot.camera.get_frame() # get_image() → get_frame() 변경 if frame is None:
                continue H, W = frame.shape[:2] # 지정 좌표 픽셀 값 확인 
                x = min(max(SAMPLE_X, 0), W-1) 
                y = min(max(SAMPLE_Y, 0), H-1) 
                pixel_color = frame[y, x] # (R, G, B) 
                r_diff = abs(int(pixel_color[0]) - TARGET_COLOR[0]) 
                g_diff = abs(int(pixel_color[1]) - TARGET_COLOR[1]) 
                b_diff = abs(int(pixel_color[2]) - TARGET_COLOR[2]) 
            print()
            for checking the list of color i want to detect :
                if find:
                    print index of the list 
                    
                    store the distance using lidar and delta how move
                    append a stored list
        
                    within_tolerance = r_diff <= TOLERANCE and g_diff <= TOLERANCE and b_diff <= TOLERANCE 
                    print(f"Forward: {forward_distance:.3f}") print(f"Pixel ({x},{y}) RGB: {pixel_color}, Diff: (R:{r_diff}, G:{g_diff}, B:{b_diff}), Match: {within_tolerance}")
                    
                    continue
                else:
                    print(f"Forward: {forward_distance:.3f}") print(f"Pixel ({x},{y}) RGB: {pixel_color}")
                    conitue
            return False 

"""