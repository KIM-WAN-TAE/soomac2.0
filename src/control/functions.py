import numpy as np

def grip_mm_to_value(grip_mm):
    grip_divide_mm_arr = [20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75]
    grip_divide_dynamixel_arr =  [2200, 2403, 2550, 2700, 2800, 2900, 3000, 3125, 3225, 3350, 3500, 3800]
    grip_value = 3800

    if grip_mm <= grip_divide_mm_arr[0]:
        grip_value = grip_divide_dynamixel_arr[0]

    elif grip_mm <= grip_divide_mm_arr[11]:
        for i in range(1, 12):
            if grip_mm <= grip_divide_mm_arr[i]:
                value_per_mm = (grip_divide_dynamixel_arr[i]-grip_divide_dynamixel_arr[i-1])/(grip_divide_mm_arr[i]-grip_divide_mm_arr[i-1])
                grip_value = grip_divide_dynamixel_arr[i-1] + (grip_mm-grip_divide_mm_arr[i-1])*value_per_mm 
                break
    else:
        grip_value = grip_divide_dynamixel_arr[11]
    return grip_value

def degree_to_dynamixel_value(degree, grip_mm, grip_option="mm", N=0): # input : (1,6) -> output : (1,6) // 5축 degree + gripper_value -> 6개의 모터 value
    
    change_para = 1024/90
    position_dynamixel = np.array(degree)
    
    # gripper_open = 3666
    # gripper_close = 2236
    # gripper_open_mm = 72 #72
    # gripper_close_mm = 19 #15.9
    
    ## about axix
    min_limits = np.array([683, 853, 565, 745, 1024])
    max_limits = np.array([3413, 3072, 3584, 3444, 3072])

    position_dynamixel[0] = (position_dynamixel[0]+180)*change_para
    position_dynamixel[1] = (position_dynamixel[1]+90)*change_para
    position_dynamixel[2] = (position_dynamixel[2]+180)*change_para
    position_dynamixel[3] = (position_dynamixel[3]+180)*change_para
    position_dynamixel[4] = (position_dynamixel[4]+180)*change_para  
    
    for i in range(5):
        if position_dynamixel[i] < min_limits[i]:
            position_dynamixel[i] = min_limits[i]
        elif position_dynamixel[i] > max_limits[i]:
            position_dynamixel[i] = max_limits[i]

    # about gripper
    if grip_option == "mm":
        grip_value = grip_mm_to_value(grip_mm)
    else:
        grip_value = grip_mm
                      
    position_dynamixel = position_dynamixel.astype(int)
    position_dynamixel = np.append(position_dynamixel, int(grip_value))
    return position_dynamixel

def cubic_trajectory(th_i, th_f, min_n = 5, max_n = 15, N_req=0): 

    # N 계산 
    delta_value = np.max(np.abs(th_f[:5] - th_i[:5])) # 모터들의 변화량 계산 후, 모든 모터에서 발생할 수 있는 최대 변화량
    max_delta_value = (1024/90)*100                     # 모터에서 발생할 수 있는 최대 변화량
    N = min_n + (delta_value / max_delta_value) * (max_n - min_n)    
    N = int(N)
    if N > max_n:
        N = max_n
    
    if N_req != 0:
        N = N_req
    t = np.linspace(0, 1, N)
    
    # 3차 다항식 계수: 초기 속도와 최종 속도를 0으로 설정 (s_curve)
    a0 = th_i[:, np.newaxis]
    a1 = 0
    a2 = 3 * (th_f - th_i)[:, np.newaxis]
    a3 = -2 * (th_f - th_i)[:, np.newaxis]
    
    # 3차 다항식을 통해 각도를 계산 (각 행이 하나의 trajectory)
    theta = a0 + a1 * t + a2 * t**2 + a3 * t**3  
    print('trajectory N : ',N)
    return theta.T
    
def gripper_trajectory(th_i, th_f, N = 5): 
    t = np.linspace(0, 1, N)
    # 3차 다항식 계수: 초기 속도와 최종 속도를 0으로 설정 (s_curve)
    a0 = th_i[:, np.newaxis]
    a1 = 0
    a2 = 3 * (th_f - th_i)[:, np.newaxis]
    a3 = -2 * (th_f - th_i)[:, np.newaxis]
    
    # 3차 다항식을 통해 각도를 계산 (각 행이 하나의 trajectory)
    theta = a0 + a1 * t + a2 * t**2 + a3 * t**3  
    print('trajectory N : ',N)
    return theta.T

