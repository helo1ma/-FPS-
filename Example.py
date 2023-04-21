mouse_offset_ratio = (2.3, 1)# 鼠标偏移量可以自己设置

# PID用法
p_y = PID_m.PID_1(0.1, 0.01, -0.01, 0.4, 0.07, 1)
p_x = PID_m.PID_1(0.1, 0.01, -0.01, 0.8, 0.07, 1)
x_pid = p_x.apex_pid_x(mouse_offset_ratio[0], rifle_x)
y_pid = p_y.apex_pid_y(mouse_offset_ratio[1], rifle_y)  # 第二个参数是锁定速度越大越快
moveR(x * x_pid, y * y_pid)

# kalman用法
kf_pos_x = Kalman1D(pos_min, 0.0001).astype(np.int_)
kf_pos_y = Kalman1D(pos_min, 0.0001).astype(np.int_)
moveR(kf_pos_x, kf_pos_y)

#也可以两个一起用
moveR(kf_pos_x * kf_pos_y, y * y_pid)