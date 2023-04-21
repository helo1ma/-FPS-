print('此模块由Ma制作,QQ:1304428142')
class PID_1:
    def __init__(self, dt, max, min, Kp, Kd, Ki):
        self.dt = dt  # 循环时长
        self.max = max  # 操作变量最大值
        self.min = min  # 操作变量最小值
        self.Kp = Kp  # 比例增益
        self.Kd = Kd  # 积分增益
        self.Ki = Ki  # 微分增益
        self.integral = 0  # 直到上一次的误差值
        self.pre_error = 0  # 上一次的误差值

    def calculate(self, setPoint, pv):
        # 其中 pv:process value 即过程值
        error = setPoint - pv  # 误差
        Pout = self.Kp * error  # 比例项
        self.integral += error * self.dt
        Iout = self.Ki * self.integral  # 积分项
        derivative = (error - self.pre_error) / self.dt
        Dout = self.Kd * derivative  # 微分项

        output = Pout + Iout + Dout  # 新的目标值

        if output > self.max:
            output = self.max
        elif output < self.min:
            output = self.min

        self.pre_error = error  # 保存本次误差，以供下次计算
        return output

    def apex_pid_x(self, val_x, ts_x):  # 计算次数
        # x方向PID数据
        dt_x, x_max, x_min, Kp_x, Kd_x, Ki_x = self.dt, self.max, self.min, self.Kp, self.Kd, self.Ki
        c_x = PID_1(dt_x, x_max, x_min, Kp_x, Kd_x, Ki_x)
        val_x = val_x  # 填鼠标偏移量
        ts_x = ts_x  # 推理次数
        z_x = 0
        # x的PID推理过程
        for x in range(ts_x):
            inc_x = c_x.calculate(0, val_x)
            z_x -= inc_x
        return z_x

    def apex_pid_y(self, val_y, ts_y):  # 计算次数
        # y方向PID数据
        dt_y, y_max, y_min, Kp_y, Kd_y, Ki_y = self.dt, self.max, self.min, self.Kp, self.Kd, self.Ki
        c_y = PID_1(dt_y, y_max, y_min, Kp_y, Kd_y, Ki_y)
        val_y = val_y
        ts_y = ts_y
        z_y = 0
        # x的PID推理过程
        for y in range(ts_y):
            inc_x = c_y.calculate(0, val_y)
            z_y -= inc_x
        return z_y
