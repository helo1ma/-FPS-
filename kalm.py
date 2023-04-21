"""
卡尔曼滤波算法，此模块由改装
"""
from pykalman import KalmanFilter


def Kalman1D(observations, damping=0.0001):
    # To return the smoothed time series data
    observation_covariance = damping
    observations_0 = int(observations[0])
    initial_value_guess = observations_0
    transition_matrix = 1
    transition_covariance = 0.1
    t = 599
    for i in range(t):
        kf = KalmanFilter(
            initial_state_mean=initial_value_guess,
            initial_state_covariance=observation_covariance,  # 观测偏差
            observation_covariance=observation_covariance,  # 观测偏差
            transition_covariance=transition_covariance,  # 预测偏差
            transition_matrices=transition_matrix
        )
        pred_state, state_cov = kf.smooth(observations)
        return pred_state
