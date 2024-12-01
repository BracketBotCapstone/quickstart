from navlie import MeasurementModel
from navlie.lib import SE23State, IMUState

from pymlg.numpy import SO3

import numpy as np

class WheelEncoderGravityAligned(MeasurementModel):
    """

    Wheel Encoder Measurement model for some kind of high-frequency, high-resolution rotary sensor on a wheel with a known radius. This applies the updates in the normal frame direction (that is, along the ground) and is more correct for a flat surface when modelling a moving pendulum.

    Currently, assumes a left perturbation.

    """

    def __init__(self, R : np.array):
        self.R = R

    def evaluate(self, x : IMUState) -> np.ndarray:

        # retrieve state parameters
        C_ab = x.attitude
        v_a = x.velocity

        # compute the gravity-aligned frame
        c21 = C_ab[1, 0]
        c11 = C_ab[0, 0]

        gamma = np.arctan2(c21, c11)
        phi_gamma = np.array([0, 0, gamma])

        C_ga = SO3.Exp(phi_gamma).T

        v_zw_g = C_ga @ v_a

        return v_zw_g
    
    def jacobian(self, x : IMUState) -> np.ndarray:
        C_ab = x.attitude
        v_a = x.velocity

        G_vu = np.zeros((3, 15))

        c21 = C_ab[1, 0]
        c11 = C_ab[0, 0]

        # # compute overall rotation for imu samples
        # anchor_euler = SO3.to_euler(C_k, order="123")
        # C_gamma = filtering_utils.c_3(anchor_euler.unsqueeze(2))
        gamma = np.arctan2(c21, c11)
        phi_gamma = np.array([0, 0, gamma])

        C_ag = SO3.Exp(phi_gamma)

        coeff_y = c11 / (c11**2 + c21**2)
        coeff_x = -c21 / (c11**2 + c21**2)

        B = np.array([1, 0, 0])
        A = np.array([0, 1, 0])
        J = np.array([1, 0, 0])

        d_gamma_phi = -coeff_y * (A @ SO3.wedge(C_ab @ B)) - coeff_x * (
            J @ SO3.wedge(C_ab @ B)
        )

        d_C_gamma = (C_ag @ SO3.wedge(np.array([0, 0, 1]))).T

        G_vu[:, 0:3] = -C_ag.T @ SO3.wedge(v_a) + d_C_gamma @ v_a @ d_gamma_phi

        G_vu[:, 3:6] = C_ag.T

        return G_vu
    
        # G_vu = torch.zeros(1, 3, 15)

        # c21 = C_k[:, 1, 0]
        # c11 = C_k[:, 0, 0]:, 

        # # # compute overall rotation for imu samples
        # # anchor_euler = SO3.to_euler(C_k, order="123")
        # # C_gamma = filtering_utils.c_3(anchor_euler.unsqueeze(2))
        # gamma = torch.arctan2(c21, c11)
        # phi_gamma = torch.Tensor([0, 0, gamma]).view(1, 3, 1)

        # C_ag = SO3.Exp(phi_gamma)
        # alpha = 1 / torch.norm(v_k, dim=1, keepdim=True)

        # v_zw_b = C_k.transpose(1, 2) @ v_k.reshape(3, 1)
        # v_zw_g = C_ag.transpose(1, 2) @ v_k.reshape(3, 1)

        # coeff_y = c11 / (c11**2 + c21**2)
        # coeff_x = -c21 / (c11**2 + c21**2)

        # B = torch.Tensor([1, 0, 0]).view(1, 3, 1)
        # A = torch.Tensor([0, 1, 0]).view(1, 1, 3)
        # J = torch.Tensor([1, 0, 0]).view(1, 1, 3)

        # d_gamma_phi = -coeff_y * (A @ C_k @ SO3.wedge(B)) - coeff_x * (
        #     J @ C_k @ SO3.wedge(B)
        # )

        # d_gamma_phi_v = (v_k @ d_gamma_phi).view(1, 3, 3)

        # d_C_gamma = (C_ag @ SO3.wedge(torch.Tensor([0, 0, 1]).view(1, 3, 1))).transpose(1, 2)

        # G_vu[:, :, 0:3] = (d_C_gamma @ d_gamma_phi_v)

        # G_vu[:, :, 3:6] = (C_ag.transpose(1, 2) @ C_k)
    
    def covariance(self, x : IMUState) -> np.ndarray:
        return self.R

class WheelEncoder(MeasurementModel):
    """
    Wheel Encoder Measurement model for some kind of high-frequency, high-resolution rotary sensor on a wheel with a known radius.
    """

    def __init__(self, R : np.array):
        self.R = R

    def evaluate(self, x : IMUState) -> np.ndarray:
        C_ab = x.attitude
        v_a = x.velocity

        v_b = C_ab.T @ v_a

        return v_b
    
    def jacobian(self, x : IMUState) -> np.ndarray:
        C_ab = x.attitude
        v_a = x.velocity

        jac = np.zeros((3, 15))

        jac[:, 3:6] = np.eye(3)
        jac[:, :3] = C_ab.T @ SO3.wedge(v_a) @ C_ab

        return jac
    
    def covariance(self, x : IMUState) -> np.ndarray:
        return self.R