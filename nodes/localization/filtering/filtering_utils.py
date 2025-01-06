import torch
from pymlg.torch import SO3


def calculate_rotation_error(C_bar : torch.Tensor, C_hat : torch.Tensor):
    """
    calculates the geodesic distance and corresponding rotation error between two [N, 3, 3] rotation histories

    individual error computation is e = Log(C_hat @ C_t.T)

    Parameters
    ----------
    C_bar : torch.Tensor
        [N, 3, 3] set of rotation matrices
    C_hat : torch.Tensor
        [N, 3, 3] set of estimated rotation matrices

    Returns
    -------
    e_C : torch.Tensor
        [N, 3, 1] set of rotational errors
    """
    e_C = SO3.Log((C_hat @ C_bar.transpose(1, 2)))

    return e_C