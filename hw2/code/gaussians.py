import math
import torch
import numpy as np

from typing import Tuple, Optional
from utils import quaternion_to_matrix
from camera import BaseCamera
from utils import load_gaussians_from_ply
import torch.nn as nn

class Gaussians:

    def __init__(
        self, init_type: str, device: str, load_path = None,
        num_points = None, isotropic = None
    ):

        self.device = device
        if self.device not in ("cpu", "cuda"):
            raise ValueError(f"Unsupported device: {self.device}")

            
        # Initialize from sparse reconstructed points
        if init_type == "points":
            if isotropic is not None and type(isotropic) is not bool:
                raise TypeError("isotropic must be either None or True or False.")
            if load_path is None:
                raise ValueError

            if isotropic is None:
                self.is_isotropic = False
            else:
                self.is_isotropic = isotropic

            data = self.load_points(load_path)
        
        # Initialize from random points
        elif init_type == "random":
            if isotropic is not None and type(isotropic) is not bool:
                raise TypeError("isotropic must be either None or True or False.")
            if num_points is None:
                raise ValueError

            if isotropic is None:
                self.is_isotropic = False
            else:
                self.is_isotropic = isotropic

            data = self.load_random(num_points)

        else:
            raise ValueError(f"Invalid init_type: {init_type}")

        self.quats = data["quats"]
        self.means = data["means"]
        self.scales = data["scales"]
        self.colors = data["colors"]
        self.opacities = data["opacities"]

        if self.device == "cuda":
            self.to_cuda()

    def __len__(self):
        return len(self.means)
    
    def train(self):
        if not self.is_isotropic:
            self.quats.requires_grad_(True)
        self.means.requires_grad_(True)
        self.scales.requires_grad_(True)
        self.colors.requires_grad_(True)
        self.opacities.requires_grad_(True)

    
    def load_gaussians(self, ply_path: str):

        data = dict()
        ply_gaussians = load_gaussians_from_ply(ply_path)

        data["means"] = torch.tensor(ply_gaussians["xyz"])
        data["quats"] = torch.tensor(ply_gaussians["rot"])
        data["scales"] = torch.tensor(ply_gaussians["scale"])
        data["opacities"] = torch.tensor(ply_gaussians["opacity"]).squeeze()
        data["colors"] = torch.tensor(ply_gaussians["dc_colours"])

        if data["scales"].shape[1] != 3:
            raise NotImplementedError("Currently does not support isotropic")

        is_isotropic = False

        return data, is_isotropic

    def load_points(self, path):

        data = dict()

        from plyfile import PlyData
        plydata = PlyData.read(path)
        
        means = np.stack([
            plydata['vertex']['x'],
            plydata['vertex']['y'],
            plydata['vertex']['z']
        ], axis=1)
        
        # Normalize colors to [0, 1] if they exist
        features_rgb = np.stack([
            plydata['vertex']['red'],
            plydata['vertex']['green'],
            plydata['vertex']['blue']
        ], axis=1).astype(np.float32) / 255.0
    
        means = torch.tensor(means.astype(np.float32))
        init_dist = torch.ones(means.shape[0]) * 0.003
        
        # Initializing means using the provided point cloud
        data["means"] = means  # (N, 3)
        print('Number of points load:', means.shape[0])

        # Initializing opacities such that all when sigmoid is applied to opacities,
        # we will have a opacity value close to (but less than) 1.0
        data["opacities"] = 10.0 * torch.ones((len(means),), dtype=torch.float32)  # (N,)

        # Initializing colors randomly
        data["colors"] = torch.logit(torch.tensor(features_rgb.astype(np.float32)))#2.0 * torch.rand((len(means), 3), dtype=torch.float32)-1  # (N, 3)

        # Initializing quaternions to be the identity quaternion
        quats = torch.zeros((len(means), 4), dtype=torch.float32)  # (N, 4)
        quats[:, 0] = 1.0
        data["quats"] = quats  # (N, 4)

        # Initializing scales using the mean distance of each point to its 9 nearest points
        init_dist = torch.clamp(init_dist, max=0.02, min=0.002)
        
        data["scales"] = torch.log(init_dist).unsqueeze(1)  # (N, 1)

        if not self.is_isotropic:
            data["scales"] = data["scales"].repeat(1, 3)  # (N, 3)

        return data

    def load_random(self, num_points: int):

        data = dict()

        # Initializing means randomly
        data["means"] = torch.randn((num_points, 3)).to(torch.float32) * 0.5  # (N, 3)

        # Initializing opacities such that all when sigmoid is applied to opacities,
        # we will have a opacity value close to (but less than) 1.0
        data["opacities"] = 10.0 * torch.ones((num_points,), dtype=torch.float32)  # (N,)

        # Initializing colors randomly
        data["colors"] = 2.0 *torch.rand((num_points, 3), dtype=torch.float32)-1  # (N, 3)

        # Initializing quaternions to be the identity quaternion
        quats = torch.zeros((num_points, 4), dtype=torch.float32)  # (N, 4)
        quats[:, 0] = 1.0
        data["quats"] = quats  # (N, 4)

        # Initializing scales randomly
        data["scales"] = torch.log((torch.rand((num_points, 1), dtype=torch.float32) + 1e-6) * 0.01)

        if not self.is_isotropic:
            data["scales"] = data["scales"].repeat(1, 3)  # (N, 3)

        return data
    
    def check_if_trainable(self):

        attrs = ["means", "scales", "colors", "opacities"]
        if not self.is_isotropic:
            attrs += ["quats"]

        for attr in attrs:
            param = getattr(self, attr)
            if not getattr(param, "requires_grad", False):
                return False

        if self.is_isotropic and self.quats.requires_grad:
            raise RuntimeError("You do not need to optimize quaternions in isotropic mode.")
        return True
    
    def to_cuda(self):

        self.quats = self.quats.cuda()
        self.means = self.means.cuda()
        self.scales = self.scales.cuda()
        self.colors = self.colors.cuda()
        self.opacities = self.opacities.cuda()

    def compute_cov_3D(self, quats: torch.Tensor, scales: torch.Tensor):
        """
        Computes the covariance matrices of 3D Gaussians from rotations and scale factor
        You can refer to equation (2) in the instruction file.
        Args:
            quats   :   A torch.Tensor of shape (N, 4) representing the rotation
                        components of 3D Gaussians in quaternion form.
            scales  :   If self.is_isotropic is True, scales is will be a torch.Tensor of shape (N, 1)
                        If self.is_isotropic is False, scales is will be a torch.Tensor of shape (N, 3).
                        Represents the scaling components of the 3D Gaussians.
        Returns:
            cov_3D  :   A torch.Tensor of shape (N, 3, 3)
        """
        # Hint: use function 'quaternion_to_matrix' to get rotation matrix from quaternion
        ### YOUR CODE STARTS HERE ###
        R = quaternion_to_matrix(quats)
        # scales are already activated in render() via apply_activations
        S_diag = scales
        if self.is_isotropic:
            S = S_diag.unsqueeze(-1) * torch.eye(3, device=scales.device).unsqueeze(0)
        else:
            S = torch.diag_embed(S_diag)
        cov_3D = R @ S @ S.transpose(-1, -2) @ R.transpose(-1, -2)
        ### YOUR CODE ENDS HERE ###

        return cov_3D.squeeze()
    
    def compute_jacobian(self, means_3D: torch.Tensor, camera: BaseCamera, img_size: Tuple):
        """
        Computes the jacobian matrix that approximates the camera projection.
        You can refer to equation (6) in the instruction file.
        Args:
            means_3D    :   A torch.Tensor of shape (N, 3) representing the means of
                            3D Gaussians in world coordinate.
            camera  :       A BaseCamera object, you can find it and its functions in camera.py
            image_size  :   A tuple of width, height
        Returns:
            J  :   A Jacobian matrix (N, 2, 3)
        """
        fx, fy = camera.fx, camera.fy
        W, H = img_size

        half_tan_fov_x = 0.5 * W / fx
        half_tan_fov_y = 0.5 * H / fy

        means_view_space = camera.transform_points(means_3D)

        tx = means_view_space[:, 0]
        ty = means_view_space[:, 1]
        tz = means_view_space[:, 2]

        tz2 = tz*tz

        lim_x = 1.3 * half_tan_fov_x
        lim_y = 1.3 * half_tan_fov_y

        tx = torch.clamp(tx/tz, -lim_x, lim_x) * tz
        ty = torch.clamp(ty/tz, -lim_y, lim_y) * tz

        ### YOUR CODE STARTS HERE ###
        eps = 1e-7
        z_safe = tz + eps
        z2_safe = tz2 + eps
        J = torch.zeros((tx.shape[0], 2, 3), device=tx.device, dtype=tx.dtype)
        J[:, 0, 0] = fx / z_safe
        J[:, 0, 2] = -fx * tx / z2_safe
        J[:, 1, 1] = fy / z_safe
        J[:, 1, 2] = -fy * ty / z2_safe
        ### YOUR CODE ENDS HERE ###

        J = J.to(self.device)
        return J  # (N, 2, 3)

    def compute_cov_2D(
        self, means_3D: torch.Tensor, quats: torch.Tensor, scales: torch.Tensor,
        camera: BaseCamera, img_size: Tuple
    ):
        """
        Computes the covariance matrices of 2D Gaussians using equation (5) in the instruction.
        Args:
            means_3D    :   A torch.Tensor of shape (N, 3) representing the means of
                            3D Gaussians in world coordinate.
            quats       :   A torch.Tensor of shape (N, 4) representing the rotation
                            components of 3D Gaussians in quaternion form.
            scales      :   If self.is_isotropic is True, scales is will be a torch.Tensor of shape (N, 1)
                            If self.is_isotropic is False, scales is will be a torch.Tensor of shape (N, 3)
            camera      :   A BaseCamera object, you can find the definition in camera.py
            img_size    :   A tuple representing the (width, height) of the image
        Returns:
            cov_2D  :   A torch.Tensor of shape (N, 2, 2)
        """
        ### YOUR CODE STARTS HERE ###
        N = means_3D.shape[0]
        J = self.compute_jacobian(means_3D, camera, img_size)
        W = camera.R.unsqueeze(0).expand(N, -1, -1)
        cov_3D = self.compute_cov_3D(quats, scales)
        cov_2D = J @ W @ cov_3D @ W.transpose(-1, -2) @ J.transpose(-1, -2)
        ### YOUR CODE ENDS HERE ###


        return cov_2D

    @staticmethod
    def compute_means_projection(means_3D: torch.Tensor, camera: BaseCamera):
        """
        Computes the means of the projected 2D Gaussians given the means of the 3D Gaussians. See eq (2), (3).
        Args:
            means_3D    :   A torch.Tensor of shape (N, 3) representing the means of
                            3D Gaussians in world coordinate.
            camera      :   A BaseCamera object, you can find the definition in camera.py
        Returns:
            means_2D    :   A torch.Tensor of shape (N, 2) representing the means of
                            2D Gaussians.
            depths      :   A torch.Tensor of shape (N, ) representing the depth of each gaussian.
        """
        ### YOUR CODE STARTS HERE ###

        points_camera = camera.transform_points(means_3D)
        points_homogeneous = points_camera @ camera.K.T
        z = points_homogeneous[:, 2:3] + 1e-7
        means_2D = points_homogeneous[:, :2] / z
        depths = points_homogeneous[:, 2]

        ### YOUR CODE ENDS HERE ###


        return means_2D, depths

    @staticmethod
    def compute_gaussian_2D(points_2D: torch.Tensor, means_2D: torch.Tensor, cov_2D_inverse: torch.Tensor):
        """
        Computes the weight of 2D Gaussians. See equation (7) in the instruction file.
        Args:
            points_2D       :   A torch.Tensor of shape (1, H*W, 2) containing the x, y points
                                corresponding to every pixel in an image. See function
                                compute_alphas in the class Scene to get more information
                                about how points_2D is created.
            means_2D        :   A torch.Tensor of shape (N, 1, 2) representing the means of
                                N 2D Gaussians.
            cov_2D_inverse  :   A torch.Tensor of shape (N, 2, 2) representing the
                                inverse of the covariance matrices of N 2D Gaussians.
        Returns:
            weights           :   A torch.Tensor of shape (N, H*W) representing the computed
                                weights of the N 2D Gaussians at every pixel location in an image.
        """

        ### YOUR CODE STARTS HERE ###
        diff = points_2D - means_2D
        tmp = torch.einsum('nij,npj->npi', cov_2D_inverse, diff)
        power = -0.5 * (diff * tmp).sum(dim=-1)
        ### YOUR CODE ENDS HERE ###


        # Computing weights = exp(power) with some post processing for numerical stability
        weights = torch.where(power > 0.0, 0.0, torch.exp(power))

        return weights

    @staticmethod
    def apply_activations(quats, scales, opacities, colors):

        # Convert logscales to scales
        scales = torch.exp(scales)

        # Normalize quaternions
        quats = torch.nn.functional.normalize(quats)

        # Bound opacities between (0, 1)
        opacities = torch.sigmoid(opacities)
        colors = torch.sigmoid(colors)

        return quats, scales, opacities, colors
    
    def replace_tensor_to_optimizer(self, optimizer, tensor, name):
        optimizable_tensors = {}
        for group in optimizer.param_groups:
            if group["name"] == name:
                stored_state = optimizer.state.get(group['params'][0], None)
                stored_state["exp_avg"] = torch.zeros_like(tensor)
                stored_state["exp_avg_sq"] = torch.zeros_like(tensor)

                del optimizer.state[group['params'][0]]
                group["params"][0] = nn.Parameter(tensor.requires_grad_(True))
                optimizer.state[group['params'][0]] = stored_state

                optimizable_tensors[group["name"]] = group["params"][0]
        return optimizable_tensors

    def reset_opacity(self, optimizer):
        def inverse_sigmoid(x):
            return torch.log(x/(1-x))
        orig_opacity = torch.sigmoid(self.opacities)
        opacities_new = inverse_sigmoid(torch.min(orig_opacity, torch.ones_like(orig_opacity)*0.01))
        optimizable_tensors = self.replace_tensor_to_optimizer(optimizer, opacities_new, "opacities")
        self.opacities = optimizable_tensors["opacities"]

    def get_num_points(self):
        return self.means.shape[0]