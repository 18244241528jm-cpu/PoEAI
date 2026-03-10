import torch
class BaseCamera:
    def __init__(self, R, T, focal_length, principal_point, image_size, device="cuda"):
        """
        Initializes a pinhole camera using COLMAP/OpenCV conventions.
        
        Args:
            R: 3x3 rotation matrix (list, numpy array, or tensor)
            T: 3x1 translation vector (list, numpy array, or tensor)
            focal_length: Tuple/list of (fx, fy)
            principal_point: Tuple/list of (cx, cy)
            image_size: Tuple/list of (width, height)
            device: string ('cuda' or 'cpu')
        """
        self.device = device
        
        # Extrinsics - ensure they are float32 tensors on the correct device
        self.R = torch.as_tensor(R, dtype=torch.float32).to(device).reshape(3, 3)
        self.T = torch.as_tensor(T, dtype=torch.float32).to(device).view(3)
        
        # Intrinsics
        self.fx = float(focal_length[0])
        self.fy = float(focal_length[1])
        self.cx = float(principal_point[0])
        self.cy = float(principal_point[1])
        # Create Intrinsic Matrix K
        self.K = torch.tensor([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0]
        ], dtype=torch.float32, device=device)
        
        # Image dimensions
        self.width = int(image_size[0])
        self.height = int(image_size[1])

    def transform_points(self, points_3D):
        """
        Transforms 3D points from World space to Camera space.
        
        Args:
            points_3D: Tensor of shape [N, 3] representing world coordinates.
            
        Returns:
            points_camera: Tensor of shape [N, 3] in camera coordinates.
        """

        points_camera = points_3D @ self.R.T + self.T
        return points_camera

    def project_2D(self, points_3D):
        """
        Projects 3D world points onto the 2D image plane.
        
        Args:
            points_3D: Tensor of shape [N, 3] representing world coordinates.
            
        Returns:
            uv_coordinates: Tensor of shape [N, 2] representing pixel coordinates (u, v).
            z_depth: Tensor of shape [N] representing the depth (Z) in camera space.
        """
        # 1. Transform to camera space
        # 1. Transform to camera space [N, 3]
        points_camera = self.transform_points(points_3D)
        points_homogeneous = points_camera @ self.K.T

        uv_hom = points_homogeneous[:, :2]  # Shape: [N, 2]
        z = points_homogeneous[:, 2:]       # Shape: [N, 1]
        
        z_safe = z + 1e-7
        uv_coordinates = uv_hom / z_safe

        return uv_coordinates, z.squeeze(-1)
    
    def cpu(self):
        """Moves camera tensors to CPU memory."""
        self.device = "cpu"
        self.R = self.R.cpu()
        self.T = self.T.cpu()
        self.K = self.K.cpu()
        return self

    def cuda(self):
        """Moves camera tensors to GPU memory."""
        self.device = "cuda"
        self.R = self.R.cuda()
        self.T = self.T.cuda()
        self.K = self.K.cuda()
        return self