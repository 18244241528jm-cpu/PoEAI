
import os
import torch
import imageio
import numpy as np
import matplotlib.pyplot as plt
import json
import torch.nn.functional as F

from PIL import Image
from plyfile import PlyData
from torch.utils.data import Dataset
from camera import BaseCamera

SH_C0 = 0.28209479177387814
CMAP_JET = plt.get_cmap("jet")
CMAP_MIN_NORM, CMAP_MAX_NORM = 5.0, 7.0

def quaternion_to_matrix(quaternions: torch.Tensor) -> torch.Tensor:
    """
    Converts a tensor of quaternions into rotation matrices.
    
    Args:
        quaternions: Tensor of shape (..., 4) formatted as (w, x, y, z).
                     'w' is the real (scalar) part.
                     
    Returns:
        Rotation matrices of shape (..., 3, 3).
    """
    # 1. Normalization (Crucial for 3DGS)
    # Unnormalized quaternions during optimization will distort the rotation matrix.
    q = F.normalize(quaternions, p=2, dim=-1)
    
    # 2. Unbind the quaternion components for the math operations
    w, x, y, z = torch.unbind(q, dim=-1)
    
    # 3. Compute the individual elements of the 3x3 rotation matrix
    # Row 0
    r00 = 1.0 - 2.0 * (y**2 + z**2)
    r01 = 2.0 * (x * y - w * z)
    r02 = 2.0 * (x * z + w * y)
    
    # Row 1
    r10 = 2.0 * (x * y + w * z)
    r11 = 1.0 - 2.0 * (x**2 + z**2)
    r12 = 2.0 * (y * z - w * x)
    
    # Row 2
    r20 = 2.0 * (x * z - w * y)
    r21 = 2.0 * (y * z + w * x)
    r22 = 1.0 - 2.0 * (x**2 + y**2)
    
    # 4. Stack them into the final (..., 3, 3) tensor
    matrix = torch.stack(
        [
            torch.stack([r00, r01, r02], dim=-1),
            torch.stack([r10, r11, r12], dim=-1),
            torch.stack([r20, r21, r22], dim=-1),
        ],
        dim=-2,
    )
    
    return matrix


class GSDataset(Dataset):
    def __init__(self, root, split, image_size=None, ply_path=None):
        super().__init__()
        self.root = root
        self.split = split
        self.target_size = image_size  # New parameter: int
        self.load_image = True

        if self.split == 'train':
            json_path = os.path.join(root, "transforms_train.json")
        elif self.split == 'test':
            json_path = os.path.join(root, "transforms_test.json")
            self.load_image = False
        else:
            json_path = os.path.join(root, "transforms_val.json")
            
        with open(json_path, 'r') as f:
            self.meta = json.load(f)

        self.images = []
        self.cameras = []
        self.masks = []

        self.points_path = ply_path

        all_frames = self.meta['frames']
        num_files = len(all_frames)
        
        idxs = range(num_files)
        fov_x = self.meta['camera_angle_x']

        for i in idxs:
            frame = all_frames[i]
            
            # --- 1. Image Loading or Dimension Fallback ---
            if self.load_image:
                img_path = os.path.join(root, frame['file_path'])
                if not os.path.exists(img_path):
                    img_path += ".png"

                img_raw = imageio.imread(img_path).astype(np.float32) / 255.0
                img_tensor = torch.from_numpy(img_raw).permute(2, 0, 1) # [C, H, W]
                
                orig_h, orig_w = img_tensor.shape[1:3]

                # --- Resizing Logic ---
                if self.target_size is not None:
                    img_tensor = F.interpolate(
                        img_tensor.unsqueeze(0), 
                        size=(self.target_size, self.target_size), 
                        mode='bilinear', 
                        align_corners=False,
                        antialias=True
                    ).squeeze(0)
                    
                    new_h, new_w = self.target_size, self.target_size
                    scale_w = new_w / orig_w
                    scale_h = new_h / orig_h
                else:
                    new_h, new_w = orig_h, orig_w
                    scale_w = scale_h = 1.0

                # Separate Image and Mask
                if img_tensor.shape[0] == 4:
                    img = img_tensor[:3, ...].permute(1, 2, 0) # [H, W, 3]
                    mask = img_tensor[3:4, ...].permute(1, 2, 0) # [H, W, 1]
                    self.masks.append(mask)
                else:
                    img = img_tensor.permute(1, 2, 0)
                
                self.images.append(img)
                
            else:
                # If not loading images, fetch dimensions from JSON or fallback to 800
                orig_w = int(frame.get('w', self.meta.get('w', 800)))
                orig_h = int(frame.get('h', self.meta.get('h', 800)))
                
                if self.target_size is not None:
                    new_h, new_w = self.target_size, self.target_size
                    scale_w = new_w / orig_w
                    scale_h = new_h / orig_h
                else:
                    new_h, new_w = orig_h, orig_w
                    scale_w = scale_h = 1.0
                
                # Append a placeholder to keep list indices aligned
                self.images.append(None)
            
            # --- Updated Intrinsics ---
            focal_orig = 0.5 * orig_w / np.tan(0.5 * fov_x)
            
            focal_new = focal_orig * scale_w
            px_new = (orig_w / 2.0) * scale_w
            py_new = (orig_h / 2.0) * scale_h

            # --- Extrinsics ---
            c2w = np.array(frame['transform_matrix'])
            flip_mat = np.diag([1, -1, -1, 1])
            c2w = c2w @ flip_mat
            
            w2c = np.linalg.inv(c2w)
            
            R = torch.tensor(w2c[:3, :3])
            T = torch.tensor(w2c[:3, 3])

            device = "cuda" if torch.cuda.is_available() else "cpu"
            camera = BaseCamera(
                focal_length=[focal_new, focal_new],
                principal_point=[px_new, py_new],
                R=R[None].to(torch.float), 
                T=T[None].to(torch.float),
                image_size=(new_h, new_w),
                device=device
            )

            self.cameras.append(camera)

        self.img_size = (self.target_size, self.target_size) if self.target_size else (800, 800)

    def __len__(self):
        # Tie length to cameras instead of images, since images might just be placeholders
        return len(self.cameras)

    def __getitem__(self, idx):
        if self.load_image:
            img = self.images[idx]
            mask = self.masks[idx] if len(self.masks) > 0 else torch.ones_like(img[..., :1])
        else:
            # Generate dummy tensors dynamically to save RAM and keep collate_fn happy
            h, w = self.cameras[idx].width, self.cameras[idx].height
            img = torch.zeros((h, w, 3), dtype=torch.float32)
            mask = torch.ones((h, w, 1), dtype=torch.float32)
            
        return img, self.cameras[idx], mask

    @staticmethod
    def collate_fn(batch):
        images = torch.stack([x[0] for x in batch], dim=0)
        cameras = [x[1] for x in batch] 
        masks = torch.stack([x[2] for x in batch], dim=0)
        return images, cameras, masks


def colour_depth_q1_render(depth):
    normalized_depth = (depth - CMAP_MIN_NORM) / (CMAP_MAX_NORM - CMAP_MIN_NORM + 1e-8)
    coloured_depth = CMAP_JET(normalized_depth)[:, :, :3]  # (H, W, 3)
    coloured_depth = (np.clip(coloured_depth, 0.0, 1.0) * 255.0).astype(np.uint8)

    return coloured_depth

def visualize_renders(scene, gt_viz_img, cameras, img_size):

    imgs = []
    viz_size = (256, 256)
    with torch.no_grad():
        for cam in cameras:
            pred_img, _, _ = scene.render(
                cam, img_size=img_size,
                bg_colour=(0.0, 0.0, 0.0),
                per_splat=-1,
            )
            img = torch.clamp(pred_img, 0.0, 1.0) * 255.0
            img = img.clone().detach().cpu().numpy().astype(np.uint8)

            if img_size[0] != viz_size[0] or img_size[1] != viz_size[1]:
                img = np.array(Image.fromarray(img).resize(viz_size))

            imgs.append(img)

    pred_viz_img = np.concatenate(imgs, axis=1)
    viz_frame = np.concatenate((pred_viz_img, gt_viz_img), axis=0)
    return viz_frame

def save_numpy_to_png(pred_npy, output_path):
    pred_clipped = np.clip(pred_npy, 0.0, 1.0)
    
    # 2. Scale from [0, 1] to [0, 255] and convert to 8-bit unsigned integer
    pred_uint8 = (pred_clipped * 255.0).astype(np.uint8)

    img = Image.fromarray(pred_uint8)
    img.save(output_path)
    print(f"Saved image successfully to {output_path}")

def load_gaussians_from_ply(path):
    # Modified from https://github.com/thomasantony/splat
    max_sh_degree = 3
    plydata = PlyData.read(path)
    xyz = np.stack((np.asarray(plydata.elements[0]["x"]),
                    np.asarray(plydata.elements[0]["y"]),
                    np.asarray(plydata.elements[0]["z"])),  axis=1)
    opacities = np.asarray(plydata.elements[0]["opacity"])[..., np.newaxis]

    features_dc = np.zeros((xyz.shape[0], 3, 1))
    features_dc[:, 0, 0] = np.asarray(plydata.elements[0]["f_dc_0"])
    features_dc[:, 1, 0] = np.asarray(plydata.elements[0]["f_dc_1"])
    features_dc[:, 2, 0] = np.asarray(plydata.elements[0]["f_dc_2"])

    extra_f_names = [p.name for p in plydata.elements[0].properties if p.name.startswith("f_rest_")]
    extra_f_names = sorted(extra_f_names, key = lambda x: int(x.split('_')[-1]))
    assert len(extra_f_names) == 3 * (max_sh_degree + 1) ** 2 - 3
    features_extra = np.zeros((xyz.shape[0], len(extra_f_names)))
    for idx, attr_name in enumerate(extra_f_names):
        features_extra[:, idx] = np.asarray(plydata.elements[0][attr_name])
    # Reshape (P,F*SH_coeffs) to (P, F, SH_coeffs except DC)
    features_extra = features_extra.reshape((features_extra.shape[0], 3, (max_sh_degree + 1) ** 2 - 1))
    features_extra = np.transpose(features_extra, [0, 2, 1])

    scale_names = [p.name for p in plydata.elements[0].properties if p.name.startswith("scale_")]
    scale_names = sorted(scale_names, key = lambda x: int(x.split('_')[-1]))
    scales = np.zeros((xyz.shape[0], len(scale_names)))
    for idx, attr_name in enumerate(scale_names):
        scales[:, idx] = np.asarray(plydata.elements[0][attr_name])

    rot_names = [p.name for p in plydata.elements[0].properties if p.name.startswith("rot")]
    rot_names = sorted(rot_names, key = lambda x: int(x.split('_')[-1]))
    rots = np.zeros((xyz.shape[0], len(rot_names)))
    for idx, attr_name in enumerate(rot_names):
        rots[:, idx] = np.asarray(plydata.elements[0][attr_name])

    xyz = xyz.astype(np.float32)
    rots = rots.astype(np.float32)
    scales = scales.astype(np.float32)
    opacities = opacities.astype(np.float32)
    shs = np.concatenate([
        features_dc.reshape(-1, 3),
        features_extra.reshape(len(features_dc), -1)
    ], axis=-1).astype(np.float32)
    shs = shs.astype(np.float32)

    dc_vals = shs[:, :3]
    dc_colours = np.maximum(dc_vals * SH_C0 + 0.5, np.zeros_like(dc_vals))

    output = {
        "xyz": xyz, "rot": rots, "scale": scales,
        "sh": shs, "opacity": opacities, "dc_colours": dc_colours
    }
    return output

def update_optimizer_state(optimizer, new_params_dict, keep_mask=None, new_indices=None):
    """
    Updates the optimizer state (momentum buffers) when the number of Gaussians changes.
    Robustly handles multiple parameter groups (xyz, opacity, rotation, etc.).
    
    Args:
        optimizer: The PyTorch optimizer.
        new_params_dict: Dict mapping group_name -> new_nn_parameter.
                         Keys should match the 'name' you gave groups in setup_optimizer 
                         (e.g., 'xyz', 'opacity', 'scaling', 'rotation', 'f_dc').
        keep_mask: (For pruning) Boolean mask of points to keep.
        new_indices: (For densification) Indices of original points to clone/split.
    """
    
    # Iterate over ALL groups (xyz, opacity, features, etc.)
    for group in optimizer.param_groups:
        name = group.get("name", None)
        
        # Only update if this group is in our new params dictionary
        if name is not None and name in new_params_dict:
            old_param = group["params"][0]
            new_param = new_params_dict[name]
            
            # 1. Retrieve the existing optimizer state for this parameter
            # Use .get() to return None if state doesn't exist yet (e.g. iter 0)
            state = optimizer.state.get(old_param, None)
            
            if state is not None:
                # 2. Modify the state (Momentum Buffers)
                # We typically update 'exp_avg' and 'exp_avg_sq'
                for key in ['exp_avg', 'exp_avg_sq']:
                    if key in state:
                        if keep_mask is not None:
                            # --- PRUNING ---
                            # Keep only the survivors
                            # Ensure mask matches state (handle optimizer/gaussian count mismatch)
                            n_state = state[key].shape[0]
                            n_mask = keep_mask.shape[0]
                            if n_mask == n_state:
                                state[key] = state[key][keep_mask]
                            elif n_mask < n_state:
                                # State has more (e.g. from failed sync) - pad mask with False to prune extra
                                keep_aligned = torch.zeros(n_state, dtype=torch.bool, device=keep_mask.device)
                                keep_aligned[:n_mask] = keep_mask
                                state[key] = state[key][keep_aligned]
                            else:
                                # Mask has more - truncate
                                state[key] = state[key][keep_mask[:n_state]]
                            
                        elif new_indices is not None:
                            # --- DENSIFICATION ---
                            # Concatenate the new momentum initialized from parents
                            new_momentum = state[key][new_indices]
                            state[key] = torch.cat([state[key], new_momentum], dim=0)

                # 3. Clean up the old parameter key
                del optimizer.state[old_param]
                
                # 4. Assign the MODIFIED state to the NEW parameter
                optimizer.state[new_param] = state
            
            else:
                # If state was None (optimizer hasn't stepped yet), just initialize empty
                # The optimizer will create the state on the next .step() automatically
                pass

            # 5. Update the Parameter in the Group
            group["params"][0] = new_param

    return optimizer

def colour_depth_q1_render(depth):
    normalized_depth = (depth - CMAP_MIN_NORM) / (CMAP_MAX_NORM - CMAP_MIN_NORM + 1e-8)
    coloured_depth = CMAP_JET(normalized_depth)[:, :, :3]  # (H, W, 3)
    coloured_depth = (np.clip(coloured_depth, 0.0, 1.0) * 255.0).astype(np.uint8)

    return coloured_depth