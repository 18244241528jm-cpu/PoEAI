from gaussians import Gaussians
import torch
from typing import Tuple, Optional
from camera import BaseCamera
import math
from utils import update_optimizer_state

class Renderer():

    def __init__(self, gaussians: Gaussians, optimizer, extent=1.0, percent_dense=0.01, grad_threshold=0.0002, device = 'cuda'):
        self.gaussians = gaussians
        self.device = self.gaussians.device
        self.optimizer = optimizer
        self.scene_extent = extent
        self.percent_dense = percent_dense
        self.grad_threshold = grad_threshold
        
        # Accumulator for View-Space Gradients (2D)
        self.xyz_gradient_accum = torch.zeros((self.gaussians.get_num_points(), 1), device=device)
        self.denom = torch.zeros((self.gaussians.get_num_points(), 1), device=device)

    # 2. Depth sorting: correct order rendering and early stopping
    def depth_sorting(self, z_vals: torch.Tensor):
        """
        Given depth values of Gaussians, return the indices to depth-wise sort
        Gaussians and at the same time remove invalid Gaussians.

        You can see the function render to see how the returned indices will be used.
        You are required to create a torch.Tensor idxs such that by using them in the
        function render we can arrange Gaussians (or equivalently their attributes such as
        the mean) in ascending order of depth. You should also make sure to not include indices
        that correspond to Gaussians with depth value less than or equal to 0.

        Args:
            z_vals  :   the depths of gaussians in shape (N,)

        Returns:
            idxs    :   idxs should be torch.Tensor of dtype int64 with length N (N <= M, where M is the
        total number of Gaussians before filtering)
        """
        ### YOUR CODE STARTS HERE ###
        valid = z_vals > 0
        idxs = torch.where(valid)[0]
        z_valid = z_vals[idxs]
        _, sort_order = torch.sort(z_valid)
        idxs = idxs[sort_order]
        ### YOUR CODE ENDS HERE ###


        return idxs
    
    def compute_alphas(self, opacities, means_2D, cov_2D, img_size):
        """
        Given some parameters of N ordered Gaussians, this function computes
        the alpha values. See equaltion (9) for more details.

        Args:
            opacities   :   A torch.Tensor of shape (N,) with the opacity value
                            of each Gaussian.
            means_2D    :   A torch.Tensor of shape (N, 2) with the means
                            of the 2D Gaussians.
            cov_2D      :   A torch.Tensor of shape (N, 2, 2) with the covariances
                            of the 2D Gaussians.
            img_size    :   The (width, height) of the image to be rendered.


        Returns:
            alphas      :   A torch.Tensor of shape (N, H, W) with the computed alpha
                            values for each of the N ordered Gaussians at every
                            pixel location.
        """
        W, H = img_size

        # point_2D contains all possible pixel locations in an image
        xs, ys = torch.meshgrid(torch.arange(W), torch.arange(H), indexing="xy")
        points_2D = torch.stack((xs.flatten(), ys.flatten()), dim = 1)  # (H*W, 2)
        points_2D = points_2D.to(self.device)

        points_2D = points_2D.unsqueeze(0)  # (1, H*W, 2)
        means_2D = means_2D.unsqueeze(1)  # (N, 1, 2)

        ### YOUR CODE STARTS HERE ###
        cov_2D_inverse = torch.linalg.inv(cov_2D)
        exp_power = Gaussians.compute_gaussian_2D(points_2D, means_2D, cov_2D_inverse)
        alphas = opacities.unsqueeze(1) * exp_power
        ### YOUR CODE ENDS HERE ###

        alphas = torch.reshape(alphas, (-1, H, W))  # (N, H, W)

        # Post processing for numerical stability
        alphas = torch.minimum(alphas, torch.full_like(alphas, 0.99))
        alphas = torch.where(alphas < 1/255.0, 0.0, alphas)

        return alphas


    def compute_transmittance(
        self, alphas: torch.Tensor,
        start_transmittance: Optional[torch.Tensor] = None
    ):
        """
        Given the alpha values of N ordered Gaussians, this function computes
        the transmittance. See eq (10).

        The variable start_transmittance contains information about the transmittance
        at each pixel location BEFORE encountering the first Gaussian in the input.
        This variable is useful when computing transmittance in mini-batches because
        we would require information about the transmittance accumulated until the
        previous mini-batch to begin computing the transmittance for the current mini-batch.

        In case there were no previous mini-batches (or we are splatting in one-shot
        without using mini-batches), then start_transmittance will be None (since no Gaussians
        have been encountered so far). In this case, the code will use a starting
        transmittance value of 1.

        Args:
            alphas                  :   A torch.Tensor of shape (N, H, W) with the computed alpha
                                        values for each of the N ordered Gaussians at every
                                        pixel location.
            start_transmittance     :   Can be None or a torch.Tensor of shape (1, H, W). Please
                                        see the docstring for more information.

        Returns:
            transmittance           :   A torch.Tensor of shape (N, H, W) with the computed transmittance
                                        values for each of the N ordered Gaussians at every
                                        pixel location.
        """
        _, H, W = alphas.shape

        if start_transmittance is None:
            S = torch.ones((1, H, W), device=alphas.device, dtype=alphas.dtype)
        else:
            S = start_transmittance

        one_minus_alphas = 1.0 - alphas
        one_minus_alphas = torch.concat((S, one_minus_alphas), dim=0)  # (N+1, H, W)

        ### YOUR CODE STARTS HERE ###
        transmittance = torch.cumprod(one_minus_alphas, dim=0)[:-1]
        ### YOUR CODE ENDS HERE ###


        # Post processing for numerical stability
        transmittance = torch.where(transmittance < 1e-4, 0.0, transmittance)  # (N, H, W)

        return transmittance

    def splat(
        self, camera: BaseCamera, means_3D: torch.tensor, z_vals: torch.Tensor,
        quats: torch.Tensor, scales: torch.Tensor, colors: torch.Tensor,
        opacities: torch.Tensor, img_size: Tuple = (256, 256),
        start_transmittance: Optional[torch.Tensor] = None,
    ):
        """
        Given N ordered (depth-sorted) 3D Gaussians (or equivalently in our case,
        the parameters of the 3D Gaussians like means, quats etc.), this function splats
        them to the image plane to render an RGB image, depth map and a silhouette map.

        Args:
            camera                  :   A BaseCamera object.
            means_3D                :   A torch.Tensor of shape (N, 3) with the means
                                        of the 3D Gaussians.
            z_vals                  :   A torch.Tensor of shape (N,) with the depths
                                        of the 3D Gaussians. # TODO: Verify shape
            quats                   :   A torch.Tensor of shape (N, 4) representing the rotation
                                        components of 3D Gaussians in quaternion form.
            scales                  :   A torch.Tensor of shape (N, 1) (if isotropic) or
                                        (N, 3) (if anisotropic) representing the scaling
                                        components of 3D Gaussians.
            colors                 :   A torch.Tensor of shape (N, 3) with the color contribution
                                        of each Gaussian.
            opacities               :   A torch.Tensor of shape (N,) with the opacity of each Gaussian.
            img_size                :   The (width, height) of the image.
            start_transmittance     :   Please see the docstring of the function compute_transmittance
                                        for information about this argument.

        Returns:
            image                   :   A torch.Tensor of shape (H, W, 3) with the rendered RGB color image.
            depth                   :   A torch.Tensor of shape (H, W, 1) with the rendered depth map.
            mask                    :   A torch.Tensor of shape (H, W, 1) with the rendered silhouette map.
            final_transmittance     :   A torch.Tensor of shape (1, H, W) representing the transmittance at
                                        each pixel computed using the N ordered Gaussians. This will be useful
                                        for mini-batch splatting in the next iteration.
        """
        ### YOUR CODE STARTS HERE ###
        means_2D, _ = self.gaussians.compute_means_projection(means_3D, camera)
        cov_2D = self.gaussians.compute_cov_2D(means_3D, quats, scales, camera, img_size)
        cov_2D[:, 0, 0] += 0.0005
        cov_2D[:, 1, 1] += 0.0005
        alphas = self.compute_alphas(opacities, means_2D, cov_2D, img_size)
        transmittance = self.compute_transmittance(alphas, start_transmittance)
        W, H = img_size
        ti_alpha = transmittance * alphas
        weighted_colors = ti_alpha.unsqueeze(-1) * colors.unsqueeze(1).unsqueeze(2)
        image = weighted_colors.sum(dim=0)
        mask = ti_alpha.sum(dim=0).unsqueeze(-1)
        ### YOUR CODE ENDS HERE ###

        final_transmittance = transmittance[-1, ...].unsqueeze(0)  # (1, H, W)

    
        return image, mask, final_transmittance

    def render(
        self, camera: BaseCamera,
        per_splat: int = -1, img_size: Tuple = (256, 256),
        bg_colour: Tuple = (0.0, 0.0, 0.0),
    ):
        """
        Given a scene represented by N 3D Gaussians, this function renders the RGB
        color image, the depth map and the silhouette map that can be observed
        from a given pytorch 3D camera.

        Args:
            camera      :   A BaseCamera object.
            per_splat   :   Number of gaussians to splat in one function call. If set to -1,
                            then all gaussians in the scene are splat in a single function call.
                            If set to any other positive interger, then it determines the number of
                            gaussians to splat per function call (the last function call might splat
                            lesser number of gaussians). In general, the algorithm can run faster
                            if more gaussians are splat per function call, but at the cost of higher GPU
                            memory consumption.
            img_size    :   The (width, height) of the image to be rendered.
            bg_color    :   A tuple indicating the RGB color that the background should have.

        Returns:
            image       :   A torch.Tensor of shape (H, W, 3) with the rendered RGB color image.
            depth       :   A torch.Tensor of shape (H, W, 1) with the rendered depth map.
            mask        :   A torch.Tensor of shape (H, W, 1) with the rendered silhouette map.
        """
        bg_colour_ = torch.tensor(bg_colour)[None, None, :]  # (1, 1, 3)
        bg_colour_ = bg_colour_.to(self.device)

        # Globally sort gaussians according to their depth value
        _, z_vals = self.gaussians.compute_means_projection(self.gaussians.means, camera)
        idxs = self.depth_sorting(z_vals)

        pre_act_quats = self.gaussians.quats[idxs]
        pre_act_scales = self.gaussians.scales[idxs]
        pre_act_opacities = self.gaussians.opacities[idxs]
        z_vals = z_vals[idxs]
        means_3D = self.gaussians.means[idxs]
        pre_act_colors = self.gaussians.colors[idxs]

        # Apply activations
        quats, scales, opacities, colors = self.gaussians.apply_activations(
            pre_act_quats, pre_act_scales, pre_act_opacities, pre_act_colors
        )

        if per_splat == -1:
            num_mini_batches = 1
        elif per_splat > 0:
            num_mini_batches = math.ceil(len(means_3D) / per_splat)
        else:
            raise ValueError("Invalid setting of per_splat")

        # In this case we can directly splat all gaussians onto the image
        if num_mini_batches == 1:

            # Get image, depth and mask via splatting
            image, mask, _ = self.splat(
                camera, means_3D, z_vals, quats, scales,
                colors, opacities, img_size
            )

        # In this case we splat per_splat number of gaussians per iteration. This makes
        # the implementation more memory efficient but at the same time makes it slower.
        else:

            W, H = img_size
            D = means_3D.device
            start_transmittance = torch.ones((1, H, W), dtype=torch.float32).to(D)
            image = torch.zeros((H, W, 3), dtype=torch.float32).to(D)
            mask = torch.zeros((H, W, 1), dtype=torch.float32).to(D)

            for b_idx in range(num_mini_batches):

                quats_ = quats[b_idx * per_splat: (b_idx+1) * per_splat]
                scales_ = scales[b_idx * per_splat: (b_idx+1) * per_splat]
                z_vals_ = z_vals[b_idx * per_splat: (b_idx+1) * per_splat]
                colours_ = colors[b_idx * per_splat: (b_idx+1) * per_splat]
                means_3D_ = means_3D[b_idx * per_splat: (b_idx+1) * per_splat]
                opacities_ = opacities[b_idx * per_splat: (b_idx+1) * per_splat]

                # Get image, depth and mask via splatting
                image_, mask_, start_transmittance = self.splat(
                    camera, means_3D_, z_vals_, quats_, scales_, colours_,
                    opacities_, img_size, start_transmittance
                )

                image = image + image_
                mask = mask + mask_

        image = image + (1.0 - mask) * bg_colour_

        if torch.is_grad_enabled():
            self.gaussians.means.retain_grad()


        return image, mask, self.gaussians.means
    
    def accumulate_grads(self, viewspace_grad):
        """ Call this after backward() to track 3D gradients """
        self.xyz_gradient_accum += torch.norm(viewspace_grad[:, :], dim=-1, keepdim=True)
        self.denom += 1

    def prune_and_densify(self, min_opacity=0.01, max_densify_ratio=None):
        max_densify_per_step = int(self.xyz_gradient_accum.shape[0] * max_densify_ratio)
        # 1. Average the accumulated gradients
        grads = self.xyz_gradient_accum / self.denom
        grads[self.denom == 0] = 0.0
        
        # --- IDENTIFY CANDIDATES ---
        # Points with high gradients (under-reconstructed areas)
        mask_grad = (grads >= self.grad_threshold).squeeze()
    
        # --- FILTER TOP K (Prioritize high gradients) ---
        num_candidates = mask_grad.sum()
        if num_candidates > max_densify_per_step:
            # Get the indices of the candidates
            candidate_indices = torch.where(mask_grad)[0]
            
            # Get the gradients for these candidates
            candidate_grads = grads[candidate_indices].squeeze()
            
            # Find the threshold value that cuts off the top K
            # 'kthvalue' is faster than full sort. We want the k-th largest, so we use negative logic 
            # or topk. Topk is often cleaner for direct indices.
            top_scores, top_indices = torch.topk(candidate_grads, k=max_densify_per_step)
            
            # Create a new, stricter mask
            mask_grad = torch.zeros_like(mask_grad)
            original_indices_to_keep = candidate_indices[top_indices]
            mask_grad[original_indices_to_keep] = True
        
        # Get scale to distinguish between "Clone" (small) and "Split" (large)
        # We take max scale dimension
        scales = torch.exp(self.gaussians.scales) # Assuming scales are stored as log
        max_scale = torch.max(scales, dim=1).values
        
        # Clone: High grad & Small scale
        mask_clone = mask_grad & (max_scale <= self.scene_extent * self.percent_dense)
        # Split: High grad & Large scale
        mask_split = mask_grad & (max_scale > self.scene_extent * self.percent_dense)

        if mask_clone.any() or mask_split.any():
            self._densify(mask_clone, mask_split)
            
        self._prune(min_opacity)
            
        # Reset accumulators
        self.xyz_gradient_accum.zero_()
        self.denom.zero_()
        torch.cuda.empty_cache()

    def _densify(self, mask_clone, mask_split, deterministic = False, device = 'cuda'):
        '''Given the mask_clone and mask_split derived from gradient map of 3D Gaussian means,
        Your task is to 1. Clone the data by making direct copy of the Gaussians at certain indices
        2. Split the gaussians by creating 2 copy of gaussians at certain indices and shrink the scale of new gaussians
        by 1.6. 
        Then concatenate the new gaussians with original ones.
        Args: 
            mask_clone  :   (N, ) binary mask inicating which gaussian should be cloned.
            mask_split  :   (N, ) binary mask inicating which gaussian should be split.
            deterministic:  Bool 
            device      :   String
        '''
        # Indices of parents
        idx_clone = torch.nonzero(mask_clone).squeeze(1)
        idx_split = torch.nonzero(mask_split).squeeze(1)

        ### YOUR CODE STARTS HERE ###
        new_means_c = self.gaussians.means[idx_clone]
        new_scales_c = self.gaussians.scales[idx_clone]
        new_quats_c = self.gaussians.quats[idx_clone]
        new_opacities_c = self.gaussians.opacities[idx_clone]
        new_colors_c = self.gaussians.colors[idx_clone]
        new_means_s = self.gaussians.means[idx_split]
        new_scales_s = self.gaussians.scales[idx_split] - math.log(1.6)
        new_quats_s = self.gaussians.quats[idx_split]
        new_opacities_s = self.gaussians.opacities[idx_split]
        new_colors_s = self.gaussians.colors[idx_split]
        ### YOUR CODE ENDS HERE ###
        
        # Perturb position slightly based on the new scale
        # (Simplified sampling)
        if not deterministic:
            stds = torch.exp(new_scales_s)
            new_means_s = new_means_s + (torch.randn_like(new_means_s) * stds)

        # --- COMBINE NEW DATA: combine the cloned and splitted gaussians with original ones ---
        cat_means = torch.cat([self.gaussians.means, new_means_c, new_means_s])
        cat_scales = torch.cat([self.gaussians.scales, new_scales_c, new_scales_s])
        cat_quats = torch.cat([self.gaussians.quats, new_quats_c, new_quats_s])

        cat_opacities = torch.cat([self.gaussians.opacities, new_opacities_c, new_opacities_s])
        cat_colors = torch.cat([self.gaussians.colors, new_colors_c, new_colors_s])

        # Indices used to extend optimizer states
        # Must match exactly the number of actually added Gaussians.
        # Here splits add one child per parent, so we use idx_split once.
        new_indices = torch.cat([idx_clone, idx_split])
        
        # Update Gaussians Class
        self.gaussians.means = torch.nn.Parameter(cat_means)
        self.gaussians.scales = torch.nn.Parameter(cat_scales)
        self.gaussians.quats = torch.nn.Parameter(cat_quats)
        self.gaussians.opacities = torch.nn.Parameter(cat_opacities)
        self.gaussians.colors = torch.nn.Parameter(cat_colors)
        
        # Update Optimizer
        new_params = {
            'means': self.gaussians.means, 'scales': self.gaussians.scales,
            'quats': self.gaussians.quats, 'opacities': self.gaussians.opacities,
            'colors': self.gaussians.colors
        }
        
        # Note: We append new params, so we pass indices of parents to clone momentum
        if self.optimizer is not None:
            self.optimizer = update_optimizer_state(self.optimizer, new_params, new_indices=new_indices)


        # Also update our own accumulators (append zeros for new points)
        num_new = new_means_c.shape[0] + new_means_s.shape[0]
        zeros = torch.zeros((num_new, 1), device=device)
        self.xyz_gradient_accum = torch.cat([self.xyz_gradient_accum, zeros])
        self.denom = torch.cat([self.denom, zeros])
        
        # For the split logic, we appended 2 children but KEPT the parent. 
        # The standard paper removes the parent (splits 1 -> 2).
        # My implementation above does 1 -> 3 (Keep parent + 2 children).
        # To strictly follow paper (1 -> 2), we should add the parents to the Prune Mask immediately.
        # But simply adding is often more stable for custom implementations. 

    def _prune(self, min_opacity):
        '''Given opacities of 3D Gaussians and a opacity threshould, this function determines which gaussian should be pruned to save memory.
        Finsh the function, calculate the pruning mask and remove gaussians with opacity < threshould.

        Args:
            threshold: Opacity threshould in float
        '''
        # Hint: Please use self.gaussians.opacities to get opacities of current gaussians, 
        # note that these opacities are raw opacities, you should apply an activation before directly using it. 
        # You can refer to func apply_activations in gaussians.py about applying activation to opacities
        
        ### YOUR CODE STARTS HERE ###
        mask_prune = torch.sigmoid(self.gaussians.opacities) < min_opacity
        ### YOUR CODE ENDS HERE ###
        
        if mask_prune.any():
            keep_mask = ~mask_prune
            # Update Tensors
            self.gaussians.means = torch.nn.Parameter(self.gaussians.means[keep_mask])
            self.gaussians.scales = torch.nn.Parameter(self.gaussians.scales[keep_mask])
            self.gaussians.quats = torch.nn.Parameter(self.gaussians.quats[keep_mask])
            self.gaussians.opacities = torch.nn.Parameter(self.gaussians.opacities[keep_mask])
            self.gaussians.colors = torch.nn.Parameter(self.gaussians.colors[keep_mask])
            
            # Update Optimizer
            new_params = {
                'means': self.gaussians.means, 'scales': self.gaussians.scales,
                'quats': self.gaussians.quats, 'opacities': self.gaussians.opacities,
                'colors': self.gaussians.colors
            }
            self.optimizer = update_optimizer_state(self.optimizer, new_params, keep_mask=keep_mask)
            
            # Update Accumulators
            self.xyz_gradient_accum = self.xyz_gradient_accum[keep_mask]
            self.denom = self.denom[keep_mask]