# HW2 Coding 任务计划书

> **极速版**：直接复制代码，最快跑通。  
> **手把手版**：每一步操作、验证、排错，适合卡住时用。

---

# 极速版（复制即用）

## 环境

```bash
pip install plyfile torchmetrics
# 解压 lego.zip 到 ./lego/
```

---

## gaussians.py

### compute_cov_3D（约 208 行）

```python
R = quaternion_to_matrix(quats)
S_diag = torch.exp(scales)
if self.is_isotropic:
    S = S_diag.unsqueeze(-1) * torch.eye(3, device=scales.device).unsqueeze(0)
else:
    S = torch.diag_embed(S_diag)
cov_3D = R @ S @ S.transpose(-1, -2) @ R.transpose(-1, -2)
```

### compute_means_projection（约 307 行）

```python
means_2D, depths = camera.project_2D(means_3D)
```

### compute_jacobian（约 253 行）

```python
eps = 1e-7
z_safe = tz + eps
z2_safe = tz2 + eps
J = torch.zeros((tx.shape[0], 2, 3), device=tx.device, dtype=tx.dtype)
J[:, 0, 0] = fx / z_safe
J[:, 0, 2] = -fx * tx / z2_safe
J[:, 1, 1] = fy / z_safe
J[:, 1, 2] = -fy * ty / z2_safe
```

### compute_cov_2D（约 279 行）

```python
N = means_3D.shape[0]
J = self.compute_jacobian(means_3D, camera, img_size)
W = camera.R.unsqueeze(0).expand(N, -1, -1)
cov_3D = self.compute_cov_3D(quats, scales)
cov_2D = J @ W @ cov_3D @ W.transpose(-1, -2) @ J.transpose(-1, -2)
```

### compute_gaussian_2D（约 336 行）

```python
diff = points_2D - means_2D
tmp = torch.einsum('nij,npj->npi', cov_2D_inverse, diff)
power = -0.5 * (diff * tmp).sum(dim=-1)
```

---

## render.py

### depth_sorting（约 44 行）

```python
valid = z_vals > 0
idxs = torch.where(valid)[0]
z_valid = z_vals[idxs]
_, sort_order = torch.sort(z_valid)
idxs = idxs[sort_order]
```

### compute_alphas（约 81 行）

```python
cov_2D_inverse = torch.linalg.inv(cov_2D)
exp_power = Gaussians.compute_gaussian_2D(points_2D, means_2D, cov_2D_inverse)
alphas = opacities.unsqueeze(1) * exp_power
```

### compute_transmittance（约 142 行）

```python
transmittance = torch.cumprod(one_minus_alphas, dim=0)[:-1]
```

### splat（约 191 行）

```python
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
```

### _densify（约 394 行）

```python
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
```

### _prune（约 476 行）

```python
mask_prune = torch.sigmoid(self.gaussians.opacities) < min_opacity
```

---

## gaussian_splatting.ipynb

### calculate_loss（Cell 5）

```python
l1_loss = torch.nn.functional.l1_loss(pred_img, gt_img)
ssim_loss = 1.0 - structural_similarity_index_measure(pred_img, gt_img, data_range=1.0)
total_loss = (1 - lamb) * l1_loss + lamb * ssim_loss
```

### setup_optimizer（学习率）

```python
parameters = [
    {'params': [gaussians.opacities], 'lr': 0.05, "name": "opacities"},
    {'params': [gaussians.scales], 'lr': 0.005, "name": "scales"},
    {'params': [gaussians.colors], 'lr': 0.0025, "name": "colors"},
    {'params': [gaussians.means], 'lr': 0.00016, "name": "means"},
    {'params': [gaussians.quats], 'lr': 0.001, "name": "quats"}
]
```

---

# 手把手版（按步骤操作）

## 前置准备

- 打开 `gaussians.py`、`render.py`、`gaussian_splatting.ipynb`
- 用 Ctrl+F 搜索 `### YOUR CODE STARTS HERE ###` 定位每个填空位置

---

## 1. compute_cov_3D

**文件**：`gaussians.py`  
**位置**：约 208 行，`if self.is_isotropic:` 和 `else:` 两个分支内

**操作**：
1. 删除 `if self.is_isotropic:` 和 `else:` 里的 `cov_3D = None`
2. 用极速版整段代码替换（含 `R`、`S_diag`、`if/else` 构造 `S`、`cov_3D`）

**验证**：运行 notebook 第一个训练 cell，若 `compute_cov_3D` 无报错，且 `cov_3D.shape == (N, 3, 3)`，则通过。

**若报错**：
- `quaternion_to_matrix` 找不到 → 文件顶部已有 `from utils import quaternion_to_matrix`，无需再 import
- `torch.diag_embed` 不可用 → 用 `torch.diag_embed(S_diag)` 或手动构造 `S = torch.eye(3).unsqueeze(0) * S_diag.unsqueeze(-1).unsqueeze(-1)`

---

## 2. compute_means_projection

**文件**：`gaussians.py`  
**位置**：约 307 行

**操作**：
1. 删除 `means_2D = None` 和 `depths = None`
2. 粘贴：`means_2D, depths = camera.project_2D(means_3D)`

**验证**：`means_2D.shape == (N, 2)`，`depths.shape == (N,)`。若 `depths.squeeze(0)` 报错，改为 `depths.squeeze()` 或直接返回 `depths`（取决于你 `project_2D` 的返回形状）。

---

## 3. compute_jacobian

**文件**：`gaussians.py`  
**位置**：约 253 行

**操作**：
1. 删除填空处内容
2. 粘贴极速版代码（注意 `tx, ty, tz, tz2, fx, fy` 已在前面定义）

**验证**：`J.shape == (N, 2, 3)`，且 `J` 与 `tx` 等在同一 device 上。

**若报错**：
- 除零 → 检查 `z_safe = tz + 1e-7` 是否已加

---

## 4. compute_cov_2D

**文件**：`gaussians.py`  
**位置**：约 279 行

**操作**：
1. 删除 `J = None`、`W = None`、`cov_3D = None`、`cov_2D = None`
2. 粘贴极速版代码

**验证**：`cov_2D.shape == (N, 2, 2)`。

---

## 5. compute_gaussian_2D

**文件**：`gaussians.py`  
**位置**：约 336 行

**操作**：
1. 删除 `power = None`
2. 粘贴极速版代码

**验证**：`power.shape == (N, H*W)`，后面 `weights = torch.exp(power)` 会用到。

**若不用 einsum**：
```python
diff = points_2D - means_2D  # (N, H*W, 2)
tmp = torch.bmm(diff.view(-1, diff.shape[1], 2), cov_2D_inverse.unsqueeze(1).expand(-1, diff.shape[1], -1, -1).reshape(-1, 2, 2))
# 更简单：用 (diff @ cov_2D_inverse) 的逐元素乘
tmp = torch.einsum('nij,npj->npi', cov_2D_inverse, diff)
power = -0.5 * (diff * tmp).sum(dim=-1)
```

---

## 6. depth_sorting

**文件**：`render.py`  
**位置**：约 44 行

**操作**：
1. 删除 `idxs = None`
2. 粘贴极速版代码

**验证**：`idxs` 为 `torch.int64`，长度 ≤ N，且 `z_vals[idxs]` 按升序排列。

---

## 7. compute_alphas

**文件**：`render.py`  
**位置**：约 81 行

**操作**：
1. 删除 `cov_2D_inverse = None`、`exp_power = None`、`alphas = None`
2. 粘贴极速版代码

**验证**：`alphas.shape == (N, H*W)`，后面会 `reshape` 成 `(N, H, W)`。

---

## 8. compute_transmittance

**文件**：`render.py`  
**位置**：约 142 行

**操作**：
1. 删除 `transmittance = None`
2. 粘贴：`transmittance = torch.cumprod(one_minus_alphas, dim=0)[:-1]`

**验证**：`transmittance.shape == (N, H, W)`。

---

## 9. splat

**文件**：`render.py`  
**位置**：约 191 行

**操作**：
1. 删除 `means_2D = None`、`cov_2D = None`、`alphas = None`、`transmittance = None`、`image = None`、`mask = None`
2. 粘贴极速版代码（注意 `cov_2D[:, 0, 0] += 0.0005` 等两行已在模板里，不要重复）

**验证**：`image.shape == (H, W, 3)`，`mask.shape == (H, W, 1)`。

---

## 10. _densify

**文件**：`render.py`  
**位置**：约 394 行

**操作**：
1. 删除 `new_means_c = None` 等所有 `None` 赋值
2. 粘贴极速版代码（注意 `new_scales_s` 要减 `math.log(1.6)`）

**验证**：当 `idx_clone` 或 `idx_split` 为空时，`torch.cat` 仍可正常工作（空 tensor 参与 cat 不会报错）。

---

## 11. _prune

**文件**：`render.py`  
**位置**：约 476 行

**操作**：
1. 删除 `mask_prune = None`
2. 粘贴：`mask_prune = torch.sigmoid(self.gaussians.opacities) < min_opacity`

**验证**：`mask_prune` 为 bool tensor，`True` 表示要删除。

---

## 12. calculate_loss

**文件**：`gaussian_splatting.ipynb`，Cell 5

**操作**：
1. 删除 `l1_loss = None`、`ssim_loss = None`、`total_loss = None`
2. 粘贴极速版代码

**验证**：`total_loss` 为标量 tensor，`l1_loss`、`ssim_loss` 可 `.detach().item()`。

---

## 13. 训练与超参

**学习率**：在 `setup_optimizer` 里按极速版修改（opacities 0.05，scales 0.005 等）。

**训练**：迭代 3000–4000，目标 PSNR > 30 dB。若 loss 不降，可适当调小学习率。

---

## 常见报错速查

| 报错 | 原因 | 处理 |
|------|------|------|
| `quaternion_to_matrix` 找不到 | 未 import | `from utils import quaternion_to_matrix` |
| `depths.squeeze(0)` 报错 | depths 已是 (N,) | 改为 `depths.squeeze()` 或直接返回 |
| Jacobian 除零 | z 为 0 | 用 `z_safe = tz + 1e-7` |
| cov_2D 奇异无法求逆 | 协方差退化 | 加 `cov_2D + 1e-6 * I` |
| _densify 中 cat 空 tensor 报错 | idx 为空 | 空 tensor 可参与 cat，无需额外判断 |
| PSNR 上不去 | 学习率或迭代数 | 调低 lr，增加 num_itrs |

---

## 推荐实现顺序

1. `compute_cov_3D` → `compute_means_projection` → `compute_jacobian` → `compute_cov_2D` → `compute_gaussian_2D`
2. `depth_sorting` → `compute_alphas` → `compute_transmittance` → `splat`
3. `calculate_loss` → 跑通训练
4. `_prune` → `_densify` → 调超参 → 保存 PNG

每完成一块可先跑 notebook 验证，再继续下一块。
