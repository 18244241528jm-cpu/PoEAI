# 第 3 讲：April Tag 与平面位姿实战

**对应讲义**：03_single_view_pose_estimation.pdf 第 11–18 页  
**本讲目标**：把第 2 讲用于真实系统：用 April Tag 提供平面点对应，得到机器人（相机）相对平面的位姿，并做质量检查。

---

## 先想清楚：这讲到底在解什么问题？

前两讲我们已经知道了理论：从点对应求 $H$，从 $H$ 分解出 $(R,\mathbf{t})$。但理论怎么落地？在真实场景里，"已知的平面 3D 点"从哪来？"图像上的 2D 点"怎么自动检测？如果结果不好怎么判断？这一讲就是把整条 pipeline 从头到尾在真实系统里走一遍。

---

## 和前面课程的关系

| 讲次 | 角色 | 关键词 |
|------|------|--------|
| **第 2 讲** | 给了算法 | 平面单应 $H$ 的 DLT 求解、$H$ 分解为 $(R,\mathbf{t})$ |
| **第 3 讲（本讲）** | 把算法用起来 | April Tag 提供真实点对应 → 完整 pipeline → 重投影质量检查 |
| **第 4 讲（下一讲）** | 打破限制 | 如果场景**不是平面**怎么办？→ PnP 与非平面方法 |

**🧭 第 2 讲给你的是"工具"，本讲教你"怎么在真实场景里使用这把工具"。第 4 讲会告诉你"如果工具的前提条件不满足（非平面），怎么换一把"。**

---

## 1. April Tag 简介

> 📖 **对照 PPT**：打开 03_single_view_pose_estimation.pdf **第 11 页** "April Tags"，有 April Tag 的实物照片——黑白方块组成的方形图案，中间编码了 ID。

### 1.1 什么是 April Tag

**April Tag** 是一种**视觉基准标记（fiducial marker）**：

- 由黑白方块组成的**方形图案**，中间编码一个 **ID**（用于区分不同 tag）。
- 检测算法能稳定地找到 tag 的**四个角点**在图像中的像素坐标。
- 每个 tag 的**物理尺寸**（边长等）已知，且我们约定 tag 所在平面为 $Z=0$，在平面上建立坐标系，则**四个角点的 3D 世界坐标已知**。

因此：**一个 April Tag 天然提供 4 对"平面上的 3D 点 ↔ 图像 2D 点"的对应**，正好满足（并超过）平面单应 $H$ 所需的最少 4 对点。

### 1.2 为何适合做平面位姿

- **平面**：Tag 贴在同一平面上（或 tag 本身视为平面），满足"平面诱导单应"的前提。
- **已知 3D 点**：角点在世界系下的坐标由 tag 尺寸和 ID 定义确定（例如以 tag 中心为原点、边长已知）。
- **稳定检测**：有成熟库（如 `apriltag`、OpenCV 等）可输出角点像素坐标，便于做 DLT → $H$ → 分解 $(R,\mathbf{t})$。

---

## 2. 机器人相对平面的位姿（问题定义）

> 📖 **对照 PPT**：打开 03_single_view_pose_estimation.pdf **第 12 页** "Pose of robot with respect to a planar surface"，示意图展示了机器人相机看向贴有 April Tag 的平面。

设：

- **平面**：例如地面、墙面或一块贴有 April Tag 的板子；在平面上建立**世界坐标系**（如以某个 tag 中心为原点、$Z$ 轴朝外法向）。
- **相机**：装在机器人上，内参 $K$ 已标定。
- **目标**：求**相机坐标系相对于该平面（世界坐标系）的位姿** $(R, \mathbf{t})$，即 **world-to-camera**：  
  若平面上一点的世界坐标为 $\mathbf{X}_w$，在相机系下为 $\mathbf{X}_c = R \mathbf{X}_w + \mathbf{t}$。

得到 $(R, \mathbf{t})$ 后，就知道机器人（相机）相对该平面如何放置（位置 + 朝向），可用于导航、对接、AR 等。

---

## 3. 从 April Tag 到单应 $H$

> 📖 **对照 PPT**：打开 03_single_view_pose_estimation.pdf **第 13–14 页** "Example: pose from April Tags" / "Homography"，第 14 页标注了 World measurement 和 Pixel measurement 的对应关系。

### 3.1 世界坐标（Tag 角点）

在**世界坐标系**下，把平面取为 $Z=0$。每个 April Tag 的四个角点在该系下的坐标是已知的，例如（以 tag 中心为原点、边长 $s$）：

- 角点 1：$(-s/2, s/2, 0)$  
- 角点 2：$(s/2, s/2, 0)$  
- 角点 3：$(s/2, -s/2, 0)$  
- 角点 4：$(-s/2, -s/2, 0)$  

写成**平面上的 2D 齐次坐标**（因为 $Z=0$，只关心 $X,Y$）：  
$\mathbf{p}_1 = (-s/2, s/2, 1)^\top$，…，$\mathbf{p}_4 = (-s/2, -s/2, 1)^\top$。

### 3.2 像素坐标（检测结果）

用 April Tag 检测库对当前图像做检测，得到每个 tag 的**四个角点在图像中的像素坐标** $(u_i, v_i)$，$i=1..4$。写成齐次：$\mathbf{x}_i = (u_i, v_i, 1)^\top$。

### 3.3 求单应 $H$

我们有 4 对对应：$\mathbf{p}_i \leftrightarrow \mathbf{x}_i$，满足 $\mathbf{x}_i \sim H \mathbf{p}_i$。

- **方法 1**：第 1 讲的 **DLT**——由每对点写出 2 个线性方程，组成 $A\mathbf{h}=\mathbf{0}$，SVD 求 $H$。  
- **方法 2**：OpenCV：`H, mask = cv2.findHomography(srcPoints, dstPoints)`，其中 `srcPoints` 为平面 2D（如角点的 $(X,Y)$），`dstPoints` 为像素 $(u,v)$。

得到 3×3 的 $H$ 后，就得到"平面 → 图像"的射影变换。

---

## 4. 从单应恢复位姿 $(R, \mathbf{t})$

第 2 讲已经完整推导：在已知内参 $K$ 的前提下，从 $H$ 可分解出 $(R, \mathbf{t})$。核心等式是：

$$
H = \lambda \, K \, [\mathbf{r}_1 \mid \mathbf{r}_2 \mid \mathbf{t}]
$$

其中 $\mathbf{r}_1, \mathbf{r}_2$ 是旋转矩阵 $R$ 的前两列，$\mathbf{t}$ 是平移，$\lambda$ 是未知缩放因子。**拆解的思路就是"反过来把 $K$ 和 $\lambda$ 消掉"**，具体分 5 步：

### Step 1：左乘 $K^{-1}$，去掉内参的影响

把 $H$ 的三列 $\mathbf{h}_1, \mathbf{h}_2, \mathbf{h}_3$ 分别左乘 $K^{-1}$，记作 $\hat{\mathbf{h}}_1, \hat{\mathbf{h}}_2, \hat{\mathbf{h}}_3$（即 $\hat{\mathbf{h}}_i = K^{-1}\mathbf{h}_i$）：

$$
\hat{\mathbf{h}}_1 = K^{-1}\mathbf{h}_1 = \lambda \, \mathbf{r}_1, \quad
\hat{\mathbf{h}}_2 = K^{-1}\mathbf{h}_2 = \lambda \, \mathbf{r}_2, \quad
\hat{\mathbf{h}}_3 = K^{-1}\mathbf{h}_3 = \lambda \, \mathbf{t}
$$

等价地，令 $H' = K^{-1} H$，则 $H' = \lambda [\mathbf{r}_1 \mid \mathbf{r}_2 \mid \mathbf{t}]$，其列即为 $\hat{\mathbf{h}}_1, \hat{\mathbf{h}}_2, \hat{\mathbf{h}}_3$。

这一步本质上是把"像素坐标空间"变回"归一化相机坐标空间"，消除了焦距和主点的影响。但 $\hat{\mathbf{h}}_i$ 还差一个未知的缩放因子 $\lambda$。

### Step 2：求 $\lambda$（利用旋转矩阵的列是单位向量）

$\mathbf{r}_1$ 是旋转矩阵的一列，模长必须等于 1。所以 $\lVert \hat{\mathbf{h}}_1 \rVert = \lVert \lambda \mathbf{r}_1 \rVert = \lvert \lambda \rvert$，即：

$$
\lambda = \frac{1}{2}\Big(\lVert \hat{\mathbf{h}}_1 \rVert + \lVert \hat{\mathbf{h}}_2 \rVert\Big)
$$

取两列的平均是为了减小数值误差。

### Step 3：除以 $\lambda$，得到 $\mathbf{r}_1, \mathbf{r}_2, \mathbf{t}$，叉乘补全 $\mathbf{r}_3$

$$
\mathbf{r}_1 = \frac{\hat{\mathbf{h}}_1}{\lambda}, \quad
\mathbf{r}_2 = \frac{\hat{\mathbf{h}}_2}{\lambda}, \quad
\mathbf{t} = \frac{\hat{\mathbf{h}}_3}{\lambda}
$$

旋转矩阵有 3 列，但 $H$ 只提供了前 2 列的信息（因为平面上 $Z=0$，第三列被"吃掉"了）。用叉乘补全：$\mathbf{r}_3 = \mathbf{r}_1 \times \mathbf{r}_2$，拼成 $R = [\mathbf{r}_1 \mid \mathbf{r}_2 \mid \mathbf{r}_3]$。

### Step 4：符号修正（$\det(R)$ 检查）

$H$ 是齐次的（乘以 $-1$ 还是同一个 $H$），所以分解时 $(R, \mathbf{t})$ 和 $(-R, -\mathbf{t})$ 都是合法解。约定取 $\det(R)=+1$（合法旋转矩阵）：若 $\det(R)=-1$，令 $R \leftarrow -R,\; \mathbf{t} \leftarrow -\mathbf{t}$。

### Step 5：最近旋转校正（SVD 投影到 $SO(3)$）

由于数值误差，Step 3 拼出的 $R$ 不一定严格正交（$R^\top R \neq I$）。修正方法：对 $R$ 做 SVD，$R = U\Sigma V^\top$，令 $R^* = UV^\top$。

> **直觉**：SVD 把"接近旋转但不完全正交"的矩阵拆成"旋转 × 拉伸 × 旋转"，丢掉中间的拉伸 $\Sigma$，只保留两端的旋转合成——这就是离原矩阵最近的合法旋转。

### 一图总结

```
H  ─── K⁻¹·各列 ───→  ĥ₁,ĥ₂,ĥ₃（ĥᵢ=K⁻¹hᵢ）
                            │
                        ÷ λ（用 ‖ĥ₁‖=‖ĥ₂‖=|λ| 求出）
                            │
                            ▼
                       [r₁ | r₂ | t]
                            │
                        r₃ = r₁ × r₂
                            │
                            ▼
                     R = [r₁ | r₂ | r₃]
                            │
                    det(R)=-1 ? 翻转符号
                            │
                     SVD 投影到 SO(3)
                            │
                            ▼
                        (R*, t) ✓
```

> 详细数值算例见**第 2 讲**第 6 节，用 $K=\text{diag}(500,500,1)$（主点 320,240）和一组具体的 $H$ 走了完整的 5 步计算。

因此**完整 pipeline**为：

```
图像 → April Tag 检测 → 角点像素 + 角点世界坐标 → findHomography / DLT → H
→ 用 K⁻¹ 消内参 → 用列模长求 λ → 除以 λ 得 r₁ r₂ t → 叉乘补 r₃ → 符号修正 → SVD 校正 → (R, t)
```

**🧭 到这里完整 pipeline 已经走通了。但实际跑起来结果可能不好——下面讲为什么以及怎么检查。**

---

## 5. 结果不好的常见原因（Not Great! Why?）

> 📖 **对照 PPT**：打开 03_single_view_pose_estimation.pdf **第 17 页** "Recover pose — Not Great! Why?"，PPT 上列出了四个原因：Non-planar, wrong detection, motion-blur, wrong calibration。

讲义中概括为四类，对应实际中"位姿跳变、明显错误"的典型来源：

| 原因 | 说明 |
|------|------|
| **非平面 (Non-planar)** | 物体或 tag 有弯曲、遮挡、不共面；或多 tag 不在同一平面却被混在一起求了一个 $H$。单应假设"平面 → 图像"被破坏。 |
| **误检测 (Wrong detection)** | 角点检测错、ID 识别错、误把其他图案当 tag。对应关系错 → $H$ 错 → $(R,\mathbf{t})$ 错。 |
| **运动模糊 (Motion blur)** | 相机或场景运动导致图像模糊，角点定位不准，$H$ 估计变差。 |
| **标定误差 (Wrong calibration)** | 内参 $K$ 不准。分解时 $H = \lambda K[\mathbf{r}_1 \mid \mathbf{r}_2 \mid \mathbf{t}]$ 依赖 $K$，$K$ 错会导致 $R$、$\mathbf{t}$ 系统性偏差甚至明显错误。 |

**应对思路**：多 tag 时用 RANSAC 选一致平面、过滤误检；模糊时提高曝光或降速；定期重新标定；下面用**重投影检查**做质量筛选。

---

## 6. 重投影检查（Reprojection Check）

> 📖 **对照 PPT**：打开 03_single_view_pose_estimation.pdf **第 18 页** "Reprojection Check"，有重投影示意图。

重投影检查的核心思想：**用估计出的位姿把已知 3D 点"投回"图像，看和实际检测到的 2D 点差多少**。差得少 → 位姿可信；差得多 → 有问题，别用。

### 6.1 第一步：把 3D 点投影到图像

用当前估计的 $(R, \mathbf{t})$ 和内参 $K$，把**已知的 3D 点**（如 tag 角点的世界坐标 $\mathbf{X}_i$）投影回图像：

$$
\mathbf{x}_i^{\text{pred}} \sim K \big( R \mathbf{X}_i + \mathbf{t} \big)
$$

**逐项解释**：

| 符号 | 含义 |
|------|------|
| $\mathbf{X}_i$ | 第 $i$ 个角点的**世界坐标**（齐次 3D，如 $(-s/2, s/2, 0)$） |
| $R \mathbf{X}_i + \mathbf{t}$ | 变换到**相机坐标系**下的 3D 点 |
| $K(\cdots)$ | 用内参把相机系 3D 点投影到**图像平面**，得到齐次像素坐标 |
| $\mathbf{x}_i^{\text{pred}}$ | 投影后的齐次坐标，形如 $(s \cdot u, s \cdot v, s)^\top$ |
| $\sim$ | 齐次等价：除以第三维 $s$ 得到像素 $(u, v)$ |

**具体计算**：设 $\mathbf{p} = K(R\mathbf{X}_i + \mathbf{t})$，则像素坐标为 $(u_i^{\text{pred}}, v_i^{\text{pred}}) = (p_1/p_3, p_2/p_3)$。

### 6.2 第二步：算重投影误差

将 $\mathbf{x}_i^{\text{pred}}$ 转为像素 $(u_i^{\text{pred}}, v_i^{\text{pred}})$，与**检测到的角点像素** $(u_i, v_i)$ 比较，算**欧氏距离**（L2 范数）：

$$
e_i = \big\lVert (u_i, v_i) - (u_i^{\text{pred}}, v_i^{\text{pred}}) \big\rVert
$$

即 $e_i = \sqrt{(u_i - u_i^{\text{pred}})^2 + (v_i - v_i^{\text{pred}})^2}$，单位是**像素**。

**直觉**：$e_i$ 表示"第 $i$ 个角点：检测位置和用位姿预测的位置差了多少像素"。理想情况下，若位姿、标定、检测都正确，$e_i$ 应接近 0。

### 6.3 第三步：用误差判断位姿是否可靠

对 $n$ 个角点（如 April Tag 的 4 个），可选用以下指标之一与阈值比较：

| 指标 | 公式 | 含义 |
|------|------|------|
| **单点误差** | $e_i$ | 第 $i$ 个点的误差 |
| **最大误差** | $\max_i e_i$ | 最差的那个点——更严格 |
| **平均误差** | $\frac{1}{n}\sum_i e_i$ | 整体水平——最常用 |

**判断规则**：

- **小于阈值**（如 1–2 像素）→ 位姿**可靠**，可用于控制、建图或输出。
- **大于阈值** → 位姿**不可信**，可能原因：
  - **检测有问题**：角点 $(u_i, v_i)$ 检测错、ID 识别错
  - **标定有问题**：内参 $K$ 不准
  - **平面假设被破坏**：tag 弯曲、遮挡、多 tag 不共面

**应对**：丢弃该帧，或触发报警，不把不可靠的位姿传给下游。

### 6.4 用途小结

- **筛选**：只接受重投影误差小的 $(R, \mathbf{t})$，用于控制或建图。  
- **调试**：误差大时排查是检测错、标定错还是非平面。  
- **可视化**：在图像上画出"检测角点"和"重投影点"，一目了然——两者重合则好，错开则差。

---

## 7. 代码实战：完整 pipeline 走一遍

既然本讲是"实战"，下面用 Python 伪代码把整条 pipeline 串起来。阅读时请对照上面各节的数学步骤。

```python
import cv2
import numpy as np

# ========== 0. 准备 ==========
# 内参矩阵 K（从标定结果读取）
K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0,  0,  1]], dtype=float)

# ========== 1. April Tag 检测 ==========
# 使用 apriltag 库（pip install apriltag）或 OpenCV aruco 模块
import apriltag
detector = apriltag.Detector()
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
results = detector.detect(gray_image)

# ========== 2. 取出一个 tag 的 4 个角点像素坐标 ==========
tag = results[0]
tag_corners_pixel = tag.corners  # shape (4, 2)，单位：像素

# ========== 3. 定义该 tag 角点的世界坐标（Z=0 平面） ==========
tag_size = 0.1  # 边长 10cm，单位：米
s = tag_size / 2
world_pts = np.array([
    [-s,  s],   # 角点 1
    [ s,  s],   # 角点 2
    [ s, -s],   # 角点 3
    [-s, -s],   # 角点 4
], dtype=float)

# ========== 4. 求单应 H ==========
H, mask = cv2.findHomography(world_pts, tag_corners_pixel)

# ========== 5. 分解 H → (R, t) ==========
# 方式 A：手动用第 2 讲公式
K_inv = np.linalg.inv(K)
M = K_inv @ H
lam = 1.0 / np.linalg.norm(M[:, 0])
r1 = lam * M[:, 0]
r2 = lam * M[:, 1]
t  = lam * M[:, 2]
r3 = np.cross(r1, r2)
R  = np.column_stack([r1, r2, r3])
# SVD 最近旋转校正
U, _, Vt = np.linalg.svd(R)
R = U @ Vt

# 方式 B：直接用 cv2.solvePnP（添加 Z=0）
world_pts_3d = np.hstack([world_pts, np.zeros((4, 1))])
_, rvec, tvec = cv2.solvePnP(world_pts_3d, tag_corners_pixel, K, None)
R_pnp, _ = cv2.Rodrigues(rvec)

# ========== 6. 重投影检查 ==========
errors = []
for i in range(4):
    X_w = np.array([world_pts[i, 0], world_pts[i, 1], 0.0])
    X_c = R @ X_w + t
    proj = K @ X_c
    u_pred, v_pred = proj[0] / proj[2], proj[1] / proj[2]
    u_det, v_det = tag_corners_pixel[i]
    err = np.sqrt((u_pred - u_det)**2 + (v_pred - v_det)**2)
    errors.append(err)

mean_err = np.mean(errors)
print(f"平均重投影误差: {mean_err:.2f} 像素")
if mean_err > 2.0:
    print("⚠️ 误差过大，位姿不可信——检查标定/检测/平面假设")
```

**重点说明**：

- **步骤 3** 中 `world_pts` 的单位必须和你想要的 $\mathbf{t}$ 单位一致（通常用米）。
- **步骤 5** 的方式 A 是本课推导的公式，方式 B 是 OpenCV 内置 PnP（第 4 讲会详细讲，这里先当"黑盒"对比用）。
- **步骤 6** 的重投影误差如果 < 1 像素，通常说明结果很好。

---

## 8. 完整 pipeline 小结

1. **输入**：图像、内参 $K$、April Tag 尺寸（及可选的世界系定义）。  
2. **检测**：April Tag 库 → 每个 tag 的 ID + 四个角点像素坐标。  
3. **对应**：每个角点的世界坐标（平面 $Z=0$）↔ 像素坐标。  
4. **单应**：`cv2.findHomography(plane_pts, pixel_pts)` 或 DLT → $H$。  
5. **位姿**：用 $K$ 分解 $H$ → $(R, \mathbf{t})$（第 2 讲算法）。  
6. **检查**：把角点 3D 用 $(R, \mathbf{t})$ 和 $K$ 重投影，算误差；若误差过大则丢弃或标记。  
7. **输出**：可靠的 $(R, \mathbf{t})$（相机相对平面）。

---

## 9. 本讲小结与"学完你能做什么"

| 内容 | 要点 |
|------|------|
| **April Tag** | 平面上的方形编码标记；提供 4 个角点的 2D–3D 对应，适合求平面单应与位姿。 |
| **问题** | 求相机（机器人）相对"贴有 April Tag 的平面"的 $(R, \mathbf{t})$。 |
| **从 Tag 到 $H$** | 角点世界坐标（平面 2D）↔ 角点像素坐标；DLT 或 `findHomography` 得 $H$。 |
| **从 $H$ 到位姿** | 调用第 2 讲的分解（$K$ + $\lambda$ + $\mathbf{r}_1,\mathbf{r}_2,\mathbf{r}_3,\mathbf{t}$ + 符号与最近旋转）。 |
| **结果差的原因** | 非平面、误检测、运动模糊、标定误差。 |
| **重投影检查** | 用 $(R,\mathbf{t})$ 和 $K$ 把 3D 角点投回图像，与检测点比较，用于筛选与调试。 |

**学完本讲你应该能**：

1. 说明 April Tag 是什么、为何适合做平面位姿。  
2. 在代码里实现或调用：April Tag 检测 → 角点对应 → `findHomography` → 分解得 $(R, \mathbf{t})$。  
3. 实现重投影并计算误差，用于判断结果是否可信。  
4. 解释"结果不好"的常见原因及改进方向。

---

## 10. 自测与练习建议

1. **查阅**：选一个 April Tag 库（如 Python `apriltag` 或 OpenCV 的 aruco/tag），看文档里如何获取角点像素和 tag 尺寸。  
2. **编程**：用一张带 April Tag 的图，跑通"检测 → 单应 → 分解 → 重投影"，在图上画出检测点与重投影点。  
3. **思考**：若场景中有多个 tag 在同一平面，如何用多 tag 的角点一起求一个 $H$？有什么好处？
4. **口述练习**：不看笔记，用自己的话把完整 pipeline 从"拿到一张图"到"输出可靠位姿"讲一遍（< 2 分钟）。如果某个环节卡住，说明那里还需要复习。

完成以上后，可以进入 **第 4 讲：非平面场景与 PnP 问题**——当场景不再是平面时，$H$ 不再适用，我们需要新的工具。
