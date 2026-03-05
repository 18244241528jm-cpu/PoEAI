# 第 6 讲：PnP 的解法二 —— P3P 最小解与 RANSAC

**对应讲义**：03_single_view_pose_estimation.pdf 第 37–48 页；可参考 04_ransac_procrustes.pdf、p3p_review.pdf  
**本讲目标**：理解"最少 3 个点可解 PnP 但有歧义 (Ambiguity)"，掌握 P3P 的几何与多项式求解思路，以及用第 4 点去歧义、用 RANSAC 抗外点 (Outliers)。

---

## 先想清楚：这讲到底在解什么问题？

上一讲的非线性优化有两个致命弱点：怕外点（错误匹配，Outliers）、怕初值 (Initialization) 不好。这一讲给出另一种思路：不用所有点，每次只随机挑 3 个点，用纯几何方法直接算出一组（最多 4 组）位姿候选——这就是 P3P。然后用 RANSAC 框架反复抽样、投票，找到最多"内点 (Inliers)"支持的那组解。这样既抗外点，又能给非线性优化提供好的初值。

---

## 和前面课程的关系

| 讲次 | 核心方法 | 特点 |
|------|---------|------|
| **第 5 讲** | 非线性最小二乘 | 精确但脆弱——需要好初值，怕外点 |
| **第 6 讲（本讲）** | P3P + RANSAC | 鲁棒——最少点求解析解，天然抗外点 |

实际工程中几乎总是**两者结合**使用：

1. **RANSAC + P3P**：在含外点的数据中找到内点 (Inliers) 集合，并给出一组不错的初始位姿。
2. **非线性优化（第 5 讲）**：在筛选出的内点上做精化，把位姿调到最准。

此外，P3P 的最后一步——从"世界系三点"和"相机系三点"的对应求 $(R, \mathbf{t})$——正是 **Procrustes（3D–3D 配准）** 问题，在 **04 系列第 1 讲** 中已经详细讲过。本讲可以直接调用那里的结论。

---

## 1. 最小解 + 外点拒绝（Solution 2: Minimal solver + outlier rejection）

> 📖 **对照 PPT**：打开 03_single_view_pose_estimation.pdf **第 37 页** "Solution 2: Minimal solver + outlier rejection"，概述了"3 点求解 + 第 4 点去歧义 + RANSAC"的完整思路。

第 5 讲用**非线性最小二乘**最小化重投影误差，但有两个问题：

- **外点**：错误对应会严重破坏结果。
- **初值**：需要较好的初始 $(R, \mathbf{t})$ 才能收敛。

另一种思路是：

1. **最小解 (Minimal solver)**：用**最少点数**（3 个点）从几何上**直接**求出一组或多组位姿候选（**P3P**）。
2. **外点拒绝 (Outlier rejection)**：用 **RANSAC** 反复抽最小子集、求候选解，用**内点数量**投票，选内点最多的解；必要时再用全部内点做非线性优化精化。

这样得到的解**对外点鲁棒**，且 P3P 给出的解可作为非线性优化的**初值**。

---

## 2. P3P 问题（Perspective-3-Point）

> 📖 **对照 PPT**：打开 03_single_view_pose_estimation.pdf **第 38 页** "P3P (Perspective-3-point) problem"，有从相机出发的三条射线到三个 3D 点的几何图。**这是理解 P3P 最关键的一张图，务必打开看。**

### 2.1 设定

- **已知**：3 个 3D 点在世界系下的坐标 $\mathbf{X}_1, \mathbf{X}_2, \mathbf{X}_3$（故三角形边长 $a, b, c$ 已知）；3 条从相机出发的**射线方向**（由 3 个像素 + 内参 $K$ 得到，可视为单位向量 $\mathbf{j}_1, \mathbf{j}_2, \mathbf{j}_3$）。
- **未知**：相机位姿 $(R, \mathbf{t})$；等价地，可先求**相机到三点的距离** $d_1, d_2, d_3$（即深度，Depth），则三点在**相机系**下的坐标为 $\mathbf{p}_i = d_i \mathbf{j}_i$，再通过 3D–3D 配准 (3D-3D Registration) 求 $R, \mathbf{t}$。

P3P 是 PnP 的**最小解 (Minimal solver)**：用 3 个点即可求位姿；但最多有 **4 组**满足"点在相机前方"的解，需要**第 4 个点**（或重投影误差，Reprojection error）来选出一组。

---

**🧭 P3P 问题设定清楚了。下面分四步求解：归一化射线 → 余弦定理列方程 → 消元得四次多项式 → 求根得距离。**

---

## 3. P3P Step 1：归一化到射线 (Normalize to rays)

> 📖 **对照 PPT**：打开 03_single_view_pose_estimation.pdf **第 39 页** "P3P: step 1 – normalize to rays"。

图像上的像素 $(u_i, v_i)$ 对应相机系下的一条射线。用内参 $K$ 将像素转为**归一化平面**上的点（或直接转为单位方向）：

$$
\mathbf{j}_i = \frac{K^{-1} (u_i, v_i, 1)^\top}{\big\| K^{-1} (u_i, v_i, 1)^\top \big\|}, \qquad i = 1,2,3
$$

则 $\mathbf{j}_i$ 为从光心出发、指向第 $i$ 个像点的**单位向量**。在相机系下，若该 3D 点距光心为 $d_i$，则其相机坐标为 $\mathbf{p}_i = d_i \mathbf{j}_i$。  
世界系下三点为 $\mathbf{X}_1, \mathbf{X}_2, \mathbf{X}_3$，边长：

$$
a = \|\mathbf{X}_2 - \mathbf{X}_3\|, \quad b = \|\mathbf{X}_1 - \mathbf{X}_3\|, \quad c = \|\mathbf{X}_1 - \mathbf{X}_2\|
$$

---

## 4. P3P Step 2–3：三角与余弦定理 (Trigonometry + Cosine Law)

> 📖 **对照 PPT**：打开 03_single_view_pose_estimation.pdf **第 40–42 页** "P3P: step 2 – trigonometry" / "Cosine Law" / "Triangle Cosine Law"，有四面体的几何图和余弦定理的三个方程。

在相机系下，三点为 $\mathbf{p}_i = d_i \mathbf{j}_i$，故

$$
\|\mathbf{p}_2 - \mathbf{p}_3\|^2 = \|d_2\mathbf{j}_2 - d_3\mathbf{j}_3\|^2 = d_2^2 + d_3^2 - 2 d_2 d_3 \cos\alpha
$$

其中 $\alpha$ 为射线 $\mathbf{j}_2$ 与 $\mathbf{j}_3$ 的夹角：$\cos\alpha = \mathbf{j}_2^\top \mathbf{j}_3$。同理定义 $\beta = \angle(\mathbf{j}_1, \mathbf{j}_3)$，$\gamma = \angle(\mathbf{j}_1, \mathbf{j}_2)$。  
三角形边长在世界系和相机系下应一致（刚体），故：

$$
\begin{aligned}
d_2^2 + d_3^2 - 2 d_2 d_3 \cos\alpha &= a^2 \\
d_1^2 + d_3^2 - 2 d_1 d_3 \cos\beta  &= b^2 \\
d_1^2 + d_2^2 - 2 d_1 d_2 \cos\gamma  &= c^2
\end{aligned}
$$

这就是**三角形余弦定理 (Cosine Law)** 在 P3P 中的形式：三个未知数 $d_1, d_2, d_3$，三个二次方程。

### 🔢 数值示例：从像素到余弦定理方程

为了加深直觉，用一组具体数字走一遍上述过程。

**给定数据：**

- 世界系三点：$\mathbf{X}_1 = (0,0,0)$，$\mathbf{X}_2 = (1,0,0)$，$\mathbf{X}_3 = (0,1,0)$
- 内参矩阵：

$$
K = \begin{pmatrix} 500 & 0 & 320 \\ 0 & 500 & 240 \\ 0 & 0 & 1 \end{pmatrix}
$$

- 像素坐标：$(u_1,v_1) = (420, 190)$，$(u_2,v_2) = (520, 195)$，$(u_3,v_3) = (415, 90)$

**Step A：求单位射线 $\mathbf{j}_i$**

用 $K^{-1}$ 将像素转到相机系再归一化：

$$
K^{-1} \begin{pmatrix}420\\190\\1\end{pmatrix} = \begin{pmatrix}0.200\\-0.100\\1.000\end{pmatrix}
\;\xrightarrow{\text{归一化}}\;
\mathbf{j}_1 = \begin{pmatrix}0.1952\\-0.0976\\0.9759\end{pmatrix}
$$

$$
K^{-1} \begin{pmatrix}520\\195\\1\end{pmatrix} = \begin{pmatrix}0.400\\-0.090\\1.000\end{pmatrix}
\;\xrightarrow{\text{归一化}}\;
\mathbf{j}_2 = \begin{pmatrix}0.3701\\-0.0833\\0.9253\end{pmatrix}
$$

$$
K^{-1} \begin{pmatrix}415\\90\\1\end{pmatrix} = \begin{pmatrix}0.190\\-0.300\\1.000\end{pmatrix}
\;\xrightarrow{\text{归一化}}\;
\mathbf{j}_3 = \begin{pmatrix}0.1790\\-0.2827\\0.9423\end{pmatrix}
$$

**Step B：求边长 $a, b, c$**

$$
a = \|\mathbf{X}_2 - \mathbf{X}_3\| = \sqrt{1^2+1^2} = \sqrt{2} \approx 1.4142, \quad
b = \|\mathbf{X}_1 - \mathbf{X}_3\| = 1, \quad
c = \|\mathbf{X}_1 - \mathbf{X}_2\| = 1
$$

**Step C：求射线夹角余弦**

$$
\cos\alpha = \mathbf{j}_2^\top \mathbf{j}_3 = 0.9617 \quad (\alpha \approx 15.91°)
$$

$$
\cos\beta = \mathbf{j}_1^\top \mathbf{j}_3 = 0.9822 \quad (\beta \approx 10.83°)
$$

$$
\cos\gamma = \mathbf{j}_1^\top \mathbf{j}_2 = 0.9833 \quad (\gamma \approx 10.48°)
$$

**Step D：写出余弦定理方程**

把数字代入，得到关于 $d_1, d_2, d_3$ 的三个方程：

$$
\begin{aligned}
d_2^2 + d_3^2 - 2 \times 0.9617 \; d_2 d_3 &= 2.0000 \quad \text{…(1)} \\
d_1^2 + d_3^2 - 2 \times 0.9822 \; d_1 d_3 &= 1.0000 \quad \text{…(2)} \\
d_1^2 + d_2^2 - 2 \times 0.9833 \; d_1 d_2 &= 1.0000 \quad \text{…(3)}
\end{aligned}
$$

接下来，P3P 的后续步骤就是从这三个方程中消元，化为一个关于单一变量的四次多项式（见第 5 节）。

> **口述练习**：拿到 3 个像素和 $K$ 后，你能一口气说出"先乘 $K^{-1}$ 再归一化得射线→算内积得余弦→代入余弦定理列出三个方程"这条线吗？试着对同学/自己说一遍。

---

## 5. P3P Step 4：消元得到四次多项式 (Solve by elimination)

> 📖 **对照 PPT**：打开 03_single_view_pose_estimation.pdf **第 43 页** "P3P: Step 4 - solve by elimination"，提到 "Equation (1) to (9), solving a fourth order polynomial"。

将 $d_2 = u d_1$、$d_3 = v d_1$ 代入上述三式，可消去 $d_1$，得到关于 $u, v$ 的方程。再从中消去 (Eliminate) $u$（或 $v$），最终得到关于**一个变量**（如 $v$）的**四次多项式 (Fourth order polynomial)**：

$$
A_4 v^4 + A_3 v^3 + A_2 v^2 + A_1 v + A_0 = 0
$$

系数 $A_0, \ldots, A_4$ 由 $a, b, c$ 和 $\cos\alpha, \cos\beta, \cos\gamma$ 确定（推导较繁琐，见 Grunert 等经典文献或 [p3p 博客](https://jingnanshi.com/blog/pnp_minimal.html)）。  
求该多项式的**实根**（通常最多 4 个），每个实根对应一组 $(u, v)$，进而得到 $d_1, d_2, d_3$（差一全局正尺度）。取 $d_1, d_2, d_3 > 0$（点在相机前方），最多得到 **4 组**有效的 $(d_1, d_2, d_3)$。

---

**🧭 到这里我们已经求出了相机到三个点的距离。下面最后一步：把"世界系三点"和"相机系三点"对齐，得到 $(R, \mathbf{t})$——这就是 Procrustes（3D-3D 配准），在 04 系列第 1 讲已经详细讲过。**

---

## 6. 从距离到位姿：3D–3D 配准 (3D-3D Registration / Procrustes)

> 📖 **对照 PPT**：打开 03_single_view_pose_estimation.pdf **第 44–45 页** "P3P Solutions and Ambiguity" / "3D to 3D registration (Procrustes problem)"。

对每一组 $(d_1, d_2, d_3)$，在相机系下三点坐标为 $\mathbf{p}_i = d_i \mathbf{j}_i$；在世界系下为 $\mathbf{X}_i$。问题变为：

**已知两团 3D 点的对应** $\mathbf{X}_i \leftrightarrow \mathbf{p}_i$，求**刚体变换** $(R, \mathbf{t})$ 使得 $\mathbf{p}_i = R \mathbf{X}_i + \mathbf{t}$（或最小化 $\sum_i \lVert \mathbf{p}_i - (R\mathbf{X}_i + \mathbf{t})\rVert ^2$）。这就是 **3D–3D 配准 (3D-3D Registration)**，也称 **Procrustes 问题**。

**典型解法（Kabsch / 闭式解）**：

1. **去中心**：$\bar{\mathbf{X}} = \frac{1}{3}\sum \mathbf{X}_i$，$\bar{\mathbf{p}} = \frac{1}{3}\sum \mathbf{p}_i$；令 $\tilde{\mathbf{X}}_i = \mathbf{X}_i - \bar{\mathbf{X}}$，$\tilde{\mathbf{p}}_i = \mathbf{p}_i - \bar{\mathbf{p}}$。
2. **平移**：$\mathbf{t} = \bar{\mathbf{p}} - R \bar{\mathbf{X}}$（先不管 $R$，平移由重心对齐确定）。
3. **旋转**：最小化 $\sum_i \lVert \tilde{\mathbf{p}}_i - R \tilde{\mathbf{X}}_i\rVert ^2$。令 $H = \sum_i \tilde{\mathbf{p}}_i \tilde{\mathbf{X}}_i^\top$，对 $H$ 做 SVD：$H = U \Sigma V^\top$，则 $R = U V^\top$（若 $\det(R)=-1$ 则取 $R = U \operatorname{diag}(1,1,-1) V^\top$）。

因此：**P3P 每得到一组 $(d_1, d_2, d_3)$，就用 Procrustes 算出一个 $(R, \mathbf{t})$**，最多 4 个候选位姿。

---

## 7. 用第 4 个点去歧义 (Disambiguating: use a 4th point)

> 📖 **对照 PPT**：打开 03_single_view_pose_estimation.pdf **第 46 页** "Disambiguating: use a 4th point"。

P3P 最多给出 **4 组** $(R, \mathbf{t})$。若只有 3 个点，无法从几何上区分这 4 组；若有**第 4 个点**及其 2D 观测：

- 对每组 $(R, \mathbf{t})$，把第 4 个 3D 点投影到图像，算**重投影误差 (Reprojection error)**。
- 选**重投影误差最小**的那组作为最终位姿。

因此：**4 个点 ⇒ 唯一解**（在无噪声、无外点意义下）。实践中常用 4 点做 P3P（或直接用 4 点求一组解并评估），再用 RANSAC 时也常用 4 点为一组最小子集。

---

**🧭 P3P 本身讲完了。但 P3P 最多给 4 组解，而且完全不抗外点。下面用 RANSAC 包一层，就能同时解决去歧义和外点问题。**

---

## 8. RANSAC 与完整流程（Full pipeline）

> 📖 **对照 PPT**：打开 03_single_view_pose_estimation.pdf **第 47 页** "Full pipeline: P3P Solver"，有完整的流程框图。

### 8.1 RANSAC 简要

**RANSAC**（Random Sample Consensus）：在含**外点 (Outliers)** 的数据中估计模型。

1. **随机抽最小子集**：例如每次抽 3 或 4 个 2D–3D 对应。
2. **用最小子集求模型**：对 PnP 即用 P3P（或 4 点法）求出一个或多个 $(R, \mathbf{t})$。
3. **数内点 (Inliers)**：对其余所有点算重投影误差，误差小于阈值的视为**内点**，统计内点个数。
4. **重复**多次（如根据内点比例设定迭代次数），选**内点最多**的那次对应的 $(R, \mathbf{t})$。
5. **（可选）精化**：用全部内点做非线性最小二乘（第 5 讲）或 `solvePnP`，得到最终位姿。

这样，**外点不会被用来求模型**，只有"一致"的内点决定位姿，因此鲁棒。

### 8.2 PnP 的完整 pipeline（P3P + RANSAC）

1. 输入：$N$ 对 2D–3D 对应、内参 $K$。
2. **RANSAC 循环**：  
   - 随机选 3 个（或 4 个）对应。  
   - 用 **P3P** 求 1～4 组 $(R, \mathbf{t})$；若有 4 个点可用第 4 点选一组。  
   - 对每组解，算所有 $N$ 个点的重投影误差，数内点。  
   - 记录内点最多的一组 $(R, \mathbf{t})$ 及内点集合。
3. **输出**：RANSAC 选出的 $(R, \mathbf{t})$；可选地用**全部内点**再跑一次非线性优化或 `cv2.solvePnP` 做精化。

---

## 9. 与第 5 讲的对比（Summary）

| 方面 | 第 5 讲：非线性最小二乘 | 第 6 讲：P3P + RANSAC |
|------|--------------------------|------------------------|
| **思路** | 最小化所有点重投影误差和 | 最小子集求解析解 + 内点投票 |
| **外点** | 不鲁棒，需先剔除外点或给好初值 | RANSAC 自然抗外点 |
| **初值** | 需要较好初值才能收敛 | P3P 给出解析解，可作初值 |
| **速度** | 依赖迭代次数与点数 | 每轮 P3P 很快；总时间取决于 RANSAC 轮数 |
| **精度** | 收敛后可在内点上很准 | 单次 P3P 仅用 3 点；精化阶段可再优化 |

实践中常**结合**：用 P3P + RANSAC 得到内点和初值，再在内点上用非线性最小二乘或 `solvePnP` 精化。

---

## 10. 本讲小结与"学完你能做什么"

| 内容 | 要点 |
|------|------|
| **最小解 + 外点拒绝 (Minimal solver + outlier rejection)** | 用 3 点求 P3P 候选解，用 RANSAC 按内点数选解，抗外点。 |
| **P3P** | 3 个 3D 点 + 3 条射线；求相机到三点的距离 $d_1, d_2, d_3$；最多 4 组解。 |
| **Step 1** | 像素 + $K$ → 单位射线 $\mathbf{j}_1, \mathbf{j}_2, \mathbf{j}_3$。 |
| **Step 2–3** | 余弦定理：$d_i^2 + d_j^2 - 2 d_i d_j \cos\theta = \text{边长}^2$，得到关于 $d_1, d_2, d_3$ 的方程。 |
| **Step 4** | 消元 → 关于一个变量的**四次多项式**，求实根得 $(d_1, d_2, d_3)$。 |
| **到位姿** | 相机系下 $\mathbf{p}_i = d_i \mathbf{j}_i$ ↔ 世界系 $\mathbf{X}_i$ → **3D–3D 配准 (3D-3D Registration / Procrustes)** 得 $(R, \mathbf{t})$。 |
| **第 4 点** | 在 4 组 P3P 解中选重投影误差最小的 ⇒ 唯一解。 |
| **完整流程** | 最小子集 → P3P → RANSAC 选内点 → 可选：全部内点精化。 |

**学完本讲你应该能**：

1. **口述 P3P 的 4 步**：归一化到射线 → 三角/余弦定理 → 消元得四次多项式 → 求根得距离 → Procrustes 得 $(R, \mathbf{t})$。  
2. 说明为何需要**第 4 个点**（去歧义）、为何要 **RANSAC**（抗外点）。  
3. 与第 5 讲对比：何时用非线性优化、何时用 P3P+RANSAC，以及如何组合使用。

> **口述练习**：找同学或对着镜子，不看笔记，把以下三件事各用 2–3 句话说清楚：
> 1. P3P 为什么最多有 4 组解？（四次多项式最多 4 个正实根）
> 2. RANSAC 是怎么抗外点的？（每次只抽 3 个点求解，用剩余所有点投票）
> 3. P3P + RANSAC 和第 5 讲的非线性优化如何配合？（前者给初值和内点，后者精化）

---

## 11. 自测与练习建议

1. **手写**：在已知 $\mathbf{j}_1, \mathbf{j}_2, \mathbf{j}_3$ 和边长 $a, b, c$ 时，写出三条余弦定理方程（关于 $d_1, d_2, d_3$）。  
2. **查阅**：OpenCV 中 `cv2.solvePnPRansac` 的接口；是否基于 P3P + RANSAC？  
3. **思考**：若数据中 50% 是外点，大约需要多少次 RANSAC 抽样才能以高概率抽到一次"全为内点"的最小子集？（与抽样次数公式有关。）

---

**单视图位姿估计** 6 讲至此全部完成：从几何与 DLT（第 1 讲）→ 平面单应与位姿分解（第 2 讲）→ April Tag 实战（第 3 讲）→ PnP 问题与直观（第 4 讲）→ 非线性最小二乘解 PnP（第 5 讲）→ P3P 与 RANSAC（第 6 讲）。建议结合讲义与代码（如 OpenCV `findHomography`、`solvePnP`、`solvePnPRansac`）做一次完整复习与实现。
