# 补充 B-2：SVD 的应用（伪逆、最小二乘、极分解）

**对应文献**：Strang, Linear Algebra, Section 6.3 (MATHLinearAlgebraStrangSVD.pdf)  
**本讲目标**：会用 SVD 理解最小范数最小二乘解、伪逆 $A^+$、以及极分解在“旋转+伸缩”中的含义，并与主线 DLT、Procrustes 对应。

---

## 1. 秩 1 分解（外积和）

由 $A = USV^\top$，按“列×行”相乘：

$$
A = U S V^\top = \sum_{j=1}^{r} \sigma_j \, \mathbf{u}_j \mathbf{v}_j^\top
$$

每一项 $\sigma_j \mathbf{u}_j \mathbf{v}_j^\top$ 都是**秩 1** 矩阵（列空间由 $\mathbf{u}_j$ 张成）。因此：

- **任意矩阵 = $r$ 个秩 1 矩阵的加权和**，权重为奇异值。
- 奇异值从大到小排列，所以**前几项贡献最大**。

**应用：图像压缩**。若 $A$ 是图像矩阵（如 1000×1000），保留前 $k$ 个奇异值，用

$$
A_k = \sigma_1 \mathbf{u}_1 \mathbf{v}_1^\top + \cdots + \sigma_k \mathbf{u}_k \mathbf{v}_k^\top
$$

近似 $A$。只需存 $k$ 个 $\sigma_j$ 和 $k$ 列 $U$、$k$ 列 $V$（共 $k(1+2n)$ 个数），远小于 $n^2$，实现压缩；$k$ 越大近似越好。

---

## 2. 有效秩（Effective rank）

理论上秩 $r = \operatorname{rank}(A)$ 是“独立行/列个数”，但**浮点运算**里：

- 用“主元个数”判秩会受舍入影响；
- 很小的主元该不该算成 0 不好决定。

**更稳的做法**：用 SVD。$A^\top A$ 与 $AA^\top$ 的特征值（即 $\sigma_j^2$）在数值上更可靠。根据数据精度设一个**阈值** $\tau$（如 $10^{-6}$ 或相对阈值），**有效秩** = 大于 $\tau$ 的奇异值个数。

例如矩阵 $\begin{bmatrix} \epsilon & 1 \\ 0 & 0 \end{bmatrix}$（$\epsilon$ 很小）：主元可能有两个，但有效秩为 1；小奇异值对应的方向在应用中可视为“噪声”或“缺失”。

---

## 3. 极分解（Polar decomposition）

**极分解**：任意实方阵 $A$ 可写成

$$
A = Q S
$$

其中 **$Q$ 正交**（旋转或反射），**$S$ 对称半正定**（伸缩）。若 $A$ 可逆，则 $S$ 正定。

**推导**：在 SVD $A = USV^\top$ 中插入 $V^\top V = I$：

$$
A = U S V^\top = (U V^\top) (V S V^\top) \triangleq Q \cdot S
$$

- **$Q = UV^\top$**：两个正交矩阵的积，故 $Q$ 正交。
- **$S = V S V^\top$**：$S$ 为对角非负，故 $S$ 对称半正定；若 $A$ 可逆则奇异值全正，$S$ 正定。

**几何意义**：把 $A$ 的变换拆成“**先伸缩 $S$，再旋转/反射 $Q$**”。在连续介质力学或机器人中：$S$ 对应拉伸/压缩（应变、主方向），$Q$ 对应刚体旋转，便于分析变形与能量。

**反向极分解**：也可写 $A = S' Q$（先旋转再伸缩），此时 $S' = U S U^\top$，$Q$ 仍为 $UV^\top$。

---

## 4. 最小二乘与伪逆

### 4.1 两种困难

对 $A\mathbf{x} = \mathbf{b}$（$A$ 为 $m\times n$）：

1. **行相关（超定）**：$\mathbf{b}$ 不在 $\mathcal{C}(A)$ 中，无解。用**最小二乘**：解 $A^\top A \widehat{\mathbf{x}} = A^\top \mathbf{b}$，得到残差最小的 $\widehat{\mathbf{x}}$。
2. **列相关（欠定）**：$A^\top A$ 不可逆，$A^\top A \widehat{\mathbf{x}} = A^\top \mathbf{b}$ 有无穷多解。我们从中选**范数最小**的解，记为 $\mathbf{x}^+$。

**最小范数最小二乘解** $\mathbf{x}^+$：在满足 $A^\top A \mathbf{x} = A^\top \mathbf{b}$ 的所有 $\mathbf{x}$ 中，取 $\lVert \mathbf{x}\rVert $ 最小的那个。它一定在**行空间** $\mathcal{C}(A^\top)$ 里。

### 4.2 为何最短解在行空间

任一解可写 $\widehat{\mathbf{x}} = \mathbf{x}_r + \mathbf{x}_n$，其中 $\mathbf{x}_r \in \mathcal{C}(A^\top)$，$\mathbf{x}_n \in \mathcal{N}(A)$，且 $\mathbf{x}_r \perp \mathbf{x}_n$。

- $A\mathbf{x}_n = \mathbf{0}$，故 $A^\top A \widehat{\mathbf{x}} = A^\top A \mathbf{x}_r$，即 $\mathbf{x}_r$ 也满足正规方程。
- $\lVert \widehat{\mathbf{x}}\rVert ^2 = \lVert \mathbf{x}_r\rVert ^2 + \lVert \mathbf{x}_n\rVert ^2$，要最短就取 $\mathbf{x}_n = \mathbf{0}$，所以 **$\mathbf{x}^+ = \mathbf{x}_r \in \mathcal{C}(A^\top)$**，且由正规方程唯一确定这一分量。

### 4.3 伪逆 $A^+$

存在一个矩阵 $A^+$（$n \times m$），使得对任意 $\mathbf{b}$ 都有

$$
\mathbf{x}^+ = A^+ \mathbf{b}
$$

即“最小范数最小二乘解 = 用 $A^+$ 左乘 $\mathbf{b}$”。$A^+$ 称为 **Moore–Penrose 伪逆**。

**用 SVD 写出 $A^+$**：设 $A = USV^\top$。对角矩阵 $S$ 的伪逆 $S^+$ 为：非零对角元取倒数、零保持不变，且 $S^+$ 的尺寸为 $n \times m$（与 $S$ 的 $m \times n$ 互转）。则

$$
\boxed{\; A^+ = V S^+ U^\top \;}
$$

验证思路：$\mathbf{x}^+ = A^+ \mathbf{b} = V S^+ U^\top \mathbf{b}$。令 $\mathbf{y} = U^\top \mathbf{b}$，则 $\mathbf{x}^+ = V S^+ \mathbf{y}$。$S^+ \mathbf{y}$ 只保留前 $r$ 个分量（为 $y_j/\sigma_j$），再乘 $V$ 得到的是行空间中的向量；且可证该向量满足 $A^\top A \mathbf{x}^+ = A^\top \mathbf{b}$ 且范数最小。

**最小二乘解公式**：

$$
\mathbf{x}^+ = A^+ \mathbf{b} = V S^+ U^\top \mathbf{b}
$$

当 $A$ 列满秩时，$A^+ = (A^\top A)^{-1} A^\top$；当 $A$ 可逆时，$A^+ = A^{-1}$。

---

## 5. 用 SVD 推导 $\mathbf{x}^+$ 为何是最优

要最小化 $\lVert A\mathbf{x} - \mathbf{b}\rVert ^2$。令 $\mathbf{y} = V^\top \mathbf{x}$（则 $\mathbf{x} = V\mathbf{y}$，$\lVert \mathbf{x}\rVert  = \lVert \mathbf{y}\rVert $），有

$$
\|A\mathbf{x} - \mathbf{b}\| = \|USV^\top \mathbf{x} - \mathbf{b}\| = \|S\mathbf{y} - U^\top \mathbf{b}\|
$$

（因 $U$ 正交不改变范数。）记 $\mathbf{c} = U^\top \mathbf{b}$，则问题变为

$$
\min_{\mathbf{y}} \|S\mathbf{y} - \mathbf{c}\|
$$

$S$ 对角：前 $r$ 个分量应取 $y_j = c_j/\sigma_j$（使 $(S\mathbf{y})_j = c_j$）；$j > r$ 时 $(S\mathbf{y})_j = 0$，无法减小 $\lvert c_j\rvert $，因此残差最小值由前 $r$ 个分量决定。在**所有**使残差最小的 $\mathbf{y}$ 中，要再让 $\lVert \mathbf{x}\rVert  = \lVert \mathbf{y}\rVert $ 最小，应取 $y_{r+1} = \cdots = y_n = 0$，即

$$
\mathbf{y}^+ = (c_1/\sigma_1, \ldots, c_r/\sigma_r, 0, \ldots, 0)^\top = S^+ \mathbf{c} = S^+ U^\top \mathbf{b}
$$

故

$$
\mathbf{x}^+ = V \mathbf{y}^+ = V S^+ U^\top \mathbf{b} = A^+ \mathbf{b}
$$

且 $\mathbf{x}^+$ 在 $V$ 的前 $r$ 列张成的空间里，即行空间，故为最小范数解。

---

## 6. 与主线的联系

### 6.1 DLT（第 1 讲）

DLT 求 $\min_{\lVert \mathbf{h}\rVert =1} \lVert A\mathbf{h}\rVert ^2$，齐次方程 $A\mathbf{h} = \mathbf{0}$。解为 $A^\top A$ 的**最小特征值**对应的单位特征向量，即 SVD 中 **$V$ 的最后一列**（对应最小 $\sigma$）。所以“对 $A$ 做 SVD，取 $V$ 最后一列”就是在求 DLT 的解。

### 6.2 Procrustes / 3D–3D 配准（第 6 讲）

已知两团 3D 点对应 $\mathbf{p}_i \leftrightarrow \mathbf{X}_i$，求旋转 $R$ 使 $\sum \lVert \mathbf{p}_i - R\mathbf{X}_i\rVert ^2$ 最小。去中心后化为 $\min_R \lVert P - R X\rVert _F^2$，其中 $P,X$ 为点阵。最优旋转为

$$
R^* = U V^\top
$$

其中 $H = P X^\top = U \Sigma V^\top$（SVD）。这里 **$R^* = UV^\top$ 正是 $H$ 的“极分解”中的正交因子**（若 $H$ 可逆且无反射则 $R^* \in SO(3)$，否则需对 $V$ 的一列取反保证 $\det R^* = 1$）。所以 Procrustes 的闭式解 = 用 SVD 取正交部分。

### 6.3 条件数

解 $A\mathbf{x} = \mathbf{b}$ 或最小二乘时，误差放大与 $\kappa(A) = \sigma_{\max}/\sigma_{\min}$ 相关。DLT 中若最小奇异值非常小，说明问题接近奇异，解对噪声敏感；可配合正则化或更多观测改善。

---

## 7. 本讲小结与“学完你能做什么”

| 内容 | 要点 |
|------|------|
| **秩 1 分解** | $A = \sum_{j=1}^r \sigma_j \mathbf{u}_j \mathbf{v}_j^\top$；前 $k$ 项可做低秩近似与压缩。 |
| **有效秩** | 用奇异值相对阈值判断“有效秩”，比主元更稳。 |
| **极分解** | $A = QS$，$Q = UV^\top$ 正交，$S = V\Sigma V^\top$ 对称半正定；几何上“伸缩 + 旋转”。 |
| **伪逆** | $A^+ = V S^+ U^\top$，$S^+$ 对角元为 $1/\sigma_j$（$\sigma_j>0$），其余为 0。 |
| **最小范数最小二乘** | $\mathbf{x}^+ = A^+ \mathbf{b}$；在行空间中且满足正规方程。 |
| **主线** | DLT 取 $V$ 最后一列；Procrustes 取 $R = UV^\top$；条件数影响稳定性。 |

**学完本讲你应该能**：

1. 用 SVD 写出 $\mathbf{x}^+ = A^+ \mathbf{b}$，并说明为何 $\mathbf{x}^+$ 在行空间且范数最小。  
2. 写出极分解 $A = QS$ 并说明 $Q$、$S$ 在几何/物理上的含义。  
3. 把 DLT（取 $V$ 最后一列）、Procrustes（$R = UV^\top$）、条件数与本讲概念对应起来。

---

## 8. 自测与练习建议

1. **手算**：对 $A = \begin{bmatrix} 1 & 2 & 2 \end{bmatrix}$（1×3），求 SVD 与 $A^+$，并验证 $A^+ \mathbf{b}$ 是 $Ax=b$ 的最小范数解（例如 $\mathbf{b}=[18]$）。  
2. **编程**：用 `np.linalg.pinv(A)` 与自写 $V S^+ U^\top$ 对比；对随机 $A$ 和 $\mathbf{b}$ 验证 $\mathbf{x}^+ = A^+ \mathbf{b}$ 满足 $A^\top A \mathbf{x}^+ = A^\top \mathbf{b}$ 且 $\lVert \mathbf{x}^+\rVert  \le \lVert \widehat{\mathbf{x}}\rVert $。  
3. **思考**：极分解 $A=QS$ 中，若 $A$ 已是正交矩阵，$S$ 是什么？若 $A$ 是纯伸缩（对角正定），$Q$ 是什么？

---

**Part B（SVD 补充）** 两讲至此结束。可继续学 **Part A（P3P 深度）** 或回顾主线第 1、6 讲以巩固 DLT 与 Procrustes 的 SVD 用法。
