# 补充 B-1：SVD 的定义与基本性质

**对应文献**：Strang, Linear Algebra, Section 6.3 Singular Value Decomposition (MATHLinearAlgebraStrangSVD.pdf)  
**本讲目标**：掌握 $A=USV^\top$ 的构成、与 $A^\top A$ / $AA^\top$ 的关系、四子空间与 $A\mathbf{v}_j = \sigma_j \mathbf{u}_j$。

---

## 1. 为什么需要 SVD

在线性代数里，我们熟悉：

- **对称矩阵** $A = A^\top$：可正交对角化 $A = Q \Lambda Q^\top$，$Q$ 正交、$\Lambda$ 对角（特征值）。
- **一般方阵**：特征向量不一定正交，甚至不一定有足够多的实特征向量。
- **长方矩阵** $m \times n$：连“特征值”都没有统一定义（$A\mathbf{x} = \lambda \mathbf{x}$ 要求 $\mathbf{x} \in \mathbb{R}^n$ 且 $A\mathbf{x} \in \mathbb{R}^n$，只有方阵才一致）。

SVD 的做法是：**不要求左右用同一组正交基**。左边用 $m \times m$ 正交矩阵 $U$，右边用 $n \times n$ 正交矩阵 $V$，中间用**对角**矩阵 $S$（可矩形），使得

$$
A = U S V^\top
$$

这样**任意** $m \times n$ 矩阵都有这种分解；对角元是**非负**的，称为**奇异值**，来自 $A^\top A$ 或 $AA^\top$ 的特征值的平方根。

---

## 2. SVD 的正式陈述

**奇异值分解（Singular Value Decomposition）**：

任意 $m \times n$ 矩阵 $A$ 可分解为

$$
A = U S V^\top = (\text{正交}) \times (\text{对角}) \times (\text{正交})
$$

其中：

- **$U$**：$m \times m$ 正交矩阵（$U^\top U = I_m$），列向量称为**左奇异向量**。
- **$V$**：$n \times n$ 正交矩阵（$V^\top V = I_n$），列向量称为**右奇异向量**。
- **$S$**：$m \times n$ 的“对角”矩阵：只有主对角线有非零元，且**非负**。这些非零元记为 $\sigma_1 \ge \sigma_2 \ge \cdots \ge \sigma_r > 0$，称为**奇异值**；$r = \operatorname{rank}(A)$，$S$ 的其余位置为 0。

通常约定奇异值从大到小排列。若 $r < \min(m,n)$，则 $S$ 的右下角会有一块零。

---

## 3. 与 $A^\top A$ 和 $AA^\top$ 的关系

对**任意** $A$，$A^\top A$ 和 $AA^\top$ 都是**对称半正定**的，且

$$
\operatorname{rank}(A^\top A) = \operatorname{rank}(AA^\top) = \operatorname{rank}(A) = r.
$$

把 $A = USV^\top$ 代入：

$$
A^\top A = (USV^\top)^\top (USV^\top) = V S^\top U^\top U S V^\top = V (S^\top S) V^\top.
$$

这里 $S^\top S$ 是 $n \times n$ 对角矩阵，前 $r$ 个对角元为 $\sigma_1^2, \ldots, \sigma_r^2$，其余为 0。因此：

- **$V$ 的列**是 **$A^\top A$** 的**单位特征向量**；
- **$A^\top A$ 的非零特征值**为 $\sigma_1^2, \ldots, \sigma_r^2$。

同理，

$$
AA^\top = U S V^\top V S^\top U^\top = U (S S^\top) U^\top.
$$

$S S^\top$ 是 $m \times m$ 对角矩阵，前 $r$ 个对角元也是 $\sigma_1^2, \ldots, \sigma_r^2$。因此：

- **$U$ 的列**是 **$AA^\top$** 的**单位特征向量**；
- **$AA^\top$ 的非零特征值**同样是 $\sigma_1^2, \ldots, \sigma_r^2$。

**总结**：$U$ 来自 $AA^\top$，$V$ 来自 $A^\top A$，**奇异值** $\sigma_j$ 是这两个矩阵**共有**的非零特征值的**平方根**。

---

## 4. 关键关系：$A\mathbf{v}_j = \sigma_j \mathbf{u}_j$

从 $A = USV^\top$ 可得 $AV = US$（因为 $V^\top V = I$）。按列看：

$$
A \mathbf{v}_j = \sigma_j \mathbf{u}_j, \qquad j = 1, \ldots, r
$$

（当 $j > r$ 时 $S$ 第 $j$ 列为零，故 $A\mathbf{v}_j = \mathbf{0}$。）

**推导**：设 $A^\top A \mathbf{v}_j = \sigma_j^2 \mathbf{v}_j$（$\mathbf{v}_j$ 是 $A^\top A$ 的单位特征向量）。两边左乘 $A$：

$$
AA^\top (A\mathbf{v}_j) = \sigma_j^2 (A\mathbf{v}_j)
$$

所以 **$A\mathbf{v}_j$ 是 $AA^\top$ 的属于特征值 $\sigma_j^2$ 的特征向量**。再算长度：

$$
\|A\mathbf{v}_j\|^2 = \mathbf{v}_j^\top A^\top A \mathbf{v}_j = \mathbf{v}_j^\top (\sigma_j^2 \mathbf{v}_j) = \sigma_j^2 \|\mathbf{v}_j\|^2 = \sigma_j^2
$$

故 $\lVert A\mathbf{v}_j\rVert  = \sigma_j$（$\sigma_j \ge 0$）。令

$$
\mathbf{u}_j = \frac{A\mathbf{v}_j}{\sigma_j}
$$

则 $\mathbf{u}_j$ 是单位向量，且是 $AA^\top$ 的属于 $\sigma_j^2$ 的单位特征向量，即 $U$ 的第 $j$ 列。于是

$$
A\mathbf{v}_j = \sigma_j \mathbf{u}_j.
$$

这就是 SVD 最常用的一条性质：**$A$ 把 $V$ 的第 $j$ 列映成 $U$ 的第 $j$ 列的 $\sigma_j$ 倍**。

---

## 5. 四子空间与 U、V 的列

$U$ 和 $V$ 不仅给出正交基，而且**恰好**对应 $A$ 的四个基本子空间：

| 子空间 | 由谁张成 | 维数 |
|--------|----------|------|
| **列空间** $\mathcal{C}(A)$ | $U$ 的**前 $r$ 列** $\mathbf{u}_1, \ldots, \mathbf{u}_r$ | $r$ |
| **左零空间** $\mathcal{N}(A^\top)$ | $U$ 的**后 $m-r$ 列** $\mathbf{u}_{r+1}, \ldots, \mathbf{u}_m$ | $m-r$ |
| **行空间** $\mathcal{C}(A^\top)$ | $V$ 的**前 $r$ 列** $\mathbf{v}_1, \ldots, \mathbf{v}_r$ | $r$ |
| **零空间** $\mathcal{N}(A)$ | $V$ 的**后 $n-r$ 列** $\mathbf{v}_{r+1}, \ldots, \mathbf{v}_n$ | $n-r$ |

原因简述：$A\mathbf{v}_j = \sigma_j \mathbf{u}_j$（$j \le r$）说明 $\mathbf{u}_j \in \mathcal{C}(A)$；$A\mathbf{v}_j = \mathbf{0}$（$j>r$）说明 $\mathbf{v}_j \in \mathcal{N}(A)$。再结合 $U,V$ 正交，维数一致，即得上述对应。

---

## 6. 数值稳定性与条件数

**正交矩阵不改变长度**：对任意 $\mathbf{x}$，$\lVert U\mathbf{x}\rVert  = \lVert \mathbf{x}\rVert $（因为 $\lVert U\mathbf{x}\rVert ^2 = \mathbf{x}^\top U^\top U \mathbf{x} = \mathbf{x}^\top \mathbf{x}$）。所以用 $U$、$V$ 做乘法不会放大误差，**数值上很安全**。

**对角矩阵 $S$** 会放大或缩小：乘以很大的 $\sigma$ 可能溢出，除以很小的 $\sigma$ 会放大舍入误差。**小奇异值**对应“几乎不可逆”的方向，是数值不稳定的来源。

当 $A$ 为 $n \times n$ 可逆时，定义**条件数**（condition number）：

$$
\kappa(A) = \frac{\sigma_{\max}}{\sigma_{\min}} = \frac{\sigma_1}{\sigma_r}
$$

条件数越大，解 $A\mathbf{x} = \mathbf{b}$ 时对 $\mathbf{b}$ 的扰动越敏感。SVD 直接给出 $\sigma_1, \ldots, \sigma_r$，因此能**显式看到**哪些方向敏感、矩阵是否“接近奇异”。

---

## 7. 与主线 DLT 的联系

主线第 1 讲中，DLT 求单应时得到**齐次线性方程组** $A\mathbf{h} = \mathbf{0}$，其中 $A$ 是 $2N \times 9$。我们求的是

$$
\min_{\|\mathbf{h}\|=1} \|A\mathbf{h}\|^2
$$

即让 $\lVert A\mathbf{h}\rVert $ 在单位向量上最小。对 $A$ 做 SVD：$A = USV^\top$，则

$$
\|A\mathbf{h}\|^2 = \|USV^\top \mathbf{h}\|^2 = \|SV^\top \mathbf{h}\|^2
$$

（因为 $U$ 正交不改变范数。）记 $\mathbf{y} = V^\top \mathbf{h}$，则 $\lVert \mathbf{y}\rVert  = \lVert \mathbf{h}\rVert  = 1$，且

$$
\|SV^\top \mathbf{h}\|^2 = \|S\mathbf{y}\|^2 = \sum_j \sigma_j^2 y_j^2
$$

在 $\sum y_j^2 = 1$ 下，要使上式最小，应把“权重”放在**最小奇异值**对应的分量上。若 $\sigma_9$ 最小且唯一，则最优 $\mathbf{y}$ 在第九个分量为 ±1、其余为 0，即 $\mathbf{h}$ 取为 **$V$ 的最后一列**（或其负）。所以 **DLT 中“对 $A$ 做 SVD，取 $V$ 的最后一列”就是在求最小奇异值对应的右奇异向量**。

---

## 8. 小例子（可选手算）

**例**：$A = \begin{bmatrix} 1 & 4 \\ 2 & 8 \end{bmatrix}$。易知 $\operatorname{rank}(A)=1$。

- $A^\top A = \begin{bmatrix} 5 & 20 \\ 20 & 80 \end{bmatrix}$，特征值 $\lambda$ 满足 $\det(A^\top A - \lambda I) = 0$，得 $\lambda_1 = 85,\, \lambda_2 = 0$。故 $\sigma_1 = \sqrt{85}$，$\sigma_2 = 0$。
- 属于 $\lambda_1 = 85$ 的单位特征向量可作为 $\mathbf{v}_1$（例如归一化 $(1,4)^\top$）；属于 0 的为 $\mathbf{v}_2 \perp \mathbf{v}_1$。
- $\mathbf{u}_1 = A\mathbf{v}_1 / \sigma_1$ 为 $AA^\top$ 的单位特征向量；$U$ 的第二列可取与 $\mathbf{u}_1$ 正交的任意单位向量。
- 于是 $A = U S V^\top$，$S = \operatorname{diag}(\sqrt{85},\, 0)$（或 $2 \times 2$ 时 $S$ 第二列为 0）。

通过这个例子可以体会：**奇异值来自 $A^\top A$，$V$ 是 $A^\top A$ 的特征向量，$U$ 由 $A\mathbf{v}_j/\sigma_j$ 得到**。

---

## 9. 本讲小结与“学完你能做什么”

| 内容 | 要点 |
|------|------|
| **SVD 陈述** | $A = USV^\top$，$U$、$V$ 正交，$S$ 对角、非负奇异值 $\sigma_1 \ge \cdots \ge \sigma_r > 0$。 |
| **与特征值** | $U$ 的列 = $AA^\top$ 的单位特征向量；$V$ 的列 = $A^\top A$ 的单位特征向量；$\sigma_j^2$ = 两者的非零特征值。 |
| **关键关系** | $A\mathbf{v}_j = \sigma_j \mathbf{u}_j$；由 $A^\top A \mathbf{v}_j = \sigma_j^2 \mathbf{v}_j$ 推出 $A\mathbf{v}_j$ 是 $AA^\top$ 的特征向量且长度为 $\sigma_j$。 |
| **四子空间** | $U$ 前 $r$ 列 → 列空间，后 $m-r$ 列 → 左零空间；$V$ 前 $r$ 列 → 行空间，后 $n-r$ 列 → 零空间。 |
| **条件数** | 可逆时 $\kappa(A) = \sigma_{\max}/\sigma_{\min}$；小奇异值对应不稳定方向。 |
| **DLT** | $\min_{\lVert \mathbf{h}\rVert =1}\lVert A\mathbf{h}\rVert ^2$ 的解为 $V$ 中对应**最小奇异值**的那一列。 |

**学完本讲你应该能**：

1. 写出任意矩阵 SVD 的形式，并说明 $U,\,S,\,V$ 的来源（$AA^\top$、$A^\top A$、奇异值）。  
2. 从 $A^\top A \mathbf{v}_j = \sigma_j^2 \mathbf{v}_j$ 推出 $A\mathbf{v}_j = \sigma_j \mathbf{u}_j$ 以及 $\mathbf{u}_j$ 是 $AA^\top$ 的单位特征向量。  
3. 对给定小矩阵手算或编程求 SVD（先算 $A^\top A$ 的特征值与 $V$，再算 $\mathbf{u}_j = A\mathbf{v}_j/\sigma_j$）。  
4. 解释主线 DLT 中“取 $V$ 最后一列”对应最小奇异值方向。

---

## 10. 自测与练习建议

1. **手算**：对 $A = \begin{bmatrix} 1 & 2 \\ 2 & 4 \end{bmatrix}$ 求 $A^\top A$ 的特征值与单位特征向量，写出 $\sigma_1$ 和 $V$ 的第一列，并验证 $A\mathbf{v}_1 = \sigma_1 \mathbf{u}_1$ 给出 $U$ 的第一列。  
2. **编程**：用 NumPy 的 `np.linalg.svd(A)` 对任意小矩阵求 $U, S, V^\top$，并验证 $A = U @ np.diag(S) @ Vt$ 以及 $\lVert A\mathbf{v}_j - \sigma_j \mathbf{u}_j\rVert $ 接近 0。  
3. **思考**：若 $A$ 的最后一列是前几列的线性组合，$S$ 中会有几个零奇异值？$V$ 的哪几列张成零空间？

完成以上后，可以进入 **补充 B-2：SVD 的应用（伪逆、最小二乘、极分解）**。
