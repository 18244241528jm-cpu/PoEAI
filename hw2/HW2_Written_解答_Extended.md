# HW2 Written 解答 · Extended 讲义版（零基础友好）

> 本版面向**零基础**同学，从最基础的概念讲起，每步都有解释，适合边做边学。

---

## 零基础必读：做题前需要知道的

### 1. 齐次坐标（Homogeneous Coordinates）

**普通坐标**：2D 点用 $(x, y)$，3D 点用 $(x, y, z)$。

**齐次坐标**：多写一维，2D 写成 $(x, y, 1)$，3D 写成 $(x, y, z, 1)$。好处是可以把「平移」也写成矩阵乘法。

**重要**：$(x, y, 1)$ 和 $(2x, 2y, 2)$ 表示**同一点**（差一个非零倍数等价）。要变回普通坐标，前两维除以第三维：$(u, v, w) \to (u/w, v/w)$。

---

### 2. 针孔相机模型（一句话）

3D 点 $(x, y, z)$ 投影到图像：$u = f_x \cdot x/z + c_x$，$v = f_y \cdot y/z + c_y$。$f_x, f_y$ 是焦距，$c_x, c_y$ 是主点。

---

### 3. 旋转矩阵（绕轴旋转）

绕 **y 轴**转 $\theta$（右手系，拇指朝 y 正方向）：
$$
R_y(\theta) = \begin{pmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{pmatrix}
$$
绕 **x 轴**转 $\theta$：
$$
R_x(\theta) = \begin{pmatrix} 1 & 0 & 0 \\ 0 & \cos\theta & -\sin\theta \\ 0 & \sin\theta & \cos\theta \end{pmatrix}
$$
记忆：绕哪轴转，哪一行/列是 $(0,0,0)$ 和 1，其余是 $\cos, \sin$。

---

### 4. 标定（Calibrated）是什么意思？

**已标定** = 已知相机内参 $K$，可以把像素坐标转换成「归一化坐标」（相当于 $K=I$ 时的坐标）。此时用**本质矩阵 $E$**，不需要基础矩阵 $F$。

---

# 1. Epipolar Constraint (40 pts)

## 1.1 对极线系数 A, B, C（10 pts）

### 一、这道题在问什么？（用大白话）

有两个相机拍了同一场景。已知它们的相对位姿（谁相对谁怎么转、怎么移），以及**图像 2**中某个点的坐标 $(x_2, y_2)$。问：这个点在**图像 1**中的匹配点，会落在哪条直线上？这条直线的方程是 $Ax_1 + By_1 + C = 0$，求 $A, B, C$。

---

### 二、对极几何（Epipolar Geometry）入门

**直觉**：图 2 中的一个点，对应图 1 中的一条**直线**，而不是一个点。因为图 2 的一个像素可以对应 3D 空间中的一整条射线，这条射线投影到图 1 就是一条线。

**对极线**：图 2 中点 $p_2$ 在图 1 中对应的那条直线，叫「图 1 中的对极线」。

**公式**：对极线 = $E^T \cdot p_2$。结果是一个 3 维向量 $(A, B, C)^T$，对应直线方程 $Ax + By + C = 0$。

**易混**：$E^T p_2$ 是「图 2 的点 → 图 1 的对极线」；$E p_1$ 是「图 1 的点 → 图 2 的对极线」。方向别反。

---

### 三、反对称矩阵（Skew-Symmetric Matrix）

向量 $\mathbf{t} = (t_x, t_y, t_z)^T$ 与另一向量 $\mathbf{v}$ 的**叉积** $\mathbf{t} \times \mathbf{v}$，可以写成矩阵乘向量：$\mathbf{t} \times \mathbf{v} = [\mathbf{t}]_\times \mathbf{v}$。

$[\mathbf{t}]_\times$ 的形式（直接背）：
$$
[\mathbf{t}]_\times = \begin{pmatrix} 0 & -t_z & t_y \\ t_z & 0 & -t_x \\ -t_y & t_x & 0 \end{pmatrix}
$$
主对角线全是 0；右上三角是负的 $t$ 分量；左下三角是正的 $t$ 分量。

---

### 四、题目给的量

- $T_x = (1, 0, 0)^T$：平移向量
- $\hat{T}_x$：$T_x$ 的反对称矩阵
- $R_y(\theta)$：绕 y 轴转 $\theta$ 的旋转矩阵
- $E = \hat{T}_x R_y(\theta)$：本质矩阵

---

### 五、一步步算

**第 1 步**：写出 $\hat{T}_x$。把 $t_x=1, t_y=0, t_z=0$ 代入上面的公式：
$$
\hat{T}_x = \begin{pmatrix} 0 & 0 & 0 \\ 0 & 0 & -1 \\ 0 & 1 & 0 \end{pmatrix}
$$

**第 2 步**：写出 $R_y(\theta)$（见零基础必读）：
$$
R_y(\theta) = \begin{pmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{pmatrix}
$$

**第 3 步**：算 $E = \hat{T}_x R_y(\theta)$。矩阵乘法，一行一行算：
- 第 1 行：$(0,0,0)$ 乘任意向量都得 0，所以第 1 行是 $(0, 0, 0)$
- 第 2 行：$(0, 0, -1)$ 点乘 $R$ 的每一列 → $(0, 0, -1) \cdot (\cos\theta, 0, -\sin\theta) = \sin\theta$，类似得 $(0, 0, -1) \cdot (0,1,0)=0$，$(0,0,-1)\cdot(\sin\theta,0,\cos\theta)=-\cos\theta$，故第 2 行是 $(\sin\theta, 0, -\cos\theta)$
- 第 3 行：$(0, 1, 0)$ 点乘 $R$ 的每列 → $(0, 1, 0)$

所以
$$
E = \begin{pmatrix} 0 & 0 & 0 \\ \sin\theta & 0 & -\cos\theta \\ 0 & 1 & 0 \end{pmatrix}
$$

**第 4 步**：算 $E^T$（转置：行变列）：
$$
E^T = \begin{pmatrix} 0 & \sin\theta & 0 \\ 0 & 0 & 1 \\ 0 & -\cos\theta & 0 \end{pmatrix}
$$

**第 5 步**：对极线 $\mathbf{l}_1 = E^T p_2$。$p_2 = (x_2, y_2, 1)^T$（齐次坐标）：
$$
\mathbf{l}_1 = E^T \begin{pmatrix} x_2 \\ y_2 \\ 1 \end{pmatrix} = \begin{pmatrix} 0\cdot x_2 + \sin\theta \cdot y_2 + 0 \cdot 1 \\ 0 + 0 + 1 \cdot 1 \\ 0 - \cos\theta \cdot y_2 + 0 \end{pmatrix} = \begin{pmatrix} y_2 \sin\theta \\ 1 \\ -y_2 \cos\theta \end{pmatrix}
$$

**第 6 步**：直线方程 $Ax_1 + By_1 + C = 0$ 对应 $\mathbf{l}_1 = (A, B, C)^T$，所以：
$$
A = y_2 \sin\theta,\quad B = 1,\quad C = -y_2 \cos\theta
$$

---

### 六、最终答案

$$
\boxed{A = y_2 \sin\theta,\quad B = 1,\quad C = -y_2 \cos\theta}
$$

---

## 1.2 用世界坐标系表示本质矩阵（10 pts）

### 一、这道题在问什么？

两个相机的位置和朝向都在**世界坐标系**下给出（光心 $c_1, c_2$，旋转 $^wR_1, ^wR_2$）。要把本质矩阵 $E$ 用这些量写出来。

---

### 二、符号含义（零基础版）

- **世界坐标系**：一个固定的 3D 坐标系，比如房间的角落。
- **$c_1, c_2$**：相机 1、相机 2 的光心在世界系下的坐标，是 3D 向量。
- **$^wR_1, ^wR_2$**：从世界系到相机 1、相机 2 的旋转。意思是：世界系中的一个向量 $\mathbf{v}$，在相机 $i$ 的坐标系下看，变成 $^wR_i \mathbf{v}$。
- **$\mathbf{X}_w$**：某个 3D 点在世界系下的坐标。
- **$\mathbf{X}_1, \mathbf{X}_2$**：同一个 3D 点在相机 1、相机 2 坐标系下的坐标。

**变换关系**：点 $\mathbf{X}_w$ 在相机 $i$ 系下 = 先把 $\mathbf{X}_w$ 平移到以 $c_i$ 为原点，再旋转：$\mathbf{X}_i = {}^wR_i (\mathbf{X}_w - c_i)$。

---

### 三、目标：写出相机 1 到相机 2 的相对位姿

标准对极几何用的是「相机 2 相对于相机 1」：$\mathbf{X}_2 = R \mathbf{X}_1 + \mathbf{t}$。我们要从 $c_1, c_2, {}^wR_1, {}^wR_2$ 推出 $R$ 和 $\mathbf{t}$。

**推导**：
- $\mathbf{X}_1 = {}^wR_1 (\mathbf{X}_w - c_1)$ → 反解：$\mathbf{X}_w = {}^wR_1^T \mathbf{X}_1 + c_1$
- $\mathbf{X}_2 = {}^wR_2 (\mathbf{X}_w - c_2) = {}^wR_2 \bigl({}^wR_1^T \mathbf{X}_1 + c_1 - c_2\bigr) = {}^wR_2 {}^wR_1^T \mathbf{X}_1 + {}^wR_2 (c_1 - c_2)$

所以：
$$
R = {}^wR_2 {}^wR_1^T,\qquad \mathbf{t} = {}^wR_2 (c_1 - c_2)
$$
$\mathbf{t}$ 的几何意义：从相机 2 指向相机 1 的向量，在相机 2 系下的表示。

---

### 四、本质矩阵 $E = [\mathbf{t}]_\times R$

代入 $\mathbf{t} = {}^wR_2 (c_1 - c_2)$：
$$
E = \bigl[{}^wR_2 (c_1 - c_2)\bigr]_\times \cdot {}^wR_2 {}^wR_1^T
$$

**恒等式**：$[R\mathbf{v}]_\times = R [\mathbf{v}]_\times R^T$。令 $\mathbf{v} = c_1 - c_2$：
$$
\bigl[{}^wR_2 \mathbf{v}\bigr]_\times = {}^wR_2 [\mathbf{v}]_\times {}^wR_2^T
$$

代入：
$$
E = {}^wR_2 [c_1 - c_2]_\times {}^wR_2^T \cdot {}^wR_2 {}^wR_1^T = {}^wR_2 [c_1 - c_2]_\times {}^wR_1^T
$$
（因为 ${}^wR_2^T {}^wR_2 = I$）

---

### 五、最终答案

$$
\boxed{E = {}^wR_2 [c_1 - c_2]_\times {}^wR_1^T}
$$

---

## 1.3 Twisted Pair 歧义（10 pts）

### 一、这道题在问什么？

从本质矩阵 $E$ 可以分解出多组 $(T, R)$。题目要证明：如果 $(T, R)$ 是一组解，那么 $(T, SR)$ 也是解，其中 $S$ 是绕 $T$ 轴旋转 180° 的旋转。

---

### 二、直观理解

想象两个相机，平移方向是 $T$。把相机 2 绕 $T$ 轴转 180°，相当于「翻个面」。对极几何（谁和谁共面）不变，所以 $E$ 的约束仍然满足。

---

### 三、Rodrigues 公式（绕轴旋转）

绕**单位向量** $\mathbf{n}$ 旋转 $\theta$ 的矩阵：
$$
R = I + \sin\theta [\mathbf{n}]_\times + (1 - \cos\theta) [\mathbf{n}]_\times^2
$$

**$\theta = 180°$ 时**：$\sin\pi = 0$，$\cos\pi = -1$，所以
$$
S = I + 0 + 2 [\mathbf{n}]_\times^2 = I + 2 [\mathbf{n}]_\times^2
$$

对单位向量 $\mathbf{n}$，有 $[\mathbf{n}]_\times^2 = \mathbf{n}\mathbf{n}^T - I$（可查书），故
$$
S = I + 2(\mathbf{n}\mathbf{n}^T - I) = 2\mathbf{n}\mathbf{n}^T - I
$$

这里 $\mathbf{n} = T / \|T\|$。

---

### 四、证明 $[T]_\times S = -[T]_\times$

对任意向量 $\mathbf{v}$：
$$
S\mathbf{v} = (2\mathbf{n}\mathbf{n}^T - I)\mathbf{v} = 2(\mathbf{n} \cdot \mathbf{v}) \mathbf{n} - \mathbf{v}
$$

于是
$$
[T]_\times (S\mathbf{v}) = T \times \bigl(2(\mathbf{n} \cdot \mathbf{v}) \mathbf{n} - \mathbf{v}\bigr) = 2(\mathbf{n} \cdot \mathbf{v})(T \times \mathbf{n}) - T \times \mathbf{v}
$$

因为 $\mathbf{n} \parallel T$（$\mathbf{n}$ 是 $T$ 的单位化），所以 $T \times \mathbf{n} = \mathbf{0}$。故
$$
[T]_\times (S\mathbf{v}) = - T \times \mathbf{v} = - [T]_\times \mathbf{v}
$$

对所有 $\mathbf{v}$ 成立，所以 $[T]_\times S = -[T]_\times$。

---

### 五、验证 $(T, SR)$ 是解

$$
[T]_\times (SR) = ([T]_\times S) R = (-[T]_\times) R = -[T]_\times R = -E
$$

对极约束是 $p_2^T E p_1 = 0$。若 $E$ 满足，则 $-E$ 也满足（两边乘 -1 仍是 0）。所以 $(T, SR)$ 对应的本质矩阵 $-E$ 同样满足对极约束，故 $(T, SR)$ 是解。

---

## 1.4 两光轴相交的充要条件（10 pts）

### 一、光轴是什么？

**光轴**：相机「朝前看」的方向。通常取为 z 轴正方向，即 $(0, 0, 1)^T$。

- 相机 1 光轴：$\mathbf{e}_3 = (0, 0, 1)^T$
- 相机 2 光轴：在相机 1 系下，是 $R$ 的第三列 $\mathbf{r}_3$（因为 $R$ 把相机 2 的 z 轴变到相机 1 系）

---

### 二、两射线相交的条件

- 射线 1：过原点，方向 $\mathbf{e}_3$
- 射线 2：过点 $T$（相机 2 光心在相机 1 系下的位置），方向 $\mathbf{r}_3$

**两直线相交** ⟺ 它们共面 ⟺ 向量 $T$ 在 $\mathbf{e}_3$ 和 $\mathbf{r}_3$ 张成的平面内。

**共面的数学条件**：$T$ 与法向 $\mathbf{e}_3 \times \mathbf{r}_3$ 垂直，即
$$
T \cdot (\mathbf{e}_3 \times \mathbf{r}_3) = 0
$$

---

### 三、算出具体形式

$$
\mathbf{e}_3 \times \mathbf{r}_3 = (0, 0, 1) \times (r_{13}, r_{23}, r_{33}) = (-r_{23}, r_{13}, 0)
$$

$$
T \cdot (\mathbf{e}_3 \times \mathbf{r}_3) = T_x(-r_{23}) + T_y(r_{13}) + T_z \cdot 0 = -T_x r_{23} + T_y r_{13} = 0
$$

即 $T_x r_{23} = T_y r_{13}$。

**退化**：若 $\mathbf{r}_3 \parallel \mathbf{e}_3$（即 $r_{13} = r_{23} = 0$），则 $\mathbf{e}_3 \times \mathbf{r}_3 = \mathbf{0}$，上式恒成立。此时两光轴平行，相交仅当 $T$ 也在光轴上，即 $T_x = T_y = 0$。

---

### 四、最终答案

- **一般**：$T_x r_{23} = T_y r_{13}$
- **退化**：$T_x = T_y = 0$

---

# 2. Homography from Pose (20 pts)

## 2.1 绕 x 轴旋转 45° 的单应（10 pts）

### 一、这道题在问什么？

相机在原点，先不转，拍一张图；然后绕自己的 x 轴转 45°，再拍一张。问：旋转前某像素，旋转后会落在哪个像素？这个映射是一个 3×3 矩阵 $H$。

---

### 二、像素与射线的对应

**像素 $\mathbf{p}$**：齐次坐标 $(u, v, 1)^T$。

**射线方向**：$\mathbf{d} = K^{-1} \mathbf{p}$。这是该像素对应的 3D 射线方向（在相机系下，归一化平面上的点）。

**旋转后**：同一世界射线，在新相机系下方向变为 $\mathbf{d}' = R \mathbf{d}$。

**新像素**：$\mathbf{p}' \sim K \mathbf{d}' = K R K^{-1} \mathbf{p}$。

所以 $H = K R K^{-1}$。

---

### 三、$R_x(45°)$ 的具体形式

$\cos 45° = \sin 45° = 1/\sqrt{2}$，代入绕 x 轴旋转公式：
$$
R_x(45°) = \begin{pmatrix} 1 & 0 & 0 \\ 0 & 1/\sqrt{2} & -1/\sqrt{2} \\ 0 & 1/\sqrt{2} & 1/\sqrt{2} \end{pmatrix}
$$

---

### 四、最终答案

$$
H = K R_x(45°) K^{-1}
$$

---

## 2.2 平移后各平面的单应（10 pts）

### 一、平面诱导单应（零基础版）

当所有 3D 点都在**同一个平面**上时，平面上的 2D 点（如 $(X, Y)$ 当 $Z=0$）到图像像素的映射是一个 **3×3 单应矩阵** $H$。

**公式**：$H = K [\mathbf{r}_i \mid \mathbf{r}_j \mid \mathbf{t}]$。选哪两列 $\mathbf{r}_i, \mathbf{r}_j$，取决于平面是哪两个坐标自由。

- 平面 $X_W = 0$：点在 $(0, Y_W, Z_W)$，自由的是 $Y, Z$，用 $\mathbf{r}_2, \mathbf{r}_3$
- 平面 $Y_W = 0$：点在 $(X_W, 0, Z_W)$，用 $\mathbf{r}_1, \mathbf{r}_3$
- 平面 $Z_W = 0$：点在 $(X_W, Y_W, 0)$，用 $\mathbf{r}_1, \mathbf{r}_2$

**平移 $\mathbf{t}$**：相机中心在世界系为 $\mathbf{c}$ 时，$\mathbf{t} = -R \mathbf{c}$。

---

### 二、本题数据

$R = R_x(45°)$，$\mathbf{c} = (1, 2, 2)^T$，故
$$
\mathbf{t} = -R \mathbf{c} = -\begin{pmatrix} 1 \\ 0 \\ 2\sqrt{2} \end{pmatrix} = \begin{pmatrix} -1 \\ 0 \\ -2\sqrt{2} \end{pmatrix}
$$

---

### 三、各平面的 $H$

- $X_W=0$：$H = K[\mathbf{r}_2, \mathbf{r}_3, \mathbf{t}]$
- $Y_W=0$：$H = K[\mathbf{r}_1, \mathbf{r}_3, \mathbf{t}]$
- $Z_W=0$：$H = K[\mathbf{r}_1, \mathbf{r}_2, \mathbf{t}]$

---

# 3. Stereo (20 pts)

### 一、立体配置说明

左、右相机，右相机相对左相机：$R = R_y(\beta)$（绕 y 轴），$T = (t_x, 0, t_z)^T$。相当于相机在 XZ 平面内移动。

**对极点**：左图中，右相机光心的投影 = 左对极点；右图中，左相机光心的投影 = 右对极点。

- 右对极点 $\mathbf{e}_r \sim T$（左光心在右系下）
- 左对极点 $\mathbf{e}_l \sim -R^T T$（右光心在左系下）

---

### 二、(i) 本质矩阵

$E = [T]_\times R$，代入 $T = (t_x, 0, t_z)^T$ 和 $R = R_y(\beta)$，逐行算得：
$$
E = \begin{pmatrix} 0 & -t_z & 0 \\ t_z\cos\beta + t_x\sin\beta & 0 & t_z\sin\beta - t_x\cos\beta \\ 0 & t_x & 0 \end{pmatrix}
$$

---

### 三、(ii) 对极点

$\mathbf{e}_r \sim (t_x, 0, t_z)^T$，
$\mathbf{e}_l \sim (t_z\sin\beta - t_x\cos\beta,\; 0,\; -t_x\sin\beta - t_z\cos\beta)^T$

---

### 四、(iii) 从对极点恢复

令 $(t_x, t_z) = (\cos\alpha, \sin\alpha)$，则平移方向由 $\alpha$ 决定。

- $\mathbf{e}_r$ 的方向即 $(\cos\alpha, 0, \sin\alpha)$，可求出 $\alpha$
- $\mathbf{e}_l$ 可化简为 $(-\cos(\alpha+\beta), 0, -\sin(\alpha+\beta))$，即方向角为 $\alpha+\beta$
- 故 $\beta = (\alpha+\beta) - \alpha$

---

# 4. Gaussian Splatting (20 pts)

### 一、背景：3D 高斯与投影

场景用一个 3D 高斯表示，均值 $\boldsymbol{\mu}$，协方差 $\Sigma$。投影到图像时，用**一阶近似**（在中心处线性化），3D 高斯变成 2D 高斯，2D 协方差 $S = J \Sigma_c J^T$，其中 $\Sigma_c = R\Sigma R^T$，$J$ 是投影的 Jacobian。

**题目约定**：求 $\nabla_\mu L$ 时，$J$ 和 $S$ 视为常数（不随 $\mu$ 变）。

---

### 二、(i) Jacobian $J$

投影 $\pi(x, y, z) = (f_x x/z + c_x,\; f_y y/z + c_y)$。对 $x, y, z$ 求偏导：
$$
J = \begin{pmatrix} f_x/z & 0 & -f_x x/z^2 \\ 0 & f_y/z & -f_y y/z^2 \end{pmatrix}
$$
$S = J R \Sigma R^T J^T$。

---

### 三、(ii) $\nabla_\mu L$ 的链式法则

$L = \frac{1}{2}\|\hat{I} - I\|^2$，$\hat{I} = a \mathbf{c}$，$a = \alpha g$，$g = \exp(-\frac{1}{2}\boldsymbol{\delta}^T S^{-1}\boldsymbol{\delta})$，$\boldsymbol{\delta} = \mathbf{p} - \tilde{\boldsymbol{\mu}}$，$\tilde{\boldsymbol{\mu}} = \pi(R\boldsymbol{\mu} + \mathbf{t})$。

链：$\mu \to \mathbf{X}_c \to \tilde{\boldsymbol{\mu}} \to \boldsymbol{\delta} \to g \to a \to \hat{I} \to L$。

- $\partial L / \partial \hat{I} = \hat{I} - I$
- $\partial L / \partial a = (\hat{I}-I)^T \mathbf{c}$
- $\partial g / \partial \boldsymbol{\delta} = -g S^{-1}\boldsymbol{\delta}$
- $\partial \boldsymbol{\delta} / \partial \tilde{\boldsymbol{\mu}} = -I$，$\partial \tilde{\boldsymbol{\mu}} / \partial \mathbf{X}_c = J$，$\partial \mathbf{X}_c / \partial \boldsymbol{\mu} = R$

汇总：
$$
\nabla_\mu L = ((\hat{I}-I)^T \mathbf{c}) \alpha g \cdot R^T J^T S^{-1} \boldsymbol{\delta}
$$

---

### 四、(iii) $\nabla_\Sigma L$

$L$ 通过 $S = J R \Sigma R^T J^T$ 依赖 $\Sigma$。$\partial g / \partial S = \frac{g}{2} S^{-1}\boldsymbol{\delta}\boldsymbol{\delta}^T S^{-1}$，$\partial S / \partial \Sigma$ 用链式得 $\partial L / \partial \Sigma = R^T J^T (\partial L / \partial S) J R$。代入：
$$
\nabla_\Sigma L = \frac{1}{2}((\hat{I}-I)^T \mathbf{c}) \alpha g \cdot R^T J^T S^{-1} \boldsymbol{\delta} \boldsymbol{\delta}^T S^{-1} J R
$$
