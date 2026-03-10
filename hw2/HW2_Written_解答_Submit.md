# HW2 Written 解答 · 学生提交版

> 本版为可提交的解题版，含推导步骤、符号说明与完整答案。

---

## 1. Epipolar Constraint (40 pts)

### 1.1 (10 pts) 对极线系数 A, B, C

**符号**：$E$ 本质矩阵，$T_x = (1,0,0)^T$ 平移，$\hat{T}_x = [T_x]_\times$ 反对称矩阵，$R_y(\theta)$ 绕 y 轴旋转，$p_2 = (x_2, y_2, 1)^T$ 图像 2 的齐次坐标。对极线 $\mathbf{l}_1 = E^T p_2$，直线 $(A,B,C)^T$ 对应 $Ax_1 + By_1 + C = 0$。

**推导**：

1. 反对称矩阵 $[\mathbf{t}]_\times$ 定义：$\mathbf{t} = (t_x, t_y, t_z)^T$ 时
   $$
   [\mathbf{t}]_\times = \begin{pmatrix} 0 & -t_z & t_y \\ t_z & 0 & -t_x \\ -t_y & t_x & 0 \end{pmatrix}
   $$
   代入 $T_x = (1,0,0)^T$ 得 $\hat{T}_x = \begin{pmatrix} 0 & 0 & 0 \\ 0 & 0 & -1 \\ 0 & 1 & 0 \end{pmatrix}$。

2. 绕 y 轴旋转：$R_y(\theta) = \begin{pmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{pmatrix}$。

3. $E = \hat{T}_x R_y(\theta)$，矩阵乘法得
   $$
   E = \begin{pmatrix} 0 & 0 & 0 \\ \sin\theta & 0 & -\cos\theta \\ 0 & 1 & 0 \end{pmatrix},\quad
   E^T = \begin{pmatrix} 0 & \sin\theta & 0 \\ 0 & 0 & 1 \\ 0 & -\cos\theta & 0 \end{pmatrix}
   $$

4. 对极线 $\mathbf{l}_1 = E^T p_2$：
   $$
   \mathbf{l}_1 = \begin{pmatrix} 0 & \sin\theta & 0 \\ 0 & 0 & 1 \\ 0 & -\cos\theta & 0 \end{pmatrix}
   \begin{pmatrix} x_2 \\ y_2 \\ 1 \end{pmatrix}
   = \begin{pmatrix} y_2\sin\theta \\ 1 \\ -y_2\cos\theta \end{pmatrix}
   $$

5. 直线 $\mathbf{l}_1 = (A, B, C)^T$ 对应 $Ax_1 + By_1 + C = 0$。

**答**：$A = y_2\sin\theta$，$B = 1$，$C = -y_2\cos\theta$

---

### 1.2 (10 pts) 用世界坐标系表示本质矩阵

**符号**：$c_1, c_2$ 相机光心在世界系，$^wR_1, ^wR_2$ 世界系到相机系旋转，$\mathbf{X}_w$ 世界点，$\mathbf{X}_i = {}^wR_i(\mathbf{X}_w - c_i)$ 在相机 $i$ 系下。

**推导**：

1. 由 $\mathbf{X}_1 = {}^wR_1(\mathbf{X}_w - c_1)$ 得 $\mathbf{X}_w = {}^wR_1^T \mathbf{X}_1 + c_1$。

2. 代入 $\mathbf{X}_2 = {}^wR_2(\mathbf{X}_w - c_2)$：
   $$
   \mathbf{X}_2 = {}^wR_2 {}^wR_1^T \mathbf{X}_1 + {}^wR_2(c_1 - c_2)
   $$
   故 $R = {}^wR_2 {}^wR_1^T$，$\mathbf{t} = {}^wR_2(c_1 - c_2)$（相机 1 光心在相机 2 系下）。

3. $E = [\mathbf{t}]_\times R$。用恒等式 $[R\mathbf{v}]_\times = R [\mathbf{v}]_\times R^T$，令 $\mathbf{v} = c_1 - c_2$：
   $$
   \bigl[{}^wR_2 \mathbf{v}\bigr]_\times = {}^wR_2 [\mathbf{v}]_\times {}^wR_2^T
   $$
   故 $E = {}^wR_2 [c_1-c_2]_\times {}^wR_2^T \cdot {}^wR_2 {}^wR_1^T = {}^wR_2 [c_1-c_2]_\times {}^wR_1^T$。

**答**：$E = {}^wR_2 [c_1 - c_2]_\times {}^wR_1^T$

---

### 1.3 (10 pts) Twisted Pair 歧义

**符号**：$T$ 平移，$R$ 旋转，$S$ 绕 $T$ 轴转 180°，$\mathbf{n} = T/\|T\|$。

**推导**：

1. Rodrigues 公式：$\theta = \pi$ 时 $S = I + 2[\mathbf{n}]_\times^2$，其中 $[\mathbf{n}]_\times^2 = \mathbf{n}\mathbf{n}^T - I$，故 $S = 2\mathbf{n}\mathbf{n}^T - I$。

2. $S\mathbf{v} = 2(\mathbf{n}\cdot\mathbf{v})\mathbf{n} - \mathbf{v}$。$[T]_\times(S\mathbf{v}) = 2(\mathbf{n}\cdot\mathbf{v})(T\times\mathbf{n}) - T\times\mathbf{v}$。因 $\mathbf{n}\parallel T$，$T\times\mathbf{n} = \mathbf{0}$，故 $[T]_\times(S\mathbf{v}) = -[T]_\times\mathbf{v}$，即 $[T]_\times S = -[T]_\times$。

3. $[T]_\times(SR) = ([T]_\times S)R = -[T]_\times R = -E$。对极约束 $p_2^T E p_1 = 0$ 对 $E$ 与 $-E$ 等价，故 $(T, SR)$ 亦为解。

---

### 1.4 (10 pts) 两光轴相交的充要条件

**符号**：$T$ 相机 2 光心在相机 1 系下，$R = [\mathbf{r}_1 \mid \mathbf{r}_2 \mid \mathbf{r}_3]$，$\mathbf{r}_3$ 相机 2 光轴方向，$\mathbf{e}_3 = (0,0,1)^T$ 相机 1 光轴。

**几何**：两射线相交 ⟺ 共面 ⟺ $T \cdot (\mathbf{e}_3 \times \mathbf{r}_3) = 0$。

**推导**：$\mathbf{e}_3 \times \mathbf{r}_3 = (-r_{23}, r_{13}, 0)^T$，故 $T_x r_{23} = T_y r_{13}$。退化（$r_{13}=r_{23}=0$）时两光轴平行，相交仅当 $T$ 在光轴上，即 $T_x = T_y = 0$。

**答**：一般 $T_x r_{23} = T_y r_{13}$；退化 $T_x = T_y = 0$。

---

## 2. Homography from Pose (20 pts)

### 2.1 绕 x 轴旋转 45° 的单应

**符号**：$K$ 内参，$\mathbf{p}$ 像素齐次坐标，$\mathbf{d} = K^{-1}\mathbf{p}$ 射线方向。

**推导**：相机绕光心旋转，$\mathbf{d}' = R_x(45°) \mathbf{d}$，故 $\mathbf{p}' \sim K R_x(45°) K^{-1} \mathbf{p}$，即 $H = K R_x(45°) K^{-1}$。

$$
R_x(45°) = \begin{pmatrix} 1 & 0 & 0 \\ 0 & 1/\sqrt{2} & -1/\sqrt{2} \\ 0 & 1/\sqrt{2} & 1/\sqrt{2} \end{pmatrix}
$$

---

### 2.2 平移后各平面的单应（完整矩阵）

**符号**：$\mathbf{c} = (1,2,2)^T$ 相机中心，$\mathbf{t} = -R\mathbf{c}$，$R = R_x(45°)$ 的列 $\mathbf{r}_1, \mathbf{r}_2, \mathbf{r}_3$。平面诱导单应 $H = K[\mathbf{r}_i, \mathbf{r}_j, \mathbf{t}]$，选平面两个自由维度对应的列。

**推导**：
$$
R = \begin{pmatrix} 1 & 0 & 0 \\ 0 & 1/\sqrt{2} & -1/\sqrt{2} \\ 0 & 1/\sqrt{2} & 1/\sqrt{2} \end{pmatrix},\quad
\mathbf{r}_1 = \begin{pmatrix} 1 \\ 0 \\ 0 \end{pmatrix},\;
\mathbf{r}_2 = \begin{pmatrix} 0 \\ 1/\sqrt{2} \\ 1/\sqrt{2} \end{pmatrix},\;
\mathbf{r}_3 = \begin{pmatrix} 0 \\ -1/\sqrt{2} \\ 1/\sqrt{2} \end{pmatrix}
$$
$$
\mathbf{t} = -R\mathbf{c} = -\begin{pmatrix} 1 & 0 & 0 \\ 0 & 1/\sqrt{2} & -1/\sqrt{2} \\ 0 & 1/\sqrt{2} & 1/\sqrt{2} \end{pmatrix}
\begin{pmatrix} 1 \\ 2 \\ 2 \end{pmatrix}
= \begin{pmatrix} -1 \\ 0 \\ -2\sqrt{2} \end{pmatrix}
$$

**(a) 平面 $X_W = 0$**：点 $(0, Y_W, Z_W)$，用 $\mathbf{r}_2, \mathbf{r}_3$：
$$
H_{X=0} = K \begin{pmatrix} 0 & 0 & -1 \\ 1/\sqrt{2} & -1/\sqrt{2} & 0 \\ 1/\sqrt{2} & 1/\sqrt{2} & -2\sqrt{2} \end{pmatrix}
$$

**(b) 平面 $Y_W = 0$**：点 $(X_W, 0, Z_W)$，用 $\mathbf{r}_1, \mathbf{r}_3$：
$$
H_{Y=0} = K \begin{pmatrix} 1 & 0 & -1 \\ 0 & -1/\sqrt{2} & 0 \\ 0 & 1/\sqrt{2} & -2\sqrt{2} \end{pmatrix}
$$

**(c) 平面 $Z_W = 0$**：点 $(X_W, Y_W, 0)$，用 $\mathbf{r}_1, \mathbf{r}_2$：
$$
H_{Z=0} = K \begin{pmatrix} 1 & 0 & -1 \\ 0 & 1/\sqrt{2} & 0 \\ 0 & 1/\sqrt{2} & -2\sqrt{2} \end{pmatrix}
$$

---

## 3. Stereo (20 pts)

**符号**：$\mathbf{X}_r = R \mathbf{X}_l + T$，$R = R_y(\beta)$，$T = (t_x, 0, t_z)^T$。$\mathbf{e}_r$ 右对极点（左光心在右图），$\mathbf{e}_l$ 左对极点（右光心在左图）。

### (i) 本质矩阵

**推导**：$[T]_\times = \begin{pmatrix} 0 & -t_z & 0 \\ t_z & 0 & -t_x \\ 0 & t_x & 0 \end{pmatrix}$，$E = [T]_\times R$，逐行算得
$$
E = \begin{pmatrix} 0 & -t_z & 0 \\ t_z\cos\beta + t_x\sin\beta & 0 & t_z\sin\beta - t_x\cos\beta \\ 0 & t_x & 0 \end{pmatrix}
$$

### (ii) 对极点

**来源**：对极点也可由 $E$ 的零空间得到。$\mathbf{e}_r$ 为右图对极点，满足 $E^T \mathbf{e}_r = 0$（对极线退化为点）。由 $E = [T]_\times R$ 得 $E^T = R^T [T]_\times^T = -R^T [T]_\times$（因 $[T]_\times^T = -[T]_\times$），故 $E^T \mathbf{e}_r = 0 \Leftrightarrow [T]_\times \mathbf{e}_r = 0$。$[T]_\times$ 的零空间由 $T$ 张成（$T \times T = 0$），故 $\mathbf{e}_r \sim T$。同理 $\mathbf{e}_l$ 满足 $E \mathbf{e}_l = 0$，由 $E \mathbf{e}_l = [T]_\times R \mathbf{e}_l = 0$ 得 $R \mathbf{e}_l \sim T$，故 $\mathbf{e}_l \sim R^T T$；几何上右光心在左系下为 $-R^T T$，故 $\mathbf{e}_l \sim -R^T T$（差一常数等价）。

$R^T T = (t_x\cos\beta - t_z\sin\beta,\; 0,\; t_x\sin\beta + t_z\cos\beta)^T$，故
$$
\mathbf{e}_r \sim \begin{pmatrix} t_x \\ 0 \\ t_z \end{pmatrix},\quad
\mathbf{e}_l \sim \begin{pmatrix} t_z\sin\beta - t_x\cos\beta \\ 0 \\ -t_x\sin\beta - t_z\cos\beta \end{pmatrix}
$$

### (iii) 从对极点恢复

令 $(t_x, t_z) = (\cos\alpha, \sin\alpha)$。$\mathbf{e}_r$ 得 $\alpha$；$\mathbf{e}_l \sim (-\cos(\alpha+\beta), 0, -\sin(\alpha+\beta))$ 得 $\alpha+\beta$，故 $\beta = (\alpha+\beta) - \alpha$。

---

## 4. Gaussian Splatting (20 pts)

**符号**：$\boldsymbol{\mu}$ 均值，$\Sigma$ 协方差，$\tilde{\boldsymbol{\mu}} = \pi(R\boldsymbol{\mu}+\mathbf{t})$ 投影中心，$\boldsymbol{\delta} = \mathbf{p} - \tilde{\boldsymbol{\mu}}$，$\Sigma_c = R\Sigma R^T$，$S = J \Sigma_c J^T$，$g = \exp(-\frac{1}{2}\boldsymbol{\delta}^T S^{-1}\boldsymbol{\delta})$，$a = \alpha g$，$\hat{I} = a\mathbf{c}$，$L = \frac{1}{2}\|\hat{I}-I\|^2$。**约定**：求 $\nabla_\mu L$ 时 $\partial J/\partial\mu = \partial S/\partial\mu = 0$。

### (i) Jacobian 与 2D 协方差

**推导**：$\pi(x,y,z) = (u, v)$，$u = f_x x/z + c_x$，$v = f_y y/z + c_y$。
$$
\frac{\partial u}{\partial x} = \frac{f_x}{z},\; \frac{\partial u}{\partial y} = 0,\; \frac{\partial u}{\partial z} = -\frac{f_x x}{z^2}
$$
$$
\frac{\partial v}{\partial x} = 0,\; \frac{\partial v}{\partial y} = \frac{f_y}{z},\; \frac{\partial v}{\partial z} = -\frac{f_y y}{z^2}
$$
故 $J = \frac{\partial\pi}{\partial\mathbf{X}_c} = \begin{pmatrix} f_x/z & 0 & -f_x x/z^2 \\ 0 & f_y/z & -f_y y/z^2 \end{pmatrix}$，其中 $(x,y,z)^T = R\boldsymbol{\mu} + \mathbf{t}$。$S = J \Sigma_c J^T = J R \Sigma R^T J^T$。

### (ii) $\nabla_\mu L$

**推导**：链式法则 $\mu \to \mathbf{X}_c \to \tilde{\boldsymbol{\mu}} \to \boldsymbol{\delta} \to g \to a \to \hat{I} \to L$。

- $\partial L/\partial\hat{I} = \hat{I} - I$
- $\partial L/\partial a = (\hat{I}-I)^T \mathbf{c}$
- $\partial g/\partial\boldsymbol{\delta} = -g S^{-1}\boldsymbol{\delta}$
- $\partial\boldsymbol{\delta}/\partial\tilde{\boldsymbol{\mu}} = -I$，$\partial\tilde{\boldsymbol{\mu}}/\partial\mathbf{X}_c = J$，$\partial\mathbf{X}_c/\partial\boldsymbol{\mu} = R$

故 $\nabla_\mu L = ((\hat{I}-I)^T \mathbf{c}) \alpha g \cdot R^T J^T S^{-1} \boldsymbol{\delta}$。

### (iii) $\nabla_\Sigma L$（含对称化）

**推导**：$L$ 经 $S = J R \Sigma R^T J^T$ 依赖 $\Sigma$。

1. 矩阵求导公式：对对称 $S$，$\frac{\partial (a^T S^{-1} b)}{\partial S} = -S^{-T} a b^T S^{-T}$。令 $a = b = \boldsymbol{\delta}$ 得 $\frac{\partial (\boldsymbol{\delta}^T S^{-1}\boldsymbol{\delta})}{\partial S} = -S^{-1}\boldsymbol{\delta}\boldsymbol{\delta}^T S^{-1}$（$S$ 对称故 $S^{-T} = S^{-1}$）。故 $\partial g/\partial S = g \cdot (-\frac{1}{2}) \cdot (-S^{-1}\boldsymbol{\delta}\boldsymbol{\delta}^T S^{-1}) = \frac{g}{2} S^{-1}\boldsymbol{\delta}\boldsymbol{\delta}^T S^{-1}$。

2. $\partial L/\partial S = (\hat{I}-I)^T \mathbf{c} \cdot \alpha \cdot \frac{g}{2} S^{-1}\boldsymbol{\delta}\boldsymbol{\delta}^T S^{-1}$。

3. $S = (JR)\Sigma(R^T J^T)$，矩阵求导链式：$\partial L/\partial\Sigma = (JR)^T (\partial L/\partial S) (R^T J^T)$。代入得
$$
\nabla_\Sigma L = \frac{1}{2}((\hat{I}-I)^T \mathbf{c}) \alpha g \cdot R^T J^T S^{-1} \boldsymbol{\delta} \boldsymbol{\delta}^T S^{-1} J R
$$

**对称化**（题目要求 "you may symmetrize it at the end"）：$\Sigma$ 为对称矩阵，梯度可取对称形式
$$
\nabla_\Sigma L \leftarrow \frac{1}{2}\bigl(\nabla_\Sigma L + (\nabla_\Sigma L)^T\bigr)
$$
