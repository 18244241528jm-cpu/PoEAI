# HW2 Written 解答

## 1. Epipolar Constraint (40 pts)

### 1.1 (10 pts) 对极线系数 A, B, C

---

#### 一、题意与所需知识

**题意**：给定本质矩阵 $E = \hat{T}_x R_y(\theta)$，其中平移 $T_x = (1, 0, 0)$，$\hat{T}_x$ 为 $T_x$ 的反对称矩阵，$R_y(\theta)$ 为绕 y 轴旋转 $\theta$ 的旋转矩阵。图像坐标已标定（calibrated），故使用本质矩阵 $E$。求：图像平面 1 中对极线 $Ax_1 + By_1 + C = 0$ 的系数 A, B, C。

**符号说明**（本节首次出现）：
- $E$：本质矩阵（Essential Matrix），编码两相机相对位姿
- $T_x$：平移向量，此处为 $(1,0,0)^T$
- $\hat{T}_x$（或 $[T_x]_\times$）：$T_x$ 的反对称矩阵
- $R_y(\theta)$：绕 y 轴旋转 $\theta$ 的旋转矩阵
- $\theta$：绕 y 轴的旋转角
- $x_1, y_1$：图像平面 1 中点的横、纵坐标
- $x_2, y_2$：图像平面 2 中点的横、纵坐标
- $A, B, C$：对极线方程的系数
- $\mathbf{l}_1$：图像 1 中的对极线，$\mathbf{l}_1 = (A, B, C)^T$ 对应直线 $Ax_1 + By_1 + C = 0$
- $p_1, p_2$：相机 1、相机 2 中的齐次图像坐标，$p_i = (x_i, y_i, 1)^T$

**所需知识**：
- **对极约束**：$p_2^T E p_1 = 0$。
- **对极线关系**：给定图像 2 中的点 $p_2$，其在图像 1 中的对极线为 $\mathbf{l}_1 = E^T p_2$。直线 $\mathbf{l}_1 = (A, B, C)^T$ 对应方程 $Ax_1 + By_1 + C = 0$。
- **反对称矩阵**：向量 $\mathbf{t} = (t_x, t_y, t_z)^T$ 的反对称矩阵 $[\mathbf{t}]_\times$ 满足 $\mathbf{t} \times \mathbf{v} = [\mathbf{t}]_\times \mathbf{v}$；$t_x, t_y, t_z$ 为 $\mathbf{t}$ 的三个分量。

---

#### 二、推导过程

**Step 1：写出 $\hat{T}_x$（即 $[T_x]_\times$）**

对于 $\mathbf{t} = (t_x, t_y, t_z)^T$，反对称矩阵定义为：
$$
[\mathbf{t}]_\times = \begin{pmatrix} 0 & -t_z & t_y \\ t_z & 0 & -t_x \\ -t_y & t_x & 0 \end{pmatrix}
$$

代入 $T_x = (1, 0, 0)^T$，得：
$$
\hat{T}_x = [T_x]_\times = \begin{pmatrix} 0 & 0 & 0 \\ 0 & 0 & -1 \\ 0 & 1 & 0 \end{pmatrix}
$$

**Step 2：写出 $R_y(\theta)$**

绕 y 轴（右手系）旋转 $\theta$ 的旋转矩阵为：
$$
R_y(\theta) = \begin{pmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{pmatrix}
$$

**Step 3：计算本质矩阵 $E = \hat{T}_x R_y(\theta)$**

$$
E = \begin{pmatrix} 0 & 0 & 0 \\ 0 & 0 & -1 \\ 0 & 1 & 0 \end{pmatrix}
\begin{pmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{pmatrix}
= \begin{pmatrix} 0 & 0 & 0 \\ \sin\theta & 0 & -\cos\theta \\ 0 & 1 & 0 \end{pmatrix}
$$

**Step 4：计算 $E^T$**

$$
E^T = \begin{pmatrix} 0 & \sin\theta & 0 \\ 0 & 0 & 1 \\ 0 & -\cos\theta & 0 \end{pmatrix}
$$

**Step 5：求图像 1 中的对极线 $\mathbf{l}_1 = E^T p_2$**

设图像 2 中的点为 $p_2 = (x_2, y_2, 1)^T$（$x_2, y_2$ 为该点的横、纵坐标），则：
$$
\mathbf{l}_1 = E^T p_2 = \begin{pmatrix} 0 & \sin\theta & 0 \\ 0 & 0 & 1 \\ 0 & -\cos\theta & 0 \end{pmatrix}
\begin{pmatrix} x_2 \\ y_2 \\ 1 \end{pmatrix}
= \begin{pmatrix} y_2 \sin\theta \\ 1 \\ -y_2 \cos\theta \end{pmatrix}
$$

**Step 6：写出对极线方程**

$\mathbf{l}_1 = (A, B, C)^T$ 对应直线 $Ax_1 + By_1 + C = 0$，故：
$$
(y_2 \sin\theta)\, x_1 + 1 \cdot y_1 + (-y_2 \cos\theta) = 0
$$

---

#### 三、最终答案

$$
\boxed{
\begin{aligned}
A &= y_2 \sin\theta \\
B &= 1 \\
C &= -y_2 \cos\theta
\end{aligned}
}
$$

其中 $y_2$ 为图像 2 中对应点的纵坐标，$\theta$ 为绕 y 轴的旋转角。

---

### 1.2 (10 pts) 用世界坐标系下的相机位姿表示本质矩阵

---

#### 一、题意与所需知识

**题意**：两相机在世界坐标系下的光心为 $c_1, c_2$，姿态（旋转）为 $^wR_1, ^wR_2$。写出对极约束 $p_2^T E p_1 = 0$ 中的本质矩阵 $E$，其中 $p_1, p_2$ 分别为相机 1、相机 2 的标定图像坐标。

**符号说明**（本节首次出现）：
- $c_1, c_2$：相机 1、相机 2 的光心在世界坐标系下的坐标（3D 向量）
- $^wR_1, ^wR_2$：从世界系到相机 1、相机 2 的旋转矩阵（world-to-camera），$^wR_i \in SO(3)$（$SO(3)$ 为 3D 旋转矩阵群）
- $\mathbf{X}_w$：3D 点在世界坐标系下的坐标
- $\mathbf{X}_1, \mathbf{X}_2$：同一 3D 点在相机 1、相机 2 坐标系下的坐标
- $R$：相机 2 相对于相机 1 的旋转矩阵
- $\mathbf{t}$：相机 2 相对于相机 1 的平移向量（相机 1 光心在相机 2 系下的坐标）

**所需知识**：
- 本质矩阵 $E = [\mathbf{t}]_\times R$，其中 $R, \mathbf{t}$ 为相机 2 相对于相机 1 的旋转和平移。
- 标准设定：$\mathbf{X}_2 = R \mathbf{X}_1 + \mathbf{t}$，$\mathbf{X}_1, \mathbf{X}_2$ 为同一 3D 点在两相机坐标系下的坐标。
- 恒等式：$[R\mathbf{v}]_\times = R [\mathbf{v}]_\times R^T$。

---

#### 二、符号约定

- $c_1, c_2 \in \mathbb{R}^3$：相机光心在世界坐标系下的坐标。
- $^wR_1, ^wR_2 \in SO(3)$：从世界系到相机 1、相机 2 的旋转矩阵。即世界系中的向量 $\mathbf{v}$ 在相机 $i$ 系下为 $^wR_i \mathbf{v}$。
- 世界系中的 3D 点 $\mathbf{X}_w$ 在相机 $i$ 系下：$\mathbf{X}_i = {}^wR_i (\mathbf{X}_w - c_i)$。

---

#### 三、推导过程

**Step 1：建立相机 1 到相机 2 的变换**

设同一 3D 点在世界系为 $\mathbf{X}_w$，在相机 1、2 系下分别为 $\mathbf{X}_1, \mathbf{X}_2$：
$$
\mathbf{X}_1 = {}^wR_1 (\mathbf{X}_w - c_1), \qquad
\mathbf{X}_2 = {}^wR_2 (\mathbf{X}_w - c_2)
$$

由 $\mathbf{X}_1 = {}^wR_1 (\mathbf{X}_w - c_1)$ 得 $\mathbf{X}_w = {}^wR_1^T \mathbf{X}_1 + c_1$。代入第二式：
$$
\mathbf{X}_2 = {}^wR_2 \bigl({}^wR_1^T \mathbf{X}_1 + c_1 - c_2\bigr)
= {}^wR_2 {}^wR_1^T \mathbf{X}_1 + {}^wR_2 (c_1 - c_2)
$$

故相对位姿为：
$$
R = {}^wR_2 {}^wR_1^T, \qquad \mathbf{t} = {}^wR_2 (c_1 - c_2)
$$

其中 $\mathbf{t}$ 为相机 1 光心在相机 2 系下的坐标（从相机 2 指向相机 1 的向量在相机 2 系下的表示）。

**Step 2：写出本质矩阵**

$$
E = [\mathbf{t}]_\times R = \bigl[{}^wR_2 (c_1 - c_2)\bigr]_\times \, {}^wR_2 {}^wR_1^T
$$

**Step 3：利用恒等式 $[R\mathbf{v}]_\times = R [\mathbf{v}]_\times R^T$**

令 $\mathbf{v} = c_1 - c_2$，则：
$$
\bigl[{}^wR_2 \mathbf{v}\bigr]_\times = {}^wR_2 [\mathbf{v}]_\times {}^wR_2^T
$$

代入 $E$：
$$
E = {}^wR_2 [c_1 - c_2]_\times {}^wR_2^T \cdot {}^wR_2 {}^wR_1^T
= {}^wR_2 [c_1 - c_2]_\times {}^wR_1^T
$$

---

#### 四、最终答案

$$
\boxed{
E = {}^wR_2 \,[c_1 - c_2]_\times \,{}^wR_1^T
}
$$

其中 $[c_1 - c_2]_\times$ 为向量 $c_1 - c_2$ 的反对称矩阵。

---

### 1.3 (10 pts) Twisted Pair 歧义证明

---

#### 一、题意与所需知识

**题意**：本质矩阵 $E$ 的分解存在歧义。证明：若 $(T, R)$ 是 $E = [T]_\times R$ 的一组解，则 $(T, SR)$ 也是解，其中 $S$ 为绕 $T$ 轴旋转 180° 的旋转矩阵。提示：可用 Rodrigues 公式。

**符号说明**（本节首次出现）：
- $T$：平移向量（本质矩阵分解中的平移）
- $R$：旋转矩阵（本质矩阵分解中的旋转）
- $S$：绕 $T$ 轴旋转 180° 的旋转矩阵
- $\mathbf{k}$：Rodrigues 公式中的单位旋转轴
- $\mathbf{n}$：$T$ 方向的单位向量，$\mathbf{n} = T/\|T\|$
- $I$：$3\times 3$ 单位矩阵
- $\mathbf{v}$：任意 3D 向量

**所需知识**：
- 本质矩阵 $E = [T]_\times R$，对极约束 $p_2^T E p_1 = 0$ 对 $E$ 的尺度不变（$E$ 与 $-E$ 等价）。
- **Rodrigues 公式**：绕单位轴 $\mathbf{k}$ 旋转 $\theta$ 的矩阵 $R = I + \sin\theta [\mathbf{k}]_\times + (1-\cos\theta) [\mathbf{k}]_\times^2$。
- 当 $\theta = 180°$ 时，$\sin\pi = 0$，$\cos\pi = -1$，故 $S = I + 2[\mathbf{k}]_\times^2$。

---

#### 二、证明

**Step 1：写出绕 $T$ 旋转 180° 的矩阵 $S$**

设 $\mathbf{n} = T / \|T\|$ 为 $T$ 方向的单位向量。由 Rodrigues 公式，$\theta = \pi$ 时：
$$
S = I + \sin\pi [\mathbf{n}]_\times + (1 - \cos\pi) [\mathbf{n}]_\times^2
= I + 2 [\mathbf{n}]_\times^2
$$

对单位向量 $\mathbf{n}$，有 $[\mathbf{n}]_\times^2 = \mathbf{n}\mathbf{n}^T - I$，故：
$$
S = I + 2(\mathbf{n}\mathbf{n}^T - I) = 2\mathbf{n}\mathbf{n}^T - I
$$

**Step 2：证明 $[T]_\times S = -[T]_\times$**

对任意向量 $\mathbf{v}$：
$$
S\mathbf{v} = (2\mathbf{n}\mathbf{n}^T - I)\mathbf{v} = 2(\mathbf{n} \cdot \mathbf{v}) \mathbf{n} - \mathbf{v}
$$

于是：
$$
[T]_\times (S\mathbf{v}) = T \times \bigl(2(\mathbf{n} \cdot \mathbf{v}) \mathbf{n} - \mathbf{v}\bigr)
= 2(\mathbf{n} \cdot \mathbf{v})(T \times \mathbf{n}) - T \times \mathbf{v}
$$

因为 $\mathbf{n} \parallel T$，有 $T \times \mathbf{n} = \mathbf{0}$，故：
$$
[T]_\times (S\mathbf{v}) = - T \times \mathbf{v} = - [T]_\times \mathbf{v}
$$

因此作为矩阵等式：
$$
[T]_\times S = -[T]_\times
$$

**Step 3：验证 $(T, SR)$ 满足本质矩阵形式**

$$
[T]_\times (SR) = ([T]_\times S) R = (-[T]_\times) R = -[T]_\times R = -E
$$

对极约束为 $p_2^T E p_1 = 0$。令 $E' = [T]_\times (SR) = -E$（$E'$ 为由 $(T, SR)$ 得到的本质矩阵），则：
$$
p_2^T E' p_1 = p_2^T (-E) p_1 = - (p_2^T E p_1) = 0
$$

故 $E'$ 与 $E$ 满足相同的对极约束。本质矩阵差一全局尺度等价，$(T, SR)$ 与 $(T, R)$ 均为有效分解。

---

#### 三、结论

$$
\boxed{
\text{若 } E = [T]_\times R \text{，则 } [T]_\times (SR) = -E \text{，且对极约束 } p_2^T E p_1 = 0 \text{ 对 } E \text{ 与 } -E \text{ 等价，故 } (T, SR) \text{ 亦为解。}}
$$

---

### 1.4 (10 pts) 两光轴相交的充要条件

---

#### 一、题意与所需知识

**题意**：求 $T$ 与 $R$ 的元素的充要条件，使得两视图相机系统的两条光轴相交（类似人眼注视同一物点）。

**符号说明**（本节首次出现）：
- $T$：相机 2 光心在相机 1 坐标系下的位置（平移向量），$T = (T_x, T_y, T_z)^T$
- $R$：相机 2 相对于相机 1 的旋转矩阵，$R = [\mathbf{r}_1 \mid \mathbf{r}_2 \mid \mathbf{r}_3]$
- $\mathbf{r}_1, \mathbf{r}_2, \mathbf{r}_3$：$R$ 的三列
- $\mathbf{r}_3$：$R$ 的第三列，即相机 2 光轴在相机 1 系下的方向
- $\mathbf{e}_3$：$(0, 0, 1)^T$，相机 1 光轴方向（z 轴）
- $r_{13}, r_{23}, r_{33}$：$\mathbf{r}_3$ 的三个分量
- $s, t$：射线参数（标量）

**所需知识**：
- 标准设定：相机 1 在原点、单位朝向；相机 2 相对位姿为 $(R, T)$。
- 光轴：相机 1 的光轴为 $(0, 0, 1)$；相机 2 的光轴在相机 1 系下为 $R$ 的第三列 $\mathbf{r}_3$。
- 两直线相交：两射线共面且不平行（或平行时基线在其上）。

---

#### 二、设定与几何

记 $R = [\mathbf{r}_1 \mid \mathbf{r}_2 \mid \mathbf{r}_3]$，$\mathbf{r}_3$ 为相机 2 光轴在相机 1 系下的方向。设 $\mathbf{e}_3 = (0, 0, 1)^T$ 为相机 1 光轴。

- 射线 1：过原点，方向 $\mathbf{e}_3$，参数形式 $\{s \mathbf{e}_3 : s \in \mathbb{R}\}$
- 射线 2：过 $T$，方向 $\mathbf{r}_3$，参数形式 $\{T + t \mathbf{r}_3 : t \in \mathbb{R}\}$

两射线相交当且仅当 $T$ 落在由 $\mathbf{e}_3$ 与 $\mathbf{r}_3$ 张成的平面内（共面条件）。

---

#### 三、推导

**共面条件**：$T$、$\mathbf{e}_3$、$\mathbf{r}_3$ 共面 ⟺ $T \cdot (\mathbf{e}_3 \times \mathbf{r}_3) = 0$（$\times$ 为叉积，$\cdot$ 为点积；$\mathbf{e}_3 \times \mathbf{r}_3$ 为两向量张成平面的法向）。

$$
\mathbf{e}_3 \times \mathbf{r}_3 = \begin{pmatrix} 0 \\ 0 \\ 1 \end{pmatrix} \times \begin{pmatrix} r_{13} \\ r_{23} \\ r_{33} \end{pmatrix}
= \begin{pmatrix} -r_{23} \\ r_{13} \\ 0 \end{pmatrix}
$$

故：
$$
T \cdot (\mathbf{e}_3 \times \mathbf{r}_3) = T_x (-r_{23}) + T_y (r_{13}) + T_z \cdot 0 = 0
$$

即
$$
\boxed{T_x \cdot r_{23} = T_y \cdot r_{13}}
$$

**退化情形**：若 $\mathbf{r}_3 \parallel \mathbf{e}_3$（$\parallel$ 表示平行，即 $r_{13} = r_{23} = 0$），则 $\mathbf{e}_3 \times \mathbf{r}_3 = \mathbf{0}$，上式恒成立。此时两光轴平行，相交仅当 $T$ 在光轴上，即 $T_x = T_y = 0$。

---

#### 四、最终答案

**充要条件**（分情形）：

1. **一般情形**（$\mathbf{r}_3$ 不平行于 $\mathbf{e}_3$，即 $r_{13}^2 + r_{23}^2 \neq 0$）：
   $$
   T_x \cdot r_{23} = T_y \cdot r_{13}
   $$

2. **退化情形**（$\mathbf{r}_3 \parallel \mathbf{e}_3$，即 $r_{13} = r_{23} = 0$）：
   $$
   T_x = T_y = 0
   $$

**等价表述**：$T$ 在由 $\mathbf{e}_3$ 与 $\mathbf{r}_3$ 张成的平面内，即 $T \in \operatorname{span}\{\mathbf{e}_3, \mathbf{r}_3\}$（$\operatorname{span}$ 表示向量张成的线性空间）。

---

## 2. Homography from Pose (20 pts)

### 2.1 绕 x 轴旋转 45° 的单应

---

#### 一、题意与所需知识

**题意**：相机初始在原点，体轴与世界轴对齐。相机绕自身 x 轴旋转 $\theta = 45°$。求将旋转前的像素点映射到旋转后像素点的单应矩阵 $H$。

**符号说明**（本节首次出现）：
- $H$：单应矩阵（Homography），将旋转前像素映射到旋转后像素的 $3\times 3$ 矩阵
- $\mathbf{p}$：像素齐次坐标，$\mathbf{p} = (u, v, 1)^T$
- $K$：相机内参矩阵（intrinsics），$3\times 3$ 可逆矩阵
- $\mathbf{X}_c$：点在相机坐标系下的齐次坐标（射线方向）
- $R_x(\theta)$：绕相机 x 轴旋转 $\theta$ 的旋转矩阵
- $\mathbf{p}_1, \mathbf{p}_2$：旋转前、旋转后的像素坐标
- $\mathbf{d}_1, \mathbf{d}_2$：归一化平面上的射线方向，$\mathbf{d} = K^{-1}\mathbf{p}$
- $\sim$：齐次意义下相等（差一非零常数因子）

**所需知识**：
- 像素坐标与相机系方向的关系：$\mathbf{p} \sim K \mathbf{X}_c$，其中 $\mathbf{X}_c$ 为相机系下的射线方向（齐次）。
- 相机绕光心旋转时，同一世界射线在旋转前后的相机系下满足 $\mathbf{X}_c' = R \mathbf{X}_c$，故 $\mathbf{p}' \sim K R K^{-1} \mathbf{p}$。

---

#### 二、推导

旋转前后相机光心不变。设旋转前像素 $\mathbf{p}_1$ 对应射线方向 $\mathbf{d}_1 = K^{-1} \mathbf{p}_1$（归一化坐标）。旋转后，该射线在新相机系下为 $\mathbf{d}_2 = R_x(45°) \mathbf{d}_1$，故新像素：
$$
\mathbf{p}_2 \sim K \mathbf{d}_2 = K R_x(45°) K^{-1} \mathbf{p}_1
$$

绕 x 轴旋转 $\theta$ 的矩阵：
$$
R_x(\theta) = \begin{pmatrix} 1 & 0 & 0 \\ 0 & \cos\theta & -\sin\theta \\ 0 & \sin\theta & \cos\theta \end{pmatrix}
$$

代入 $\theta = 45°$，$\cos 45° = \sin 45° = 1/\sqrt{2}$：
$$
R_x(45°) = \begin{pmatrix} 1 & 0 & 0 \\ 0 & 1/\sqrt{2} & -1/\sqrt{2} \\ 0 & 1/\sqrt{2} & 1/\sqrt{2} \end{pmatrix}
$$

---

#### 三、最终答案

$$
\boxed{
H = K \, R_x(45°) \, K^{-1}
= K \begin{pmatrix} 1 & 0 & 0 \\ 0 & 1/\sqrt{2} & -1/\sqrt{2} \\ 0 & 1/\sqrt{2} & 1/\sqrt{2} \end{pmatrix} K^{-1}
}
$$

其中 $K$ 为相机内参矩阵。若 $K = \begin{pmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{pmatrix}$（$f_x, f_y$：焦距；$c_x, c_y$：主点坐标），则 $H$ 的具体形式由 $K R_x(45°) K^{-1}$ 代入可得。

---

### 2.2 平移后各平面的世界坐标到像素单应

---

#### 一、题意与所需知识

**题意**：在 2.1 的旋转基础上，相机再平移到世界系中的 $(1, 2, 2)^T$。对平面 $X_W=0$、$Y_W=0$、$Z_W=0$ 分别求将世界坐标映射到像素坐标的 $3\times 3$ 矩阵 $H$。

**符号说明**（本节首次出现）：
- $X_W, Y_W, Z_W$：世界坐标系下的三个坐标分量
- $\mathbf{c}$：相机光心在世界坐标系下的坐标，此处 $\mathbf{c} = (1, 2, 2)^T$
- $\mathbf{t}$：world-to-camera 平移向量，$\mathbf{t} = -R\mathbf{c}$
- $\lambda$：非零尺度因子（单应差一尺度等价）
- $\mathbf{r}_i, \mathbf{r}_j$：$R$ 的列向量，对应平面上的两个自由维度
- $\mathbf{X}_w$：3D 点在世界坐标系下的坐标
- $\mathbf{X}_c$：3D 点在相机坐标系下的坐标

**所需知识**：
- 平面诱导单应：$H = \lambda K [\mathbf{r}_i \mid \mathbf{r}_j \mid \mathbf{t}]$，其中 $\mathbf{r}_i, \mathbf{r}_j$ 为 $R$ 中与该平面两个自由维度对应的列，$\mathbf{t}$ 为平移（world-to-camera）。
- 外参：$\mathbf{X}_c = R \mathbf{X}_w + \mathbf{t}$，相机中心在世界系为 $\mathbf{c}$ 时，$\mathbf{t} = -R \mathbf{c}$。

---

#### 二、设定

- 旋转：$R = R_x(45°)$，记列向量为 $\mathbf{r}_1, \mathbf{r}_2, \mathbf{r}_3$。
- 相机中心：$\mathbf{c} = (1, 2, 2)^T$，故 $\mathbf{t} = -R \mathbf{c}$。

$$
R = \begin{pmatrix} 1 & 0 & 0 \\ 0 & 1/\sqrt{2} & -1/\sqrt{2} \\ 0 & 1/\sqrt{2} & 1/\sqrt{2} \end{pmatrix}, \quad
\mathbf{r}_1 = \begin{pmatrix} 1 \\ 0 \\ 0 \end{pmatrix}, \;
\mathbf{r}_2 = \begin{pmatrix} 0 \\ 1/\sqrt{2} \\ 1/\sqrt{2} \end{pmatrix}, \;
\mathbf{r}_3 = \begin{pmatrix} 0 \\ -1/\sqrt{2} \\ 1/\sqrt{2} \end{pmatrix}
$$

$$
\mathbf{t} = -R \mathbf{c} = -\begin{pmatrix} 1 & 0 & 0 \\ 0 & 1/\sqrt{2} & -1/\sqrt{2} \\ 0 & 1/\sqrt{2} & 1/\sqrt{2} \end{pmatrix}
\begin{pmatrix} 1 \\ 2 \\ 2 \end{pmatrix}
= -\begin{pmatrix} 1 \\ 0 \\ 2\sqrt{2} \end{pmatrix}
= \begin{pmatrix} -1 \\ 0 \\ -2\sqrt{2} \end{pmatrix}
$$

---

#### 三、各平面的 $H$

平面上的点可写成 2D 齐次坐标，$H$ 的列为 $K$ 左乘 $[\mathbf{r}_i, \mathbf{r}_j, \mathbf{t}]$ 中对应的列。

**(a) 平面 $X_W = 0$**

平面上点为 $(0, Y_W, Z_W)$，对应齐次 $(Y_W, Z_W, 1)^T$，需 $\mathbf{r}_2, \mathbf{r}_3$。记 $H_{X=0}$ 为该平面的世界坐标到像素的单应矩阵：
$$
\boxed{
H_{X=0} = K \begin{bmatrix} \mathbf{r}_2 & \mathbf{r}_3 & \mathbf{t} \end{bmatrix}
= K \begin{pmatrix} 0 & 0 & -1 \\ 1/\sqrt{2} & -1/\sqrt{2} & 0 \\ 1/\sqrt{2} & 1/\sqrt{2} & -2\sqrt{2} \end{pmatrix}
}
$$

**(b) 平面 $Y_W = 0$**

平面上点为 $(X_W, 0, Z_W)$，需 $\mathbf{r}_1, \mathbf{r}_3$：
$$
\boxed{
H_{Y=0} = K \begin{bmatrix} \mathbf{r}_1 & \mathbf{r}_3 & \mathbf{t} \end{bmatrix}
= K \begin{pmatrix} 1 & 0 & -1 \\ 0 & -1/\sqrt{2} & 0 \\ 0 & 1/\sqrt{2} & -2\sqrt{2} \end{pmatrix}
}
$$

**(c) 平面 $Z_W = 0$**

平面上点为 $(X_W, Y_W, 0)$，需 $\mathbf{r}_1, \mathbf{r}_2$：
$$
\boxed{
H_{Z=0} = K \begin{bmatrix} \mathbf{r}_1 & \mathbf{r}_2 & \mathbf{t} \end{bmatrix}
= K \begin{pmatrix} 1 & 0 & -1 \\ 0 & 1/\sqrt{2} & 0 \\ 0 & 1/\sqrt{2} & -2\sqrt{2} \end{pmatrix}
}
$$

以上 $H$ 将各平面上的 2D 齐次坐标 $(u, v, 1)^T$ 映射为像素齐次坐标 $\mathbf{p} \sim H (u, v, 1)^T$，其中 $u, v$ 为该平面上两个自由维度的世界坐标（如平面 $Z_W=0$ 时 $u=X_W, v=Y_W$）。

---

## 3. Stereo (20 pts)

立体相机配置：两相机内参相同，右相机相对左相机满足 $\mathbf{X}_r = R \mathbf{X}_l + T$，其中 $R$ 为绕 y 轴旋转 $\beta$，$T = (t_x, 0, t_z)^T$。标定图像坐标记为 $\mathbf{x}_l, \mathbf{x}_r$。

---

### 3.1 符号说明

- $\mathbf{X}_l, \mathbf{X}_r$：3D 点在左、右相机坐标系下的坐标
- $R$：右相机相对于左相机的旋转，$R = R_y(\beta)$（绕 y 轴）
- $T$：右相机相对于左相机的平移，$T = (t_x, 0, t_z)^T$
- $\beta$：绕 y 轴的旋转角
- $t_x, t_z$：平移在 x、z 方向的分量（仅方向可恢复，尺度不可恢复）
- $\mathbf{x}_l, \mathbf{x}_r$：左、右图像的标定（归一化）齐次坐标
- $\mathbf{e}_l, \mathbf{e}_r$：左、右图像中的对极点

---

### 3.2 (i) 本质矩阵 $E$

对极约束为 $\mathbf{x}_r^T E \mathbf{x}_l = 0$，本质矩阵 $E = [T]_\times R$。

**Step 1：反对称矩阵 $[T]_\times$**

$T = (t_x, 0, t_z)^T$，故
$$
[T]_\times = \begin{pmatrix} 0 & -t_z & 0 \\ t_z & 0 & -t_x \\ 0 & t_x & 0 \end{pmatrix}
$$

**Step 2：旋转矩阵 $R = R_y(\beta)$**

$$
R = \begin{pmatrix} \cos\beta & 0 & \sin\beta \\ 0 & 1 & 0 \\ -\sin\beta & 0 & \cos\beta \end{pmatrix}
$$

**Step 3：计算 $E = [T]_\times R$**

逐行计算：
- 第 1 行：$(0, -t_z, 0) R = (0, -t_z, 0)$
- 第 2 行：$(t_z, 0, -t_x) R = (t_z\cos\beta + t_x\sin\beta,\; 0,\; t_z\sin\beta - t_x\cos\beta)$
- 第 3 行：$(0, t_x, 0) R = (0, t_x, 0)$

故
$$
\boxed{
E = \begin{pmatrix}
0 & -t_z & 0 \\
t_z\cos\beta + t_x\sin\beta & 0 & t_z\sin\beta - t_x\cos\beta \\
0 & t_x & 0
\end{pmatrix}
}
$$

**注**：$E$ 只依赖平移方向 $(t_x, t_z)$ 与 $\beta$，平移模长不可恢复。

---

### 3.3 (ii) 左、右对极点

- **右对极点 $\mathbf{e}_r$**：左相机光心在右图中的投影。左光心在右系下为 $T$，故 $\mathbf{e}_r \sim T$。
- **左对极点 $\mathbf{e}_l$**：右相机光心在左图中的投影。右光心在左系下为 $-R^T T$，故 $\mathbf{e}_l \sim -R^T T$。

$$
R^T = \begin{pmatrix} \cos\beta & 0 & -\sin\beta \\ 0 & 1 & 0 \\ \sin\beta & 0 & \cos\beta \end{pmatrix}
$$

$$
R^T T = \begin{pmatrix} t_x\cos\beta - t_z\sin\beta \\ 0 \\ t_x\sin\beta + t_z\cos\beta \end{pmatrix}
$$

故
$$
\boxed{
\mathbf{e}_r \sim \begin{pmatrix} t_x \\ 0 \\ t_z \end{pmatrix}, \qquad
\mathbf{e}_l \sim \begin{pmatrix} t_z\sin\beta - t_x\cos\beta \\ 0 \\ -t_x\sin\beta - t_z\cos\beta \end{pmatrix}
}
$$

---

### 3.4 (iii) 从对极点恢复未知量

**提示**：令 $(t_x, t_z) = (\cos\alpha, \sin\alpha)$，将平移方向参数化为角度 $\alpha$。未知量为 $\alpha$ 与 $\beta$（共 2 个）。

**Step 1：从右对极点得 $\alpha$**

$\mathbf{e}_r \sim (\cos\alpha, 0, \sin\alpha)^T$。在标定坐标系下，对极点方向即该向量，故
$$
\alpha = \arctan2(t_z, t_x) = \arctan2(\sin\alpha, \cos\alpha)
$$
即从 $\mathbf{e}_r$ 可直接读出 $\alpha$。

**Step 2：从左对极点得 $\alpha + \beta$**

$$
\mathbf{e}_l \sim \begin{pmatrix} t_z\sin\beta - t_x\cos\beta \\ 0 \\ -t_x\sin\beta - t_z\cos\beta \end{pmatrix}
= \begin{pmatrix} \sin\alpha\sin\beta - \cos\alpha\cos\beta \\ 0 \\ -\cos\alpha\sin\beta - \sin\alpha\cos\beta \end{pmatrix}
= \begin{pmatrix} -\cos(\alpha+\beta) \\ 0 \\ -\sin(\alpha+\beta) \end{pmatrix}
$$

故 $\mathbf{e}_l \sim (\cos(\alpha+\beta), 0, \sin(\alpha+\beta))^T$（差一符号等价），即 $\mathbf{e}_l$ 的方向角为 $\alpha + \beta$，可求出 $\alpha + \beta$。

**Step 3：恢复 $\beta$**

$$
\boxed{\beta = (\alpha + \beta) - \alpha}
$$

由 $\mathbf{e}_r$ 得 $\alpha$，由 $\mathbf{e}_l$ 得 $\alpha + \beta$，相减即得 $\beta$。平移方向 $(t_x, t_z)$ 由 $\alpha$ 确定，因此两个对极点足以恢复全部外参（平移方向 + 旋转角 $\beta$）。

---

## 4. Gaussian Splatting (20 pts)

单 3D 高斯场景，针孔相机内参 $K$、外参 $(R, \mathbf{t})$，世界点 $\mathbf{X}$ 映射到相机系 $\mathbf{X}_c = R\mathbf{X} + \mathbf{t}$。高斯参数 $\theta = (\boldsymbol{\mu}, \Sigma, \mathbf{c}, \alpha)$，投影中心 $\tilde{\boldsymbol{\mu}} = \pi(R\boldsymbol{\mu} + \mathbf{t})$，$\boldsymbol{\delta} = \mathbf{p} - \tilde{\boldsymbol{\mu}}$，$\Sigma_c = R\Sigma R^T$，$S = J \Sigma_c J^T$，$g(\mathbf{p}) = \exp(-\frac{1}{2}\boldsymbol{\delta}^T S^{-1}\boldsymbol{\delta})$，$a(\mathbf{p}) = \alpha g(\mathbf{p})$，$\hat{I}(\mathbf{p}) = a(\mathbf{p})\mathbf{c}$，$L(\mathbf{p}) = \frac{1}{2}\|\hat{I}(\mathbf{p}) - I(\mathbf{p})\|^2$。

**重要**：计算 $\nabla_{\boldsymbol{\mu}} L$ 时，将 $J$ 和 $S$ 视为与 $\boldsymbol{\mu}$ 无关，即 $\frac{\partial J}{\partial \boldsymbol{\mu}} = 0$，$\frac{\partial S}{\partial \boldsymbol{\mu}} = 0$。

---

### 4.1 符号说明

- $K$：内参矩阵，$K = \begin{pmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{pmatrix}$
- $R, \mathbf{t}$：外参（world-to-camera）
- $\boldsymbol{\mu}$：3D 高斯均值
- $\Sigma$：3D 高斯协方差（对称正定）
- $\mathbf{c}$：RGB 颜色
- $\alpha$：不透明度
- $\pi(\cdot)$：针孔投影
- $\tilde{\boldsymbol{\mu}}$：高斯中心在图像上的投影
- $\boldsymbol{\delta}$：像素 $\mathbf{p}$ 与 $\tilde{\boldsymbol{\mu}}$ 的差
- $\Sigma_c$：相机系下的协方差
- $J$：$\pi$ 在 $\mathbf{X}_c = R\boldsymbol{\mu} + \mathbf{t}$ 处的 Jacobian
- $S$：投影后的 2D 协方差
- $g, a, \hat{I}, L$：高斯 footprint、不透明度贡献、渲染颜色、损失

---

### 4.2 (i) 投影 Jacobian $J$ 与 2D 协方差 $S$（5 pts）

**针孔投影**：$\mathbf{X}_c = (x, y, z)^T \mapsto \pi(\mathbf{X}_c) = (f_x x/z + c_x,\; f_y y/z + c_y)^T$。

记 $u = f_x x/z + c_x$，$v = f_y y/z + c_y$，则
$$
\frac{\partial u}{\partial x} = \frac{f_x}{z},\quad \frac{\partial u}{\partial y} = 0,\quad \frac{\partial u}{\partial z} = -\frac{f_x x}{z^2}
$$
$$
\frac{\partial v}{\partial x} = 0,\quad \frac{\partial v}{\partial y} = \frac{f_y}{z},\quad \frac{\partial v}{\partial z} = -\frac{f_y y}{z^2}
$$

故
$$
\boxed{
J = \frac{\partial \pi}{\partial \mathbf{X}_c} = \begin{pmatrix} f_x/z & 0 & -f_x x/z^2 \\ 0 & f_y/z & -f_y y/z^2 \end{pmatrix}
}
$$

其中 $(x, y, z)^T = R\boldsymbol{\mu} + \mathbf{t}$，$J$ 在 $\mathbf{X}_c = R\boldsymbol{\mu} + \mathbf{t}$ 处取值。

**2D 协方差**：
$$
\boxed{
S = J \Sigma_c J^T = J (R \Sigma R^T) J^T
}
$$

即 $S$ 由 $J$、$R$、$\Sigma$ 显式给出。

---

### 4.3 (ii) 对 $\boldsymbol{\mu}$ 的梯度 $\nabla_{\boldsymbol{\mu}} L$（10 pts）

链式法则（$J$、$S$ 与 $\boldsymbol{\mu}$ 无关）：
$$
L = \frac{1}{2}\|\hat{I} - I\|^2,\quad \hat{I} = a\mathbf{c},\quad a = \alpha g,\quad g = \exp\left(-\frac{1}{2}\boldsymbol{\delta}^T S^{-1}\boldsymbol{\delta}\right),\quad \boldsymbol{\delta} = \mathbf{p} - \tilde{\boldsymbol{\mu}},\quad \tilde{\boldsymbol{\mu}} = \pi(R\boldsymbol{\mu} + \mathbf{t})
$$

**Step 1**：$\frac{\partial L}{\partial \hat{I}} = \hat{I} - I$

**Step 2**：$\frac{\partial L}{\partial a} = \frac{\partial L}{\partial \hat{I}} \cdot \frac{\partial \hat{I}}{\partial a} = (\hat{I} - I)^T \mathbf{c}$

**Step 3**：$\frac{\partial g}{\partial \boldsymbol{\delta}} = g \cdot (-S^{-1}\boldsymbol{\delta})$，故 $\frac{\partial L}{\partial \boldsymbol{\delta}} = (\hat{I}-I)^T \mathbf{c} \cdot \alpha \cdot g \cdot (-S^{-1}\boldsymbol{\delta}) = -(\hat{I}-I)^T \mathbf{c} \cdot \alpha g \cdot S^{-1}\boldsymbol{\delta}$

**Step 4**：$\boldsymbol{\delta} = \mathbf{p} - \tilde{\boldsymbol{\mu}}$，故 $\frac{\partial \boldsymbol{\delta}}{\partial \tilde{\boldsymbol{\mu}}} = -I_2$，$\frac{\partial \tilde{\boldsymbol{\mu}}}{\partial \mathbf{X}_c} = J$，$\frac{\partial \mathbf{X}_c}{\partial \boldsymbol{\mu}} = R$。

故
$$
\frac{\partial L}{\partial \boldsymbol{\mu}} = R^T J^T \frac{\partial L}{\partial \tilde{\boldsymbol{\mu}}} = R^T J^T \left(-(-(\hat{I}-I)^T \mathbf{c} \cdot \alpha g \cdot S^{-1}\boldsymbol{\delta})\right) = (\hat{I}-I)^T \mathbf{c} \cdot \alpha g \cdot R^T J^T S^{-1}\boldsymbol{\delta}
$$

即
$$
\boxed{
\nabla_{\boldsymbol{\mu}} L(\mathbf{p}) = \bigl((\hat{I}(\mathbf{p}) - I(\mathbf{p}))^T \mathbf{c}\bigr) \cdot \alpha \cdot g(\mathbf{p}) \cdot R^T J^T S^{-1} \boldsymbol{\delta}
}
$$

---

### 4.4 (iii) 对 $\Sigma$ 的梯度 $\nabla_{\Sigma} L$（5 pts）

$L$ 通过 $S = J R \Sigma R^T J^T$ 依赖 $\Sigma$。

**Step 1**：$\frac{\partial L}{\partial a} = (\hat{I}-I)^T \mathbf{c}$，$\frac{\partial a}{\partial g} = \alpha$，$\frac{\partial g}{\partial (-\frac{1}{2}\boldsymbol{\delta}^T S^{-1}\boldsymbol{\delta})} = g$。

对 $S$ 的导数：$\frac{\partial}{\partial S}\bigl(\boldsymbol{\delta}^T S^{-1}\boldsymbol{\delta}\bigr) = -S^{-1}\boldsymbol{\delta}\boldsymbol{\delta}^T S^{-1}$（$S$ 对称），故
$$
\frac{\partial g}{\partial S} = g \cdot \left(-\frac{1}{2}\right) \cdot \left(-S^{-1}\boldsymbol{\delta}\boldsymbol{\delta}^T S^{-1}\right) = \frac{g}{2} S^{-1}\boldsymbol{\delta}\boldsymbol{\delta}^T S^{-1}
$$

$$
\frac{\partial L}{\partial S} = (\hat{I}-I)^T \mathbf{c} \cdot \alpha \cdot \frac{g}{2} S^{-1}\boldsymbol{\delta}\boldsymbol{\delta}^T S^{-1}
$$

**Step 2**：$S = (JR) \Sigma (R^T J^T)$。对 $L = f(S)$ 且 $S = A\Sigma B$，有 $\frac{\partial L}{\partial \Sigma} = A^T \frac{\partial L}{\partial S} B^T$。令 $A = JR$，$B = R^T J^T$，则
$$
\frac{\partial L}{\partial \Sigma} = R^T J^T \frac{\partial L}{\partial S} J R
$$

代入 $\frac{\partial L}{\partial S}$：
$$
\nabla_{\Sigma} L = R^T J^T \left((\hat{I}-I)^T \mathbf{c} \cdot \alpha \cdot \frac{g}{2} S^{-1}\boldsymbol{\delta}\boldsymbol{\delta}^T S^{-1}\right) J R
$$

记 $k = (\hat{I}-I)^T \mathbf{c} \cdot \alpha \cdot \frac{g}{2}$，则
$$
\nabla_{\Sigma} L = k \cdot R^T J^T S^{-1}\boldsymbol{\delta}\boldsymbol{\delta}^T S^{-1} J R
$$

$\Sigma$ 对称，将梯度对称化：
$$
\boxed{
\nabla_{\Sigma} L(\mathbf{p}) = \frac{1}{2}\bigl((\hat{I}(\mathbf{p}) - I(\mathbf{p}))^T \mathbf{c}\bigr) \alpha \, g(\mathbf{p}) \cdot R^T J^T S^{-1} \boldsymbol{\delta} \boldsymbol{\delta}^T S^{-1} J R
}
$$

$R^T J^T S^{-1}\boldsymbol{\delta}\boldsymbol{\delta}^T S^{-1} J R$ 为对称矩阵，若需保证 $\nabla_{\Sigma} L$ 对称，可取 $\frac{1}{2}(\nabla_{\Sigma} L + (\nabla_{\Sigma} L)^T)$。
