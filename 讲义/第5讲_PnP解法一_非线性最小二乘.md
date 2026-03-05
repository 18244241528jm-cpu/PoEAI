# 第 5 讲：PnP 的解法一 —— 非线性最小二乘

**对应讲义**：03_single_view_pose_estimation.pdf 第 27–36 页  
**本讲目标**：掌握用"最小化重投影误差"解 PnP 的思路，以及 Gauss-Newton、Levenberg-Marquardt 和实现细节。

---

## 先想清楚：这讲到底在解什么问题？

上一讲定义了 PnP 问题：已知一堆 3D 点和它们的 2D 像素，求相机位姿。这讲给出第一种解法思路：把"真实像素位置"和"用当前猜测的 $(R,\mathbf{t})$ 算出的投影位置"之间的差距定义为"重投影误差"，然后不断调整 $(R,\mathbf{t})$ 使这个误差越来越小。这就是"非线性最小二乘"——本质上是在做"不断试错、逐步逼近"的迭代优化。

---

## 和前面课程的关系

- **第 4 讲**定义了 PnP 问题本身：给定 $n$ 个 3D–2D 对应，求 $(R,\mathbf{t})$。
- **第 5 讲（本讲）**= **解法一**：基于优化的思路——构造重投影误差，用非线性最小二乘（Gauss-Newton / LM）迭代求解。
- **第 6 讲**= **解法二**：基于最小子集的思路——P3P 只用 3 个点求解析解，再配合 RANSAC 剔除外点。

两种思路**互补**：实际系统中往往先用 P3P + RANSAC 得到一个粗略位姿和内点集合，再在内点上跑非线性最小二乘做精化。

---

## 1. 最小化重投影误差（Solution 1: Minimizing reprojection error）

> 📖 **对照 PPT**：打开 03_single_view_pose_estimation.pdf **第 27 页** "Solution 1: minimizing reprojection error"，有残差定义的示意。

### 1.1 残差定义

设第 $i$ 个 3D 点在世界系下为 $\mathbf{X}_i$，其图像观测（像素）为 $\mathbf{x}_i = (u_i, v_i)^\top$。若当前位姿为 $(R, \mathbf{t})$，内参为 $K$，则**预测的投影**为：

$$
\mathbf{x}_i^{\text{pred}} = \pi\big( K(R\mathbf{X}_i + \mathbf{t}) \big)
$$

其中 $\pi$ 表示**齐次到非齐次**的投影：$(x, y, z)^\top \mapsto (x/z, y/z)^\top$（即除以深度 $z$ 得到像素坐标）。**重投影残差**（2D 向量）为：

$$
\mathbf{r}_i(R, \mathbf{t}) = \mathbf{x}_i - \mathbf{x}_i^{\text{pred}} = \mathbf{x}_i - \pi\big( K(R\mathbf{X}_i + \mathbf{t}) \big)
$$

### 1.2 目标函数

**非线性最小二乘**：最小化所有点上的残差平方和，即

$$
\min_{R,\,\mathbf{t}} \quad F(\mathbf{a}) = \frac{1}{2} \sum_{i=1}^{N} \big\| \mathbf{r}_i(\mathbf{a}) \big\|^2
$$

其中 $\mathbf{a}$ 是把 $(R, \mathbf{t})$ 参数化后的向量（如旋转向量 + 平移，共 6 维）。**各项含义**：

| 符号 | 含义 |
|------|------|
| $\mathbf{a}$ | 参数向量，在 PnP 中即 $(R, \mathbf{t})$ 的参数化 |
| $\mathbf{r}_i(\mathbf{a})$ | 第 $i$ 个点的残差（观测像素 − 投影像素） |
| $\lVert \mathbf{r}_i \rVert^2$ | 残差平方，只关心误差大小、不关心正负 |
| $\frac{1}{2}$ | 常数，求导时与平方的 2 抵消，便于推导 |
| $\sum_i$ | 对所有观测点求和 |

**为什么要平方？** (1) 正负误差都贡献正值，不会互相抵消；(2) 大误差被放大，优化更偏向减小大误差；(3) 可导，便于迭代。

这里 $R \in SO(3)$，$\mathbf{t} \in \mathbb{R}^3$。因为投影 $\pi$ 与 $R\mathbf{X}_i + \mathbf{t}$ 都是**非线性的**，所以 $F(\mathbf{a})$ 关于 $\mathbf{a}$ **非线性**，没有像线性最小二乘那样的"一步解析解"，需要**迭代优化**。

---

## 2. 非线性最小二乘 vs 线性最小二乘

> 📖 **对照 PPT**：打开 03_single_view_pose_estimation.pdf **第 28 页** "Non-linear Least Square"，对比了线性最小二乘（有解析解/正规方程）和非线性最小二乘（需迭代）。

### 2.1 线性最小二乘（回顾）

**问题**：给定矩阵 $A$ 和向量 $\mathbf{b}$，求 $\mathbf{x}$ 使 $\lVert A\mathbf{x} - \mathbf{b} \rVert^2$ 最小。

**解**：对 $\mathbf{x}$ 求导并令其为零，得**正规方程** $A^\top A \mathbf{x} = A^\top \mathbf{b}$。若 $A^\top A$ 可逆，则 $\mathbf{x} = (A^\top A)^{-1} A^\top \mathbf{b}$，**一步得解**，无需迭代。

**直观**：$A\mathbf{x} = \mathbf{b}$ 是线性方程组；当无精确解时，最小二乘求的是"最接近"的解，即让误差向量 $A\mathbf{x} - \mathbf{b}$ 的模长最小。

### 2.2 非线性最小二乘

**问题**：求 $\mathbf{a}$ 使 $\lVert \mathbf{r}(\mathbf{a}) \rVert^2$ 最小，其中 $\mathbf{r}(\mathbf{a})$ 是 $\mathbf{a}$ 的**非线性**函数。

**难点**：对 $\mathbf{a}$ 求导后得到的是非线性方程，一般**没有解析解**。

**思路**：在当前位置 $\mathbf{a}$ 做**线性近似** $\mathbf{r}(\mathbf{a}+\Delta\mathbf{a}) \approx \mathbf{r} + J\Delta\mathbf{a}$，把问题变成"求 $\Delta\mathbf{a}$ 使 $\lVert \mathbf{r} + J\Delta\mathbf{a} \rVert^2$ 最小"——这又回到了**线性**最小二乘，可一步解出 $\Delta\mathbf{a}$，再更新 $\mathbf{a}$，迭代进行。

### 2.3 为何 PnP 是非线性的？

PnP 的残差 $\mathbf{r}_i = \mathbf{x}_i - \pi(K(R\mathbf{X}_i + \mathbf{t}))$ 中：
- **旋转** $R\mathbf{X}_i$：含 $\sin$、$\cos$，对 $R$ 非线性；
- **投影** $\pi(x,y,z) = (x/z, y/z)$：含**除法**，对 $z$ 非线性。

因此 $\mathbf{r}(\mathbf{a})$ 关于 $\mathbf{a}$ 非线性，属于**非线性最小二乘**。

---

**🧭 目标函数写好了，但它是非线性的，没法一步解出来。下面的核心思路是：在当前位置做线性近似，解一个简单的局部问题，走一步，再近似，再走一步……**

---

## 3. 线性化（Linearization）

> 📖 **对照 PPT**：打开 03_single_view_pose_estimation.pdf **第 29–30 页** "Linearization"，展示了泰勒展开后如何变成局部线性最小二乘。

### 3.1 为何用泰勒展开？

目标函数 $F(\mathbf{a}) = \frac{1}{2}\sum_i \lVert \mathbf{r}_i(\mathbf{a}) \rVert^2$ 对 $\mathbf{a}$ 是**非线性的**（投影里有旋转、除法 $x/z,\, y/z$），无法直接求解析解。因此采用**迭代**：在当前的 $\mathbf{a}$ 附近，用**一阶泰勒展开**把 $\mathbf{r}(\mathbf{a})$ 近似成**线性函数**，解一个线性问题得到增量 $\Delta\mathbf{a}$，再更新 $\mathbf{a} \leftarrow \mathbf{a} + \Delta\mathbf{a}$，反复迭代直到收敛。

### 3.2 泰勒展开公式怎么来的？

在当前参数 $\mathbf{a}$ 处，将残差**一阶泰勒展开**：

$$
\mathbf{r}(\mathbf{a} + \Delta\mathbf{a}) \approx \mathbf{r}(\mathbf{a}) + J(\mathbf{a}) \, \Delta\mathbf{a}
$$

**推导**：标量函数 $f(x)$ 在 $x_0$ 处的一阶泰勒是 $f(x_0 + \Delta x) \approx f(x_0) + f'(x_0)\,\Delta x$。当 $\mathbf{r}$ 是向量、$\mathbf{a}$ 是向量时，标量导数 $f'(x_0)$ 变成**雅可比矩阵** $J(\mathbf{a}) = \frac{\partial \mathbf{r}}{\partial \mathbf{a}}$，于是上式就是向量版的一阶泰勒。

**雅可比矩阵 $J$ 是什么？**

$J$ 是残差 $\mathbf{r}$ 对参数 $\mathbf{a}$ 的偏导矩阵。设 $\mathbf{r}$ 有 $m$ 维（如 $2N$ 个残差分量，$N$ 个点每点 2 维），$\mathbf{a}$ 有 $n$ 维（如 6 维：旋转 3 + 平移 3），则 $J$ 是 $m \times n$ 矩阵：

$$
J_{ij} = \frac{\partial r_i}{\partial a_j}
$$

即第 $i$ 行对 $i$ 个残差分量，第 $j$ 列对 $j$ 个参数分量。$J$ 描述的是：**参数 $a_j$ 微小变化时，残差 $r_i$ 会如何变化**。

**直觉**：在 $\mathbf{a}$ 附近，用"当前值 + 线性修正"近似新值；$J\,\Delta\mathbf{a}$ 表示参数从 $\mathbf{a}$ 变到 $\mathbf{a}+\Delta\mathbf{a}$ 时，残差的线性变化量。

### 3.3 变成局部线性最小二乘

于是

$$
\big\| \mathbf{r}(\mathbf{a} + \Delta\mathbf{a}) \big\|^2 \approx \big\| \mathbf{r}(\mathbf{a}) + J \Delta\mathbf{a} \big\|^2
$$

右边 $\lVert \mathbf{r} + J\Delta\mathbf{a} \rVert^2$ 关于 $\Delta\mathbf{a}$ 是**二次函数**，最小化它等价于解**线性**方程组（正规方程）。"线性"指的是残差 $\mathbf{r} + J\Delta\mathbf{a}$ 对 $\Delta\mathbf{a}$ 是线性的——没有 $\Delta a_i \cdot \Delta a_j$ 之类的交叉项在残差里。因此这是**线性最小二乘**：在局部用线性模型近似非线性残差，求**增量** $\Delta\mathbf{a}$。

**直觉**：想象你站在山上，想找最低点。非线性问题就像地形起伏不平，你看不到全貌。泰勒展开的意思是：在你脚下画一个"小平面"来近似真实地形，在这个小平面上找最低点很容易（线性最小二乘），走到那里，再画一个新的小平面，如此反复，就能一步步走到谷底。

---

## 4. 高斯-牛顿法（Gauss-Newton Method）

> 📖 **对照 PPT**：打开 03_single_view_pose_estimation.pdf **第 31 页** "Gauss-Newton Method"，列出了解析 vs 数值雅可比的选择。

### 4.1 迭代格式

每步做三件事，循环执行直到收敛：

#### Step 1：线性化

在当前参数 $\mathbf{a}$ 处，计算：
- **残差** $\mathbf{r}(\mathbf{a})$：所有点的"观测 − 预测"拼成的向量；
- **雅可比** $J = \frac{\partial \mathbf{r}}{\partial \mathbf{a}}$：残差对参数的偏导矩阵。

含义：用当前 $\mathbf{a}$ 和 $J$ 在局部构造线性近似 $\mathbf{r}(\mathbf{a}+\Delta\mathbf{a}) \approx \mathbf{r} + J\Delta\mathbf{a}$。

#### Step 2：解局部最小二乘

最小化近似后的残差平方和：
$$
\min_{\Delta\mathbf{a}} \quad \lVert \mathbf{r} + J \Delta\mathbf{a} \rVert^2
$$

**正规方程怎么来的？**

记 $\mathbf{e} = \mathbf{r} + J\Delta\mathbf{a}$，目标是最小化 $\mathbf{e}^\top \mathbf{e}$。对 $\Delta\mathbf{a}$ 的每个分量求偏导并令其为零。用矩阵求导法则：

$$
\frac{\partial}{\partial \Delta\mathbf{a}} \big( \mathbf{r} + J\Delta\mathbf{a} \big)^\top \big( \mathbf{r} + J\Delta\mathbf{a} \big) = 0
$$

展开得 $\mathbf{e}^\top \mathbf{e} = \mathbf{r}^\top\mathbf{r} + 2\mathbf{r}^\top J\Delta\mathbf{a} + \Delta\mathbf{a}^\top J^\top J \Delta\mathbf{a}$。对 $\Delta\mathbf{a}$ 求导：
$\frac{\partial}{\partial \Delta\mathbf{a}}(\mathbf{e}^\top\mathbf{e}) = 2 J^\top \mathbf{r} + 2 J^\top J \Delta\mathbf{a}$。令其为零：

$$
J^\top J \, \Delta\mathbf{a} = -J^\top \mathbf{r}
$$

这就是**正规方程**。形式与线性最小二乘 $A^\top A \mathbf{x} = A^\top \mathbf{b}$ 一致（这里 $A=J$，$\mathbf{b}=-\mathbf{r}$，$\mathbf{x}=\Delta\mathbf{a}$）。

**求解**：若 $J^\top J$ 可逆，则 $\Delta\mathbf{a} = -(J^\top J)^{-1} J^\top \mathbf{r}$。实践中常用 Cholesky 分解或 SVD，避免显式求逆。

**直觉**：$\Delta\mathbf{a}$ 是"在当前线性近似下，能让残差下降最快的方向"。

#### Step 3：更新

$$
\mathbf{a} \leftarrow \mathbf{a} + \Delta\mathbf{a}
$$

或加步长 $\alpha \in (0, 1]$：$\mathbf{a} \leftarrow \mathbf{a} + \alpha \Delta\mathbf{a}$（当步长过大、残差反而上升时，可缩小 $\alpha$ 做线搜索）。

**收敛判断**：重复上述三步，直到 $\lVert \Delta\mathbf{a} \rVert$ 或 $\lVert \mathbf{r} \rVert$ 足够小，即参数几乎不再变化或残差已足够小。

#### 简单 1D 例子（体会迭代）

设 $r(a) = a^2 - 2$（标量），目标是求 $a$ 使 $r(a)^2$ 最小，即 $a^2 = 2$，解为 $a^* = \sqrt{2}$。在 $a=1$ 处线性化：$r(1)= -1$，$J = r'(1) = 2$，故 $r(1+\Delta a) \approx -1 + 2\Delta a$。最小化 $(-1+2\Delta a)^2$ 得 $\Delta a = 1/2$，于是 $a \leftarrow 1 + 0.5 = 1.5$。再在 $a=1.5$ 处线性化，得 $\Delta a \approx 0.083$，$a \leftarrow 1.583$……如此迭代，$a$ 会逐渐逼近 $\sqrt{2} \approx 1.414$。这就是"线性化 → 解局部问题 → 更新"的直观过程。

### 4.2 雅可比：解析 vs 数值

- **解析雅可比**：写出 $\mathbf{r}_i$ 对 $R, \mathbf{t}$（或对旋转向量、四元数等参数）的偏导公式，直接算 $J$。**收敛快、精度高**，推荐在 PnP 中使用。
- **数值雅可比**：用差分近似，例如 $\frac{\partial r}{\partial a_j} \approx \frac{r(\mathbf{a} + \epsilon \mathbf{e}_j) - r(\mathbf{a})}{\epsilon}$。实现简单，但慢且对 $\epsilon$ 敏感。

讲义强调：**In practice, converge much faster if analytical Jacobian is available.**

---

## 5. 列文伯格-马夸尔特法（Levenberg-Marquardt, LM）

> 📖 **对照 PPT**：打开 03_single_view_pose_estimation.pdf **第 32 页** "Levenberg-Marquardt (LM)"，强调 "The step should not be too large"。

### 5.1 动机：步长不要太大

高斯-牛顿的步 $\Delta\mathbf{a}$ 可能过大（尤其远离解时），导致线性近似失效、发散。**LM** 在步长上做**阻尼**，介于 Gauss-Newton 与梯度下降之间：

- 解的是**阻尼正规方程**：
  $$
  (J^\top J + \lambda I) \, \Delta\mathbf{a} = -J^\top \mathbf{r}
  $$
  或 $(J^\top J + \lambda \operatorname{diag}(J^\top J)) \Delta\mathbf{a} = -J^\top \mathbf{r}$（带尺度时常用后者）。

- $\lambda$ 大：更接近梯度下降，步小、稳。
- $\lambda$ 小：更接近 Gauss-Newton，步大、收敛快。

**一句话区别**：Gauss-Newton 完全信任脚下的"小平面"，可能步子太大走过头；LM 多了一个"刹车"（阻尼项 $\lambda$），步子太大时自动减速。

### 5.2 实践

根据本轮残差是否下降，增大或减小 $\lambda$，再解上式得到 $\Delta\mathbf{a}$，更新 $\mathbf{a}$。很多库（如 Ceres、g2o、scipy.optimize.least_squares）默认或可选 LM。

---

## 6. 实践中怎么做（In practice）

> 📖 **对照 PPT**：打开 03_single_view_pose_estimation.pdf **第 33–34 页** "In practice"，强调解析雅可比收敛更快。

### 6.1 谁负责什么？

| 你实现 | 库负责 |
|--------|--------|
| **残差函数** $\mathbf{r}(\mathbf{a})$：给定 $\mathbf{a}$、3D 点、2D 点、$K$，算"观测 − 投影" | GN/LM 迭代、解正规方程、收敛判断、步长/阻尼 |
| **（可选）解析雅可比** $J$：$\mathbf{r}$ 对 $\mathbf{a}$ 的偏导 | 若不提供，用数值差分近似 $J$ |

**注意**：泰勒展开 $\mathbf{r}(\mathbf{a}+\Delta\mathbf{a}) \approx \mathbf{r} + J\Delta\mathbf{a}$ 是库内部用的**线性近似**，不是残差函数的定义。残差函数 $\mathbf{r}(\mathbf{a})$ 必须由你根据问题写出，例如 PnP 中 $\mathbf{r}_i = \mathbf{x}_i - \pi(K(R\mathbf{X}_i + \mathbf{t}))$。

### 6.2 参数化：$\mathbf{a}$ 里装的是什么？

$\mathbf{a}$ 是 $(R, \mathbf{t})$ 的**参数化**，不是 $R$ 的 9 个元素。常用形式：

- **轴角 + 平移**：$\mathbf{a} = (\boldsymbol{\omega}^\top, \mathbf{t}^\top)^\top$，6 维；
- **四元数 + 平移**：$\mathbf{a} = (\mathbf{q}^\top, \mathbf{t}^\top)^\top$，7 维（$\mathbf{q}$ 需单位约束）。

每次迭代中，库调用你的残差函数时传入当前 $\mathbf{a}$，你需在函数内部把 $\mathbf{a}$ 转成 $R, \mathbf{t}$（如用 Rodrigues 或四元数乘法），再算投影和残差。

### 6.3 初值是谁的初值？

初值是**优化参数** $\mathbf{a}$ 的初始猜测 $\mathbf{a}_0$。库从 $\mathbf{a}_0$ 开始迭代，逐步逼近使 $\lVert \mathbf{r}(\mathbf{a}) \rVert^2$ 最小的 $\mathbf{a}$。初值来源见第 8.2 节。

### 6.4 库的输入输出（scipy vs OpenCV）

**scipy.optimize.least_squares**（通用非线性最小二乘）：

| 输入 | 含义 |
|------|------|
| `fun` | 残差函数 `fun(x, *args) -> array`，返回残差向量 |
| `x0` | 初值，如 `[ωx, ωy, ωz, tx, ty, tz]` |
| `args` | 额外数据，如 `(points_3d, points_2d, K)` |
| `method` | `'lm'` 或 `'trf'` |
| `jac` | 可选，解析雅可比函数 |

| 输出 | 含义 |
|------|------|
| `sol.x` | 优化后的参数向量 |
| `sol.success` | 是否收敛 |
| `sol.fun` | 最终残差向量 |

**OpenCV cv2.solvePnP**（PnP 专用，内部已实现重投影残差和 LM）：

| 输入 | 含义 |
|------|------|
| `objectPoints` | 3D 点，shape `(N, 3)` |
| `imagePoints` | 2D 点，shape `(N, 2)` |
| `cameraMatrix` | 内参 $K$ |
| `rvec`, `tvec` | 初值（轴角 + 平移），`useExtrinsicGuess=True` 时传入 |

| 输出 | 含义 |
|------|------|
| `rvec`, `tvec` | 优化后的旋转向量和平移向量 |
| `cv2.Rodrigues(rvec)` | 轴角 → 旋转矩阵 $R$ |

**对比**：scipy 需自己写残差和投影；OpenCV 已封装，只需给点对和 $K$，但灵活性较低。

### 6.5 代码示例：scipy 的用法模式

以指数拟合 $y \approx a e^{bx}$ 为例，体会"残差函数 + 初值 + 库调用"的模式：

```python
from scipy.optimize import least_squares

def residual(p, x, y):           # 残差函数：必须自己定义
    a, b = p
    return a * np.exp(b * x) - y  # 预测 − 观测

x = np.array([0.0, 0.5, 1.0, 1.5, 2.0])
y = np.array([1.0, 1.7, 2.9, 4.8, 8.2])

sol = least_squares(residual, x0=[1.0, 1.0], args=(x, y), method='lm')
print("优化后 a, b =", sol.x)   # 输出参数
```

对应到 PnP：把 `p` 换成位姿参数 $\mathbf{a}$，把 `residual` 换成重投影残差，`args` 传入 `(points_3d, points_2d, K)`，其余流程相同。

**OpenCV PnP 示例**（无需写残差，直接调用）：

```python
success, rvec, tvec = cv2.solvePnP(
    objectPoints=pts_3d,    # (N, 3) 世界坐标
    imagePoints=pts_2d,     # (N, 2) 像素坐标
    cameraMatrix=K,
    distCoeffs=None,
    flags=cv2.SOLVEPNP_ITERATIVE  # 内部用 LM
)
R, _ = cv2.Rodrigues(rvec)  # 轴角 → 3×3 旋转矩阵
```

---

## 7. 一个注意事项（One Caveat）：保持 R 合法

> 📖 **对照 PPT**：打开 03_single_view_pose_estimation.pdf **第 35 页** "One Caveat — Need special care to make R valid"。

优化变量必须是**合法的旋转表示**：

- 若用 **旋转向量** $\boldsymbol{\omega} \in \mathbb{R}^3$：更新时用 Rodrigues 转成 $R$，自然保证 $R \in SO(3)$。
- 若用 **四元数**：约束单位长度，更新后归一化。
- **不要**把 $R$ 的 9 个元素当自由变量无约束优化，否则结果可能不是正交矩阵（$\det \neq 1$ 或 $R^\top R \neq I$）。

因此：**Need special care to make R valid**——用合适的参数化并在每步得到合法的 $R$。

---

**🧭 算法本身讲完了。但实际用起来有两个致命问题——外点和初值。这直接引出了下一讲的 P3P + RANSAC。**

---

## 8. 难点：外点与初值（Issues: outliers and initialization）

> 📖 **对照 PPT**：打开 03_single_view_pose_estimation.pdf **第 36 页** "Issue: outliers and initialization"，明确指出两个关键问题。

### 8.1 外点（Outliers）

若部分 2D–3D 对应是**错的**（误匹配、误检测），其残差会很大，平方后主导目标函数，导致估计的 $(R, \mathbf{t})$ 被"拉偏"。**非线性最小二乘本身不抗外点**。

**应对**：下一讲用 **P3P + RANSAC**：用最小子集求位姿、用内点投票，再在**内点**上做非线性优化（或直接采用 RANSAC 给出的位姿）。

### 8.2 初值（Initialization）

非线性优化**依赖初值**：初值太差可能收敛到局部最优甚至不收敛。PnP 的初值可从以下方式得到：

| 来源 | 含义 |
|------|------|
| **P3P 的结果** | 用 3 对 2D–3D 点，通过 P3P 算法直接算出的 $(R, \mathbf{t})$。这是解析解，通常较粗糙，但可作为非线性优化的初值。第 6 讲会讲 P3P。 |
| **RANSAC** | 用内点子集（如 4 个点）求得的位姿。RANSAC 剔除外点后，内点求得的解更可靠。 |
| **上一帧位姿** | 在视频或连续帧中，前一帧已估计好的 $(R, \mathbf{t})$。相机运动通常平滑，上一帧位姿可作为当前帧初值。 |

所以：**Bad initialization prevents convergence** → 常与"最小解 + RANSAC"配合使用。

---

## 9. 算法小结（仅非线性优化部分）

1. **参数化**：$(R, \mathbf{t})$ 用旋转向量 + $\mathbf{t}$（或四元数 + $\mathbf{t}$）作为优化变量 $\mathbf{a}$。
2. **残差**：对每个点 $i$，$\mathbf{r}_i = \mathbf{x}_i - \pi(K(R\mathbf{X}_i + \mathbf{t}))$，拼成 $\mathbf{r}$。
3. **雅可比**：$\frac{\partial \mathbf{r}}{\partial \mathbf{a}}$（对旋转向量/四元数及 $\mathbf{t}$），解析或数值。
4. **迭代**：用 GN 或 LM 解 $(J^\top J + \lambda D)\Delta\mathbf{a} = -J^\top \mathbf{r}$，更新 $\mathbf{a}$，转回 $R, \mathbf{t}$，直到收敛。
5. **输出**：$(R, \mathbf{t})$。

---

## 10. 本讲小结与"学完你能做什么"

| 内容 | 要点 |
|------|------|
| **目标** | 最小化重投影误差 $\sum_i \lVert \mathbf{x}_i - \pi(K(R\mathbf{X}_i+\mathbf{t}))\rVert ^2$。 |
| **非线性** | 无一步解析解；需线性化 + 迭代。 |
| **线性化** | 在当前参数处泰勒展开，$\mathbf{r} + J\Delta\mathbf{a}$，得到局部线性最小二乘。 |
| **Gauss-Newton** | 每步解 $J^\top J \Delta\mathbf{a} = -J^\top \mathbf{r}$，更新参数。 |
| **LM** | 阻尼项 $(J^\top J + \lambda I)$ 控制步长，更稳。 |
| **雅可比** | 解析雅可比收敛更快；参数化用旋转向量或四元数保证 $R$ 合法。 |
| **难点** | 外点破坏结果；初值差导致不收敛 → 引出 P3P + RANSAC。 |

**学完本讲你应该能**：

1. 写出 PnP 的**重投影残差**和**目标函数**。  
2. 说明 Gauss-Newton 和 LM 在做什么（线性化 → 解局部 LS → 更新；LM 加阻尼）。  
3. 解释为何需要**合法旋转参数化**、为何**外点和初值**是关键难点，以及为何常与下一讲的 P3P + RANSAC 配合使用。

---

## 11. 自测与练习建议

1. **手写**：对单点 PnP 残差 $\mathbf{r} = \mathbf{x} - \pi(K(R\mathbf{X}+\mathbf{t}))$，写出对 $\mathbf{t}$ 的偏导（假设 $R$ 固定）。  
2. **查阅**：在 Ceres 或 scipy 文档中看如何传入残差和（可选）雅可比，解非线性最小二乘。  
3. **思考**：若不加 RANSAC，直接对含 30% 外点的数据做非线性最小二乘，估计会怎样？
4. **口述练习**：不看笔记，用自己的话向同学解释："为什么 PnP 用非线性最小二乘？Gauss-Newton 和 LM 的核心区别是什么？为什么还需要 RANSAC？"——如果能在 2 分钟内讲清楚，说明你真正理解了。

完成以上后，可以进入 **第 6 讲：PnP 的解法二 —— P3P 最小解与 RANSAC**。
