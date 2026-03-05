# 补充 A-3：数值稳定性与 Absolute Orientation

**对应文献**：Haralick et al., IJCV 1994 — Introduction 后段（数值稳定性动机）、Section 6 Conclusions、**Appendix I**（Absolute Orientation）、**Appendix II**（数值精度与分析）（p3p_review.pdf）  
**本讲目标**：理解 P3P 实现时为何要关心数值稳定性，以及从 $s_1,s_2,s_3$ 到位姿 $(R,\mathbf{t})$ 的线性解法（与 Procrustes/主线第 6 讲衔接）。

---

## 1. 数值稳定性动机（为何 P3P 实现要小心）

### 1.1 舍入误差会累积

P3P 的求解链很长：**输入**（边长 $a,b,c$、$\cos\alpha,\cos\beta,\cos\gamma$）→ **中间**（(6)(7)、(8)、四次多项式系数 $A_4,\ldots,A_0$、求根）→ **回代**（$u,v \to s_1,s_2,s_3$）→ **Absolute Orientation**（求 $R,\mathbf{t}$）。每一步都在**有限精度**下计算，舍入误差会**传播并放大**，尤其在做除法、相近数相减、或系数对根非常敏感时。

### 1.2 方程使用顺序会极大影响误差

文献强调：**“The order of using these equations to derive the final solution affects the accuracy of numerical results.”** 例如：

- 变量替换若改为 $s_1 = u s_2,\ s_3 = v s_2$（而不是 $s_2 = u s_1,\ s_3 = v s_1$），得到的多项式系数会不同（相当于 $a\leftrightarrow b,\ \alpha\leftrightarrow\beta$），**数值行为**可能差很多。
- 对**同一解法**，三个点对的**输入顺序**（6 种排列）也会导致 6 种不同的计算路径，精度可相差**约 10³ 倍**（文献：随机排列约 $0.19\times 10^{-8}$，用 Appendix II 的分析选顺序可达约 $0.9\times 10^{-12}$，接近“每次选最优排列”的 $0.41\times 10^{-12}$）。

因此：**“Depending on the order of the substitutions utilized, the relative error can change over a thousand to one.”** 这种差异**主要来自计算顺序**，而不是几何退化（几何不稳定时会更糟）。

### 1.3 高分辨率与后续阶段

- 传感器与图像分辨率提高后，输入更精细，**数值稳定性**在总误差中的占比会更大。
- P3P 求得的 $s_1,s_2,s_3$ 若有误差，会直接带入 **Absolute Orientation**；舍入误差在 P3P 阶段就被放大，到 3D–3D 配准时会**进一步放大**，导致最终位姿误差很大。所以文献说：**“The accumulation of rounding errors will be propagated into the calculation of the absolute orientation problem. As a result the error would be very serious at the final stage.”**

---

## 2. Appendix I：Absolute Orientation（从三点到位姿的线性解法）

### 2.1 问题重述

已知：

- **相机系**下三点 $\mathbf{p}_i = (x_i, y_i, z_i)^\top$（由 P3P 得到 $\mathbf{p}_i = s_i \mathbf{j}_i$）；
- **世界系**下对应三点 $\mathbf{p}_i' = (x_i', y_i', z_i')^\top$。

求**旋转矩阵 $R$** 与**平移向量 $\mathbf{T}$**，使

$$
\mathbf{p}_i = R \mathbf{p}_i' + \mathbf{T}, \qquad i = 1,2,3 \qquad \text{(a.1)}
$$

即 **world-to-camera**：把世界坐标变到相机坐标。在摄影测量中称为 **Absolute Orientation**。

### 2.2 文献中的线性做法（利用共面约束）

$R$ 有 9 个元素但满足 $R^\top R = I$（6 个约束），所以只有 3 个自由度；直接线性化会得到**欠定**系统。文献的做法是：

- 利用**三点共面**（三角形）：在世界系中可设平面为 $z'=0$，即 $z_i'=0$。这样 (a.1) 的第三行 $z_i = r_{31}x_i' + r_{32}y_i' + t_z$ 中不出现 $r_{33}$，未知数减少。
- 把 $R$ 的 9 个元素用 6 个独立量加约束 (a.2) 表示（例如 $r_{13}, r_{23}, r_{33}$ 由前两行通过正交性推出），再代入 (a.1)，得到**关于 $r_{11}, r_{12}, r_{21}, r_{22}, r_{31}, r_{32}, t_x, t_y, t_z$ 的 9 个线性方程**，写成 $AX = B$。只要三点不共线，$A$ 可逆，有**唯一解**。解出 $X$ 后再用 (a.2) 求出 $r_{13}, r_{23}, r_{33}$。

这样就从“相机系三点 + 世界系三点”**线性**得到 $(R, \mathbf{T})$，与迭代无关，实现简单；缺点是对共面假设和数值误差较敏感。

### 2.3 与 Kabsch / Procrustes 的对应（主线第 6 讲、04_ransac_procrustes）

更常用、数值上往往更稳的是 **3D–3D 配准**的闭式解（Kabsch / Procrustes）：

1. **去中心**：$\bar{\mathbf{p}} = \frac{1}{3}\sum \mathbf{p}_i$，$\bar{\mathbf{p}}' = \frac{1}{3}\sum \mathbf{p}_i'$；令 $\tilde{\mathbf{p}}_i = \mathbf{p}_i - \bar{\mathbf{p}}$，$\tilde{\mathbf{p}}_i' = \mathbf{p}_i' - \bar{\mathbf{p}}'$。
2. **平移**：$\mathbf{T} = \bar{\mathbf{p}} - R \bar{\mathbf{p}}'$（求出 $R$ 后代入）。
3. **旋转**：最小化 $\sum_i \lVert \tilde{\mathbf{p}}_i - R \tilde{\mathbf{p}}_i'\rVert ^2$。令 $H = \sum_i \tilde{\mathbf{p}}_i \tilde{\mathbf{p}}_i'^\top$，对 $H$ 做 SVD：$H = U \Sigma V^\top$，则
   $$
   R = U V^\top
   $$
   （若 $\det(R)=-1$，取 $R = U \operatorname{diag}(1,1,-1) V^\top$ 保证为纯旋转。）

这样得到的 $R$ 是**正交的**，不依赖共面假设；三点时与 Appendix I 在无噪声下等价，有噪声时 Procrustes 更稳。**从 P3P 得到 $\mathbf{p}_i = s_i \mathbf{j}_i$ 后，用上述 Procrustes 步骤即可得到 $(R, \mathbf{T})$**，与主线第 6 讲、04_ransac_procrustes 一致。

---

## 3. Appendix II：数值精度与分析技巧

### 3.1 方程顺序与输入排列

- **变量替换顺序**：例如用 $s_2=u s_1,\, s_3=v s_1$ 与用 $s_1=u s_2,\, s_3=v s_2$ 会得到**不同系数的多项式**，数值行为不同。
- **三点对的排列**：把 3 个 2D–3D 对应按 6 种顺序输入同一 P3P 流程，会得到 6 种**计算路径**（中间方程组合、消元顺序不同），精度可差约 **10³ 倍**。  
Appendix II 的目标之一就是：**用分析手段选一个“好”的排列与方程顺序**，使大多数情况下误差较小，而不必每次随机试。

### 3.2 “先算大根还是小根”（Numerically stable way）

文献在 Finsterwalder、Grunert 等处多次提到：**“The numerically stable way of doing this computation is to determine the small root in terms of the larger root.”** 原因简述：

- 二次方程 $ax^2+bx+c=0$ 的根若一大一小，用求根公式直接算**小根**时会出现“大数减大数”，有效数字损失；改为先用公式算**大根** $x_{\text{large}}$，再用**韦达定理** $x_{\text{small}} = c/(a \cdot x_{\text{large}})$ 得到小根，可减少舍入误差。  
类似地，在 (8) 中算 $u$、在 (11) 中算 $v$ 时，选“大根”“小根”的先后顺序也会影响稳定性。Appendix II 的分析会指导在**每一步**选哪种顺序。

### 3.3 多项式零点的敏感性（Sensitivity of polynomial zeros）

四次多项式 $P(v) = A_4 v^4 + \cdots + A_0$ 的根对**系数** $A_i$ 的扰动很敏感时，系数的小误差（来自前面步骤的舍入）会导致根的大偏差。Appendix II 用**零点对系数的灵敏度**（偏导 $\partial x/\partial a_i$）和**最坏相对/绝对误差**来估计：

- 哪些系数或哪一步最容易放大误差；
- 哪种**方程顺序/排列**会使多项式条件较好、零点漂移较小。

这样就能**预测**哪种顺序更稳，而不必只靠大量蒙特卡洛试。

### 3.4 舍入误差传播（Worst-case rounding error）

加减乘除在有限精度下每步都会引入相对/绝对误差；这些误差在后续运算中会**传播**。Appendix II 对每类运算建模（如 $fl(x_1 \pm x_2)$ 的误差与 $x_1,x_2$ 的误差关系），并考虑**最坏情况**（例如相对误差取 $0.5\times 10^{1-d}$，$d$ 为有效位数），得到系数或根的**最坏相对/绝对误差**。结合零点敏感性，可估计**最终距离/位姿误差**的量级，并据此选择计算顺序。

---

## 4. 实验结论（Section 6 Conclusions）

文献做了**数十万次**实验，比较六种解法、不同方程顺序与排列、以及 Appendix II 的分析方法。要点概括：

- **Finsterwalder** 在多数情况下**精度最好**（双精度约 $10^{-11}$ 量级，单精度约 $10^{-2}$ 量级；具体数值见原文）。
- **不同方程对、不同变量替换**会带来**不同的数值行为**；没有一种在所有几何下都最优，但可通过**排列与顺序**改善。
- **用 Appendix II 的分析**为 Grunert 解法选顺序，可将精度提升到约 **$0.9\times 10^{-12}$**，与“每次试遍选最优排列”的 $0.41\times 10^{-12}$ 很接近，且比**随机排列**的 $0.19\times 10^{-8}$ 好约 **千倍**。  
因此：**“The analysis techniques in Appendix II are effective in determining equation order manipulation.”**

---

## 5. 实现 P3P 时的实用建议

1. **输入顺序**：若可能，用 Appendix II 的灵敏度/误差分析选一个固定“好”排列；或对 3 个点对做少量排列（如 2–3 种），取重投影误差最小或最稳的那组。
2. **求根与回代**：二次根用“大根 + 韦达定理得小根”；四次根可用稳定求根顺序（如先提取绝对值较小的根再做 deflation）；回代时避免“小数除小数”等易放大误差的写法。
3. **从 $s_1,s_2,s_3$ 到位姿**：优先用 **Procrustes（Kabsch）**：去中心 → SVD($H$) → $R = UV^\top$ → $\mathbf{t} = \bar{\mathbf{p}} - R\bar{\mathbf{p}}'$；与 Appendix I 的线性法相比更稳，且不依赖共面假设。
4. **精度与退化**：需要高精度时用双精度；接近退化（三点共线、视角很扁）时结果不可靠，可结合重投影或第 4 点做筛选。

---

## 6. 本讲小结与“学完你能做什么”

| 内容 | 要点 |
|------|------|
| **数值稳定性动机** | 舍入误差累积；**方程/输入顺序**可导致相对误差相差约 **10³**；误差会传到 Absolute Orientation。 |
| **Appendix I** | 已知 $\mathbf{p}_i$（相机）与 $\mathbf{p}_i'$（世界），求 $R,\mathbf{T}$ 使 $\mathbf{p}_i = R\mathbf{p}_i' + \mathbf{T}$；文献用共面假设化为 9×9 线性方程组。 |
| **与 Procrustes** | 去中心 → $H = \sum \tilde{\mathbf{p}}_i \tilde{\mathbf{p}}_i'^\top$，SVD 得 $R=UV^\top$，再 $\mathbf{t}=\bar{\mathbf{p}}-R\bar{\mathbf{p}}'$；与主线第 6 讲、04 讲义一致。 |
| **Appendix II** | 方程顺序与输入排列；**“小根用大根表示”**；多项式零点敏感性；最坏舍入误差传播。 |
| **实验结论** | Finsterwalder 常最准；用 Appendix II 选顺序可使 Grunert 提升约千倍，接近“最优排列”。 |

**学完本讲你应该能**：

1. 说明为何 P3P 实现要关心**方程顺序**和**输入排列**，以及误差如何传到 Absolute Orientation。  
2. 写出 **Absolute Orientation** 的问题（$\mathbf{p}_i = R\mathbf{p}_i' + \mathbf{T}$），并实现或调用 **Procrustes（Kabsch）** 从 $\mathbf{p}_i,\mathbf{p}_i'$ 得到 $(R,\mathbf{t})$。  
3. 理解 Appendix II 的要点：选顺序、小根用大根表示、零点敏感性、舍入传播；并在写 P3P 时避免明显不稳定的写法。

---

## 7. 自测与练习建议

1. **编程**：用主线第 6 讲或 04_ransac_procrustes 的步骤，从三组 $\mathbf{p}_i = s_i\mathbf{j}_i$ 与 $\mathbf{p}_i' = \mathbf{X}_i$ 实现 Procrustes，得到 $R,\mathbf{t}$，并验证 $\mathbf{p}_i \approx R\mathbf{p}_i' + \mathbf{t}$。  
2. **思考**：若 P3P 给出的 $s_1,s_2,s_3$ 有 1% 的误差，对 Procrustes 求得的 $R,\mathbf{t}$ 会有什么量级的影响？为何文献说 P3P 阶段的误差会在最后“very serious”？  
3. **查阅**：文献中“determine the small root in terms of the larger root”在你使用的求根或二次方程代码里是否已有类似处理？

---

**Part A（P3P 深度）** 三讲至此全部结束。可回顾主线第 6 讲与 04_ransac_procrustes，把 P3P → 距离 → Procrustes → $(R,\mathbf{t})$ 的完整管线在代码里跑通，并注意数值顺序与稳定性。
