# 补充 A-1：P3P 问题定义与 Grunert 解法

**对应文献**：Haralick et al., “Review and Analysis of Solutions of the Three Point Perspective Pose Estimation Problem”, IJCV 1994 — Section 2（Problem Definition）, Section 3 中 Grunert’s Solution（p3p_review.pdf）  
**本讲目标**：与主线第 6 讲衔接，用文献记号严格写出 P3P 问题并完整推一遍 Grunert 的四次多项式。

---

## 1. 问题设定（与主线第 6 讲的对应）

### 1.1 文献中的记号

- **相机坐标系**：原点在**透视中心**（光心），图像平面在光心前距离 $f$ 处。
- **三点** $\mathbf{p}_1, \mathbf{p}_2, \mathbf{p}_3$：三角形三个顶点在**相机系**下的位置（未知）。
- **已知边长**（三角形边长在刚体下与坐标系无关）：
  $$
  a = \|\mathbf{p}_2 - \mathbf{p}_3\|, \quad b = \|\mathbf{p}_1 - \mathbf{p}_3\|, \quad c = \|\mathbf{p}_1 - \mathbf{p}_2\|
  $$
- **观测**：三点在图像上的投影 $\mathbf{q}_i = (u_i, v_i)^\top$（或齐次），由针孔模型
  $$
  u_i = f \frac{x_i}{z_i}, \quad v_i = f \frac{y_i}{z_i}
  $$
  可知**从光心指向 $\mathbf{p}_i$ 的单位方向**为
  $$
  \mathbf{j}_i = \frac{(u_i, v_i, f)^\top}{\sqrt{u_i^2 + v_i^2 + f^2}}, \qquad i = 1,2,3
  $$
  $\mathbf{j}_i$ 由图像观测和内参 $f$（或 $K$）得到，故**已知**。
- **未知距离**：设光心到三点的距离为 $s_1, s_2, s_3$（$s_i = \lVert \mathbf{p}_i\rVert $），则
  $$
  \mathbf{p}_i = s_i \mathbf{j}_i, \qquad i = 1,2,3
  $$
  因此**只要求出 $s_1, s_2, s_3$，三点在相机系下的位置就确定**，再结合世界系下的三点即可做 3D–3D 配准得到 $(R, \mathbf{t})$。

### 1.2 与主线第 6 讲的对应

- 主线中的“单位射线 $\mathbf{j}_i$”= 文献中的 $\mathbf{j}_i$（由像素 + $K$ 归一化）。
- 主线中的“相机到三点的距离 $d_1, d_2, d_3$”= 文献中的 $s_1, s_2, s_3$。
- 边长 $a, b, c$ 由世界系下已知三点算出，与坐标系无关，故在相机系下仍成立。

---

## 2. 四面体与余弦定理三式

光心 $O$ 与三点 $\mathbf{p}_1, \mathbf{p}_2, \mathbf{p}_3$ 构成**四面体**。在光心处的三个角（与边 $a, b, c$ 相对）记为 $\alpha, \beta, \gamma$：

- $\alpha$ 对应边 $a$（$\mathbf{p}_2$–$\mathbf{p}_3$）：$\cos\alpha = \mathbf{j}_2^\top \mathbf{j}_3$
- $\beta$ 对应边 $b$（$\mathbf{p}_1$–$\mathbf{p}_3$）：$\cos\beta = \mathbf{j}_1^\top \mathbf{j}_3$
- $\gamma$ 对应边 $c$（$\mathbf{p}_1$–$\mathbf{p}_2$）：$\cos\gamma = \mathbf{j}_1^\top \mathbf{j}_2$

由**余弦定理**（三角形三边与一角）得三式：

$$
\begin{aligned}
s_2^2 + s_3^2 - 2 s_2 s_3 \cos\alpha &= a^2 \qquad \text{(1)} \\
s_1^2 + s_3^2 - 2 s_1 s_3 \cos\beta  &= b^2 \qquad \text{(2)} \\
s_1^2 + s_2^2 - 2 s_1 s_2 \cos\gamma  &= c^2 \qquad \text{(3)}
\end{aligned}
$$

三个未知数 $s_1, s_2, s_3$，三个二次方程；已知量为 $a, b, c$ 和 $\cos\alpha, \cos\beta, \cos\gamma$。

---

## 3. 变量替换：$s_2 = u s_1$，$s_3 = v s_1$

引入比值 $u, v$：

$$
s_2 = u s_1, \qquad s_3 = v s_1 \qquad \text{(4)}
$$

代入 (1)(2)(3)，并约去 $s_1^2$（假设 $s_1 \neq 0$）：

- (1) → $s_1^2(u^2 + v^2 - 2uv\cos\alpha) = a^2$
- (2) → $s_1^2(1 + v^2 - 2v\cos\beta) = b^2$
- (3) → $s_1^2(1 + u^2 - 2u\cos\gamma) = c^2$

由此可把 $s_1^2$ 用两种方式表示，例如由 (2)(3) 得

$$
s_1^2 = \frac{b^2}{1 + v^2 - 2v\cos\beta} = \frac{c^2}{1 + u^2 - 2u\cos\gamma}
$$

以及由 (1)(2) 等可得 $s_1^2 = a^2/(u^2 + v^2 - 2uv\cos\alpha)$。**让这些 $s_1^2$ 的表达式两两相等**，就得到只含 $u, v$ 的方程。

---

## 4. 得到关于 $u, v$ 的两个方程 (6)(7)

文献中通过 (1)(2)(3) 与 (4) 的代数变形，得到形如：

- **(6)**：关于 $u^2, v^2, uv, v, u$ 的二次方程（系数含 $a, b, c, \cos\alpha, \cos\beta, \cos\gamma$）。
- **(7)**：另一个关于 $u, v$ 的二次方程。

具体地，将 $s_1^2$ 从 (2)(3) 用 $u,v$ 表示后代入 (1) 的变形，或直接由“两式 $s_1^2$ 相等”得到 (6)(7)。例如由

$$
\frac{b^2}{1 + v^2 - 2v\cos\beta} = \frac{c^2}{1 + u^2 - 2u\cos\gamma}
$$

整理可得 (7)；由 (1) 与 (2) 的 $s_1^2$ 相等可得 (6)。这里不写出 (6)(7) 的完整系数（见文献），只保留结构：**(6)(7) 均为 $u, v$ 的二次型 + 一次 + 常数 = 0**。

---

## 5. Grunert 消元：$u$ 用 $v$ 表示，再代入得四次多项式

**第一步**：从 (6) 解出 $u^2$（用 $v$ 与 $uv$ 表示），代入 (7)。(7) 中若出现 $u^2$ 就换成该表达式，整理后 (7) 变成**只含 $u$ 与 $v$ 的一次/二次项**的等式，可解出

$$
u = \frac{\text{关于 } v \text{ 的表达式}}{2(\cos\gamma - v\cos\beta)} \qquad \text{(8)}
$$

（分母来自 (7) 中 $u$ 的系数；具体见文献 (8)。）

**第二步**：把 (8) 的 $u$ 代入 (6)。(6) 中 $u^2$ 和 $u$ 都变成 $v$ 的有理函数，通分后分母为 $[2(\cos\gamma - v\cos\beta)]^2$。两边乘分母，得到**关于 $v$ 的多项式**，最高次为 **4**，即

$$
A_4 v^4 + A_3 v^3 + A_2 v^2 + A_1 v + A_0 = 0 \qquad \text{(9)}
$$

**系数 $A_4, A_3, A_2, A_1, A_0$** 完全由 $a, b, c$ 和 $\cos\alpha, \cos\beta, \cos\gamma$ 决定。文献中给出（符号与文献一致）：

$$
\begin{aligned}
A_4 &= \Big(1 - \frac{a^2 - c^2}{b^2}\Big)^2 - \frac{4c^2}{b^2}\cos^2\alpha \\
A_3 &= 4\Big(1 - \frac{a^2 - c^2}{b^2}\Big)\Big(1 + \frac{a^2 - c^2}{b^2}\Big)\cos\beta - \frac{4c^2}{b^2}\cos\alpha\cos\gamma + \frac{4c^2}{b^2}\cos^2\alpha\cos\beta \\
&\quad \ldots \text{（其余项见原文）}
\end{aligned}
$$

$A_2, A_1, A_0$ 同样为 $a,b,c$ 与 $\cos\alpha,\cos\beta,\cos\gamma$ 的多项式。**重要点**：一旦得到 (9)，就可对 $v$ 求根（最多 4 个实根）。

---

## 6. 从根 $v$ 回到 $u$，再回到 $s_1, s_2, s_3$

对 (9) 的**每个实根 $v$**：

1. **求 $u$**：由 (8) 得 $u = u(v)$（注意分母 $\cos\gamma - v\cos\beta$ 为零时的退化需单独处理）。
2. **求 $s_1^2$**：例如用 (5)（文献中 $s_1^2 = c^2/(1+u^2-2u\cos\gamma)$）或 (2) 的 $s_1^2 = b^2/(1+v^2-2v\cos\beta)$。取 $s_1 > 0$（点在相机前方）。
3. **求 $s_2, s_3$**：$s_2 = u s_1$，$s_3 = v s_1$。

若 $s_1, s_2, s_3$ 中有非正或导致点在相机后方，则舍去该组。**最多保留 4 组** $(s_1, s_2, s_3)$（对应“点在相机前方”的 P3P 解）；文献也提到实际中常出现 2 组解（Wolfe et al. 1991）。

---

## 7. 从 $(s_1, s_2, s_3)$ 到位姿（与主线第 6 讲一致）

对每组 $(s_1, s_2, s_3)$：

- 相机系下三点：$\mathbf{p}_i = s_i \mathbf{j}_i$，$i=1,2,3$。
- 世界系下三点：$\mathbf{X}_1, \mathbf{X}_2, \mathbf{X}_3$（已知）。
- 求刚体变换 $(R, \mathbf{t})$ 使 $\mathbf{p}_i = R\mathbf{X}_i + \mathbf{t}$（或最小二乘意义下的最佳拟合）→ **3D–3D 配准（Absolute Orientation / Procrustes）**，见文献 Appendix I 或主线第 6 讲。

---

## 8. 本讲小结与“学完你能做什么”

| 内容 | 要点 |
|------|------|
| **问题** | 已知边长 $a,b,c$、单位射线 $\mathbf{j}_1,\mathbf{j}_2,\mathbf{j}_3$；未知 $s_1,s_2,s_3$；$\mathbf{p}_i = s_i\mathbf{j}_i$。 |
| **余弦定理** | (1)(2)(3)：$s_2^2+s_3^2-2s_2 s_3\cos\alpha=a^2$ 等；$\cos\alpha=\mathbf{j}_2^\top\mathbf{j}_3$ 等。 |
| **变量替换** | $s_2=u s_1$, $s_3=v s_1$ → 三式变关于 $u,v$ 的方程 (6)(7)。 |
| **Grunert 消元** | (6) 解出 $u^2$ 代入 (7) → $u=u(v)$ 的 (8)；再代入 (6) → 四次多项式 (9) $A_4 v^4+\cdots+A_0=0$。 |
| **从根到距离** | 对每个实根 $v$ 用 (8) 得 $u$，用 (5) 或 (2) 得 $s_1>0$，再 $s_2=us_1$, $s_3=vs_1$；最多 4 组有效解。 |

**学完本讲你应该能**：

1. 用文献记号写出 P3P 的已知/未知量，并说明 (1)(2)(3) 的来源（余弦定理）。  
2. 写出变量替换 (4) 并说明如何得到 (6)(7)。  
3. 口述 Grunert 的两步：先 (6)(7) → (8) 得到 $u(v)$，再代入 (6) 得 (9)；知道 (9) 是 $v$ 的四次多项式，系数由 $a,b,c,\cos\alpha,\cos\beta,\cos\gamma$ 确定。  
4. 与主线第 6 讲对应：文献的 $s_i, \mathbf{j}_i, a,b,c$ = 主线的 $d_i$、射线、边长；从 (9) 求根到 $s_1,s_2,s_3$ = 主线 “Step 4” 求距离。

---

## 9. 自测与练习建议

1. **手推**：在 (4) 下，从 (2)(3) 写出 $s_1^2 = b^2/(1+v^2-2v\cos\beta)$ 和 $s_1^2 = c^2/(1+u^2-2u\cos\gamma)$，令两者相等，整理成关于 $u, v$ 的二次方程（即 (7) 的一种形式）。  
2. **编程**：给定 $a,b,c$ 和 $\cos\alpha,\cos\beta,\cos\gamma$，按文献或本文写出 (9) 的系数 $A_4,\ldots,A_0$（可查原文公式），用 `np.roots([A4,A3,A2,A1,A0])` 求 $v$，再算 $u$ 和 $s_1,s_2,s_3$，验证 (1)(2)(3)。  
3. **思考**：若三角形退化（三点共线），(9) 的系数会怎样？此时 P3P 是否仍有唯一性？

完成以上后，可以进入 **补充 A-2：六种 P3P 解法的代数比较**。
