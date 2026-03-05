# 第 1 讲：Procrustes 问题与 Kabsch 算法

**对应讲义**：04_ransac_procrustes.pdf 第 7–14 页  
**本讲目标**：第一次学就能理解“两堆 3D 点已知谁对谁，怎么求一个旋转+平移把它们对齐”，并会用手算或代码完成 Kabsch 的每一步。

---

## 先别记公式：问题到底是什么？

想象你有**两堆 3D 点**，而且你知道**谁和谁是一对**：  
比如第一堆里第 1 个点对应第二堆里第 1 个点，第 2 个对第 2 个……  
但这两堆点是在**两个不同的坐标系**里量的（例如一个是相机坐标系，一个是世界坐标系）。

**我们想做的事**：找一个**旋转 $R$** 和一个**平移 $\mathbf{t}$**，把“第二堆点”用 $R$ 旋转、再整体平移 $\mathbf{t}$ 之后，能和“第一堆点”尽量重合。  
换句话说：**用刚体变换（只旋转+平移，不缩放、不拉伸）把两团点对齐**。  
这个问题就叫 **Procrustes 问题**（刚体情形），求出来的 $R$ 和 $\mathbf{t}$ 就是“从第二个坐标系到第一个坐标系”的变换。

**重要前提**：我们**已经知道对应关系**（没有匹配错误），而且**没有缩放**——两堆点之间的相对形状是一样的，只是摆的方向和位置不同。满足这两点，下面讲的方法才适用。

---

## 和前面课的关系：Homography 为什么要 4 个点，P3P 只要 3 个？

（若你还没学单应和 PnP，可先跳过这小节，不影响本讲主体。）

- **单应（Homography）**：描述的是“一个平面上的点”到“另一个平面上的点”的映射，**不只有位姿**，还包含平面的透视变形，所以**自由度是 8**。要确定 8 个未知数，至少需要 4 对点（每对 2 个方程）。
- **P3P**：假设相机已经标定，我们只求**相机的位姿**（旋转 3 + 平移 3 = **6 个自由度**）。几何上 3 个 3D 点就能提供足够约束（会得到最多 4 组解，再用第 4 个点选一组），所以**最少 3 个点**。

本讲的 Procrustes 也是在求**刚体变换**（6 个自由度），所以从“最少点数”的角度，**不共线的 3 个点**就可以唯一确定 $R$ 和 $\mathbf{t}$；多于 3 个点时，就用“最小二乘”找最贴合所有点对的 $R$ 和 $\mathbf{t}$。

---

## 1. 把问题写成数学

设**第一堆点**（例如相机系）为 $\mathbf{p}_1, \ldots, \mathbf{p}_n$，**第二堆点**（例如世界系）为 $\mathbf{q}_1, \ldots, \mathbf{q}_n$，且**第 $i$ 个和第 $i$ 个是一对**。  
我们要找**旋转矩阵 $R$**（3×3，满足 $R^\top R = I$、$\det R = 1$）和**平移向量 $\mathbf{t}$**（3×1），使得

$$
\mathbf{p}_i \approx R \mathbf{q}_i + \mathbf{t}, \qquad i = 1,\ldots,n
$$

“尽量重合”用**最小二乘**表示：让**所有点对的误差平方和**最小，即

$$
\min_{R,\,\mathbf{t}} \quad \sum_{i=1}^{n} \big\| \mathbf{p}_i - (R\mathbf{q}_i + \mathbf{t}) \big\|^2
$$

这里 $R$ 必须是**合法旋转**（正交、行列式为 1）。下面会看到：**可以先求出最优的 $\mathbf{t}$（用 $R$ 表示），再专心求 $R$，而且 $R$ 有一个闭式解（Kabsch），不用迭代。**

---

## 2. 第一步：平移应该取成什么样？（去中心）

**直觉**：如果只允许平移、不许旋转，那“最对齐”的平移就是：**把两堆点的“中心（重心）”移到一起**。  
数学上，两堆点的中心分别是

$$
\bar{\mathbf{p}} = \frac{1}{n}\sum_{i=1}^{n} \mathbf{p}_i, \qquad \bar{\mathbf{q}} = \frac{1}{n}\sum_{i=1}^{n} \mathbf{q}_i
$$

可以证明：**对任意给定的 $R$，使误差平方和最小的平移就是**

$$
\mathbf{t} = \bar{\mathbf{p}} - R \bar{\mathbf{q}}
$$

也就是说：先把第二堆点用 $R$ 旋转，再整体平移，让**旋转后的第二堆的中心**恰好落在**第一堆的中心**上。  
所以：**我们只要先专心把 $R$ 求出来，平移再用上面这个式子算就行。** 这就是讲义里的 “Step 1: centering”——后面我们会把两堆点都**减去各自的中心**，变成“中心在原点”的两团点，这样平移就暂时不用管了。

---

## 3. 第二步：只看“去中心”后的点，问题变成只求 $R$

记**去中心**后的点为

$$
\tilde{\mathbf{p}}_i = \mathbf{p}_i - \bar{\mathbf{p}}, \qquad \tilde{\mathbf{q}}_i = \mathbf{q}_i - \bar{\mathbf{q}}
$$

把 $\mathbf{t} = \bar{\mathbf{p}} - R\bar{\mathbf{q}}$ 代入目标函数，可以化简（展开后交叉项会消掉）得到：

$$
\sum_{i} \big\| \mathbf{p}_i - (R\mathbf{q}_i + \mathbf{t}) \big\|^2
= \sum_{i} \big\| \tilde{\mathbf{p}}_i - R\tilde{\mathbf{q}}_i \big\|^2
$$

也就是说：**在最优平移 $\mathbf{t} = \bar{\mathbf{p}} - R\bar{\mathbf{q}}$ 下，最小化“点对误差平方和”等价于最小化“去中心后的点对”的误差平方和**，而且**式子里面已经没有 $\mathbf{t}$ 了**。  
所以接下来我们**只需求**：

$$
\min_{R \in SO(3)} \quad \sum_{i=1}^{n} \big\| \tilde{\mathbf{p}}_i - R\tilde{\mathbf{q}}_i \big\|^2
$$

这里 $SO(3)$ 表示“所有 3×3 旋转矩阵”的集合。下面会把它变成**一个只和 $R$ 有关的、容易求最大值的式子**。

---

## 4. 第三步：从“最小化距离和”到“最大化 trace”

把平方展开：

$$
\big\| \tilde{\mathbf{p}}_i - R\tilde{\mathbf{q}}_i \big\|^2
= \|\tilde{\mathbf{p}}_i\|^2 + \|R\tilde{\mathbf{q}}_i\|^2 - 2 \tilde{\mathbf{p}}_i^\top R\tilde{\mathbf{q}}_i
$$

因为 $R$ 是旋转，**不改变向量长度**，所以 $\lVert R\tilde{\mathbf{q}}_i\rVert ^2 = \lVert \tilde{\mathbf{q}}_i\rVert ^2$。因此

$$
\sum_i \big\| \tilde{\mathbf{p}}_i - R\tilde{\mathbf{q}}_i \big\|^2
= \sum_i \|\tilde{\mathbf{p}}_i\|^2 + \sum_i \|\tilde{\mathbf{q}}_i\|^2 - 2 \sum_i \tilde{\mathbf{p}}_i^\top R\tilde{\mathbf{q}}_i
$$

前两项**和 $R$ 无关**，所以**要让上面这个和最小，等价于让最后一项最大**，即

$$
\max_{R \in SO(3)} \quad \sum_i \tilde{\mathbf{p}}_i^\top R\tilde{\mathbf{q}}_i
$$

记矩阵

$$
H = \sum_{i=1}^{n} \tilde{\mathbf{p}}_i \tilde{\mathbf{q}}_i^\top
$$

（每一对点贡献一个 3×3 矩阵 $\tilde{\mathbf{p}}_i \tilde{\mathbf{q}}_i^\top$，全部加起来就是 $H$。）可以证明：

$$
\sum_i \tilde{\mathbf{p}}_i^\top R\tilde{\mathbf{q}}_i = \operatorname{tr}(R^\top H)
$$

这里 $\operatorname{tr}$ 表示矩阵的**迹**（对角线元素之和）。所以问题变成：

$$
\boxed{\; \max_{R \in SO(3)} \quad \operatorname{tr}(R^\top H) \;}
$$

这就是讲义里的 “Step 2: substituting t back → Maximizing the trace”。**下一步就是：在所有旋转矩阵里，谁能让 $\operatorname{tr}(R^\top H)$ 最大？答案由 $H$ 的 SVD 给出。**

---

## 5. Frobenius 范数的一句话（可选）

若你见过 **Frobenius 范数**：$\lVert A\rVert _F^2 = \operatorname{tr}(A^\top A)$。有一个性质：**正交矩阵左乘不改变 F-范数**，即 $\lVert U A\rVert _F = \lVert A\rVert _F$。  
在推导“最小化 $\lVert \tilde{\mathbf{p}}_i - R\tilde{\mathbf{q}}_i\rVert ^2$ 等价于最大化 $\operatorname{tr}(R^\top H)$”时，也可以用 F-范数把 $\sum_i \lVert \tilde{\mathbf{p}}_i - R\tilde{\mathbf{q}}_i\rVert ^2$ 写成矩阵形式再化简。第一次学只要知道：**最后我们就是在最大化 $\operatorname{tr}(R^\top H)$** 即可。

---

## 6. Kabsch 算法：最优 $R$ 的闭式解（SVD）

对矩阵 $H$ 做 **SVD**（奇异值分解）：

$$
H = U \Sigma V^\top
$$

其中 $U$、$V$ 是 3×3 正交矩阵，$\Sigma$ 是对角矩阵（非负奇异值）。则**使 $\operatorname{tr}(R^\top H)$ 最大的旋转矩阵**为

$$
R = U V^\top
$$

**注意**：若算出来 $\det(R) = -1$（表示是“旋转+反射”），则要把 $V$ 的最后一列取反，再算一次 $R = UV^\top$，保证 $\det(R) = 1$（纯旋转）。  
这个结论就叫 **Kabsch 算法**（或 Procrustes 的 SVD 形式）。  
**第一次学可以只记这一句**：两堆点去中心后，算 $H = \sum_i \tilde{\mathbf{p}}_i \tilde{\mathbf{q}}_i^\top$，对 $H$ 做 SVD，$R = UV^\top$（必要时把 $V$ 一列取反使 $\det R=1$），平移 $\mathbf{t} = \bar{\mathbf{p}} - R\bar{\mathbf{q}}$。

---

## 7. 完整步骤小结（按顺序做就行）

1. **算中心**：$\bar{\mathbf{p}} = \frac{1}{n}\sum \mathbf{p}_i$，$\bar{\mathbf{q}} = \frac{1}{n}\sum \mathbf{q}_i$。  
2. **去中心**：$\tilde{\mathbf{p}}_i = \mathbf{p}_i - \bar{\mathbf{p}}$，$\tilde{\mathbf{q}}_i = \mathbf{q}_i - \bar{\mathbf{q}}$。  
3. **算 $H$**：$H = \sum_{i=1}^{n} \tilde{\mathbf{p}}_i \tilde{\mathbf{q}}_i^\top$（3×3 矩阵）。  
4. **对 $H$ 做 SVD**：$H = U \Sigma V^\top$。  
5. **取旋转**：$R = U V^\top$；若 $\det(R) = -1$，把 $V$ 的第三列乘以 $-1$ 再算 $R$。  
6. **取平移**：$\mathbf{t} = \bar{\mathbf{p}} - R \bar{\mathbf{q}}$。  
7. **输出**：刚体变换 $(R, \mathbf{t})$；第二堆点变换到第一堆坐标系为 $\mathbf{p}_{\text{pred},i} = R\mathbf{q}_i + \mathbf{t}$。

---

## 8. 最少需要几个点？

- **3 个点**：若三点**不共线**，刚体变换由 3 对对应点**唯一确定**（忽略反射时）。此时 $H$ 一般是满秩的，Kabsch 给出唯一合理的 $R$。  
- **多于 3 个点**：用上面同样的公式，$H = \sum_i \tilde{\mathbf{p}}_i \tilde{\mathbf{q}}_i^\top$ 会把所有点对都算进去，得到的是**最小二乘意义下**最贴合所有点对的 $R$ 和 $\mathbf{t}$，更抗噪声。  
- **共线或近似共线**：几何上旋转会绕该直线有歧义，数值上 $H$ 会接近奇异，结果不稳定；实际中尽量用**不共线**的三点或更多点。

---

## 9. 和主线第 6 讲（P3P）的衔接

在 **P3P** 里，我们先用 3 个点从图像几何求出**相机到三点的距离** $d_1, d_2, d_3$，于是**相机系**下三点为 $\mathbf{p}_i = d_i \mathbf{j}_i$；**世界系**下三点为 $\mathbf{X}_i$。  
接下来要算的“从世界系到相机系的旋转和平移”，就是**两团 3D 点** $\mathbf{p}_i$ 与 $\mathbf{X}_i$ 的**刚体配准**——正是本讲的 Procrustes 问题。所以 **P3P 的最后一步就是用 Kabsch（本讲）求 $R$ 和 $\mathbf{t}$**。

---

## 10. 本讲小结（可当小抄）

| 问题 | 两堆 3D 点一一对应，求旋转 $R$ 和平移 $\mathbf{t}$ 使 $\mathbf{p}_i \approx R\mathbf{q}_i + \mathbf{t}$。 |
|------|----------------------------------------------------------------------------------------------------------|
| 平移最优解 | $\mathbf{t} = \bar{\mathbf{p}} - R\bar{\mathbf{q}}$；先求 $R$ 再算 $\mathbf{t}$。 |
| 去中心后 | 只需求 $\min_R \sum_i \lVert \tilde{\mathbf{p}}_i - R\tilde{\mathbf{q}}_i\rVert ^2$，等价于 $\max_R \operatorname{tr}(R^\top H)$。 |
| $H$ 是什么 | $H = \sum_i \tilde{\mathbf{p}}_i \tilde{\mathbf{q}}_i^\top$。 |
| Kabsch | $H = U\Sigma V^\top$ → $R = UV^\top$（必要时修正使 $\det R=1$）。 |
| 最少点数 | 不共线 3 点可唯一确定刚体；更多点用最小二乘。 |

**学完本讲你应该能**：  
- 用**自己的话**说清楚“Procrustes 在求什么”“为什么要去中心”“为什么最后是最大化 trace”；  
- **按上面 7 步**手算或编程实现 Kabsch；  
- 说出 P3P 里“求完距离之后”的那一步就是本讲的 Procrustes。

---

## 11. 自测与练习（第一次学建议都做）

1. **口述**：不写公式，用一两句话向别人解释“两堆 3D 点已知对应，怎么求旋转和平移？先干什么再干什么？”  
2. **手算**：设两对点 $\mathbf{p}_1=(1,0,0), \mathbf{p}_2=(0,1,0)$，$\mathbf{q}_1=(0,1,0), \mathbf{q}_2=(-1,0,0)$（相当于绕 $z$ 轴转 90°）。去中心后算 $H$，再对 $H$ 做 SVD，写出 $R$ 和 $\mathbf{t}$，并验证 $\mathbf{p}_i = R\mathbf{q}_i + \mathbf{t}$。  
3. **编程**：用 NumPy 的 `np.linalg.svd` 对任意 $n$ 对点实现上面 7 步，并检查 $\det(R)=1$、$\mathbf{t} = \bar{\mathbf{p}} - R\bar{\mathbf{q}}$。

完成以上后，可以继续 **第 2 讲：外点与 RANSAC 思想**。
