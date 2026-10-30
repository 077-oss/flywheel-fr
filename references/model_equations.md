# 飞轮储能调频仿真 —— 数学模型与方程推导

本文档严格推导 Skill 中 `fr_sim.m` 使用的全部数学模型，供论文写作与公式核对。所有变量采用 SI 单位制或标幺值（per-unit, pu），进入微分方程的状态量统一标幺。

## 0. 系统拓扑

8 区等值机互联系统（`nArea = 8`），每区由 1 台等值同步发电机等效（代表该区所有机组的聚合惯量与调差能力）。区与区之间由联络线连接，构成环形拓扑。飞轮储能系统（FES）**集中式单点接入第 `kb` 区母线**。

- 每区等值机：惯量常数 $H_i$、阻尼系数 $D_i$、调差系数 $R_i$、调速器时间常数 $T_{gi}$
- 联络线：同步功率系数 $T_{ij}$（区 $i$–$j$ 之间）
- 飞轮：详细部件模型（转子 + PMSM + 变流器），额定基准功率 $S_{fes}$

---

## 1. 同步发电机模型（每区）

### 1.1 摆动方程（转子动力学，二阶）

标幺形式的经典摆动方程：

$$
\frac{2H_i}{\omega_s} \frac{d\omega_i}{dt} = P_{mi} - P_{ei} - D_i (\omega_i - 1)
$$

其中：
- $H_i$：惯量常数（秒），表征该区等值机储存的旋转动能与额定容量之比
- $\omega_s = 2\pi f_0$：同步角速度（基准）
- $\omega_i$：转子角速度（标幺，$\omega_i = f_i/f_0$）
- $P_{mi}$：机械功率（标幺）
- $P_{ei}$：电磁功率（标幺）
- $D_i$：阻尼系数（标幺），反映负荷频率特性与发电机阻尼绕组效应

### 1.2 功角方程

$$
\frac{d\delta_i}{dt} = \omega_s (\omega_i - 1)
$$

### 1.3 调速器-原动机模型（一阶）

原动机与调速器的简化一阶传递函数：

$$
T_{gi} \frac{dP_{mi}}{dt} = P_{ref,i} - P_{mi} - \frac{1}{R_i} (\omega_i - 1)
$$

其中 $R_i$ 为调差系数（机组的一次调频能力），$\frac{1}{R_i}(\omega_i-1)$ 是调速器依据频率偏差调节汽门/导叶产生的功率增量。

---

## 2. 网络耦合（跨区功率流动）

区 $i$ 的电磁功率由本地负荷、联络线交换功率构成：

$$
P_{ei} = P_{Li} + \sum_{j \neq i} T_{ij}(\delta_i - \delta_j)
$$

联络线功率按同步功率系数 $T_{ij}$ 与两区功角差线性近似（经典电力系统小扰动分析假设）：

$$
P_{tie,ij} = T_{ij}(\delta_i - \delta_j)
$$

其中 $T_{ij} = \dfrac{|V_i||V_j|}{X_{ij}}\cos(\delta_{i0}-\delta_{j0})$，与联络线电抗 $X_{ij}$ 成反比。

---

## 3. 飞轮储能详细部件模型

### 3.1 转子动力学（储能本体）

飞轮储存的旋转动能与转速平方成正比：

$$
E_{fes} = \frac{1}{2} J_{fes} \omega_f^2, \qquad
J_{fes} = \frac{1}{2} m r^2 \;(\text{实心圆盘})
$$

转子运动方程（能量守恒：电功率输出 = 动能减少速率）：

$$
J_{fes} \omega_f \frac{d\omega_f}{dt} = -P_e = -T_e \omega_f
\quad\Longrightarrow\quad
\frac{d\omega_f}{dt} = -\frac{T_e}{J_{fes}}
$$

### 3.2 PMSM 电磁转矩

永磁同步电机（PMSM）电磁转矩：

$$
T_e = \frac{3}{2} p \left[ \psi_f i_q + (L_d - L_q) i_d i_q \right]
$$

采用 $i_d = 0$ 矢量控制时简化为：

$$
T_e = \frac{3}{2} p\, \psi_f i_q
$$

### 3.3 荷电状态 SOC

$$
\text{SOC} = \frac{E_{fes}}{E_{fes,\max}} = \left(\frac{\omega_f}{\omega_{f,\max}}\right)^2
$$

**关键结论**：飞轮 SOC 与转速平方成正比，转速下限受 SOC 约束（通常保留 20%~100% 可用区间）。

### 3.4 充放电功率关系（含效率）

放电（$T_e>0$）：$P_{fes} = T_e \omega_f \cdot \eta$（电功率 = 机械功率 × 效率）
充电（$T_e<0$）：$P_{fes} = T_e \omega_f / \eta$

### 3.5 功率标幺基准（关键）

飞轮功率标幺与电网标幺必须统一到同一基准：

$$
P_{fes,pu} = \frac{P_{fes}}{S_{base}}, \qquad
S_{fw} = \frac{P_{fes,rate}}{S_{base}}
$$

- $S_{base}$：系统基准功率（默认 100 MW）
- $P_{fes,rate}$：飞轮额定功率（默认 1 MW，即 $S_{fw}=0.01$）
- 转矩与功率换算：$T_e = \dfrac{P_{fes,pu}\cdot S_{base}}{\omega_f}$

---

## 4. 组合调频控制（飞轮核心价值）

飞轮功率参考由三项叠加，**这是飞轮区别于常规储能的关键**：

$$
\Delta P_{fes}^* = \underbrace{2 H_{fes} \frac{d\Delta f}{dt}}_{\text{虚拟惯量}} +
\underbrace{K_f (f_0 - f)}_{\text{一次调频下垂}} +
\underbrace{P_{AGC}}_{\text{二次调频}}
$$

### 4.1 虚拟惯量（惯性响应）

$$
\Delta P_{inertia} = 2 H_{fes} \frac{d\Delta f}{dt} = 2 H_{fes} \frac{df}{dt}
$$

其中 $H_{fes}$ 为飞轮等效虚拟惯量常数。虚拟惯量**抑制频率变化率 RoCoF**，在扰动瞬间（$df/dt$ 最大时）快速注入功率，是飞轮"秒级响应"能力的体现。

### 4.2 一次调频下垂（含死区 + 测量延时）

$$
\Delta P_{droop} =
\begin{cases}
-K_f (f - f_0), & |f - f_0| \ge \varepsilon \\
0, & |f - f_0| < \varepsilon
\end{cases}
$$

- $K_f$：下垂系数（pu/Hz）
- $\varepsilon$：一次调频死区（典型 $\pm 0.033$ Hz），避免飞轮对小波动频繁动作

频率测量含一阶低通延时环节：

$$
\frac{d f_m}{dt} = \frac{1}{T_m}(f - f_m)
$$

其中 $f_m$ 为滤波后频率，$T_m$ 为测量时间常数（典型 0.1 s）。

### 4.3 二次调频（AGC）

$$
\Delta P_{AGC} = K_{p,agc} \cdot \text{ACE} + K_{i,agc} \int \text{ACE}\, dt
$$

其中区域控制偏差 ACE（Area Control Error）：

$$
\text{ACE} = \Delta f + \sum_j B_{ij} \Delta P_{tie,ij}
$$

在简化模型中（本 Skill 默认 $K_{p,agc}=0$），AGC 采用纯积分控制消除稳态频差：

$$
\frac{d P_{AGC}}{dt} = K_{i,agc} \cdot \Delta f
$$

---

## 5. 完整状态空间

系统状态向量 $\mathbf{x} \in \mathbb{R}^{3N+4}$：

$$
\mathbf{x} = [\delta_1,\dots,\delta_N,\; \omega_1,\dots,\omega_N,\; P_{m1},\dots,P_{mN},\; \omega_f,\; f_m,\; P_{AGC},\; \xi_{agc}]^T
$$

总状态数 $= 3N + 4 = 3\times 8 + 4 = 28$（8 区系统），为刚性系统，采用 `ode15s` 求解。

---

## 6. 关键指标定义

| 指标 | 定义 | 物理意义 |
|---|---|---|
| 频率最低点 nadir | $\min_t f(t)$ | 频率跌落最深处，衡量调频效果 |
| 最大 RoCoF | $\max_t |df/dt|$ | 频率变化率峰值，惯性响应压制目标 |
| 稳态频差 | $f(\infty) - f_0$ | AGC 二次调频应消除 |
| 调频里程 | $\int |P_{fes}| dt$ | 飞轮能量贡献的累计度量 |

---

## 7. 模型假设与适用边界（论文需声明）

1. **等值聚合**：每区多台机组聚合为 1 台等值机，忽略区内机组间振荡模式（区内电磁暂态远快于机电暂态）。
2. **联络线线性化**：同步功率系数基于小扰动线性化，适用于功角差较小的场景。
3. **负荷模型**：忽略负荷电压/频率的二次特性，仅用阻尼系数 $D_i$ 综合表征。
4. **飞轮损耗**：仅考虑电机效率 $\eta$，忽略真空风阻与轴承损耗的瞬时动态（对调频秒级过程影响小，稳态损耗可另行叠加）。
5. **PMSM 简化**：采用 $i_d=0$ 矢量控制，忽略电流内环动态（带宽远高于机电时间尺度）。

这些假设是学术界飞轮储能调频研究的通用做法，与参考文献（陈俊涛、王宪、周玉洁等）一致。
