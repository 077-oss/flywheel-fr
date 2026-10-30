# Simulink 对照模型搭建指南

`fr_sim.m` 是纯脚本（ode15s）实现，适合快速改参、批量仿真、出论文图。为验证模型正确性并提供可视化拓扑，可搭建等价的 Simulink 模型作对照。

## 对照原则

同一套微分方程，两种实现方式，结果应在数值容差内一致：

| 环节 | 脚本实现 | Simulink 对应模块 |
|---|---|---|
| 摆动方程 | `dwd = ws/(2H)*(Pm-Pe-D*(w-1))` | Integrator + Gain（$1/2H$）|
| 调速器 | 一阶微分方程 | Transfer Fcn：$\dfrac{1/R}{1+T_g s}$ |
| 联络线 | `sum(Tij*(δi-δj))` | Gain（$T_{ij}$）+ Sum |
| 飞轮转子 | `dwf = -Te/J` | Integrator + Gain（$-1/J$）|
| 一次调频死区 | `if abs(dfHz)<eps` | Dead Zone 模块（$\pm\varepsilon$）|
| 测量延时 | 一阶低通 | Transfer Fcn：$\dfrac{1}{1+T_m s}$ |
| AGC | 积分器 | Integrator（$K_{i,agc}$）|

## 搭建步骤

### Step 1：创建模型文件

```matlab
new_system('fw_fr_simulink');
open_system('fw_fr_simulink');
```

### Step 2：单区发电机子系统（封装为 Subsystem）

内部结构：
```
    ┌─────────┐    ┌──────────┐
Δω─→│ 1/Ri    │─→  │ 1/(1+Tgi s) │─→ Pmi
    └─────────┘    └──────────┘
                            │
    Pmi ─┐                  │
          ├─(Σ)─→ 1/(2Hi s) ─→ Δω
    -Pe ──┘    (× ws)
```

- Gain `1/Ri`、Transfer Fcn `1/(1+Tgi*s)`、Integrator + Gain `ws/(2Hi)`
- 阻尼项：`-D*Δω` 反馈到求和点

### Step 3：联络线耦合

区 i 与 j 之间：
```
Δδi ──→ (×Tij) ─┐
                 ├─(Σ)─→ Ptie,ij
Δδj ──→ (×Tij) ─┘
```

8 区环形互联：每区与相邻两区交换功率。

### Step 4：飞轮子系统

```
       ┌─────────────────────────────────────┐
       │  组合调频控制                         │
Δf ──→ │  [DeadZone ±ε]→(×Kf) ─┐             │
       │                        ├─(Σ)─→ Pref │
df/dt→ │  (×2Hfes) ────────────┘             │
       │  [Integrator Ki]→Pagc ─┘            │
       └─────────────────────────────────────┘
                          │
                    ┌─────▼─────┐
              Pref→│ 功率→转矩  │→ Te → 1/J ∫ → ωf → SOC
                    └───────────┘
```

### Step 5：对照验证

```matlab
% 脚本结果
R = fr_sim();
% Simulink 结果
simOut = sim('fw_fr_simulink');
% 对比频率曲线
figure; plot(R.t, R.f(:,1)); hold on;
plot(simOut.tout, simOut.freq); legend('脚本','Simulink');
```

## 注意事项

1. **标幺基准一致**：脚本与 Simulink 的功率基准必须统一（默认 1 pu = 系统总额定容量）。
2. **求解器**：Simulink 建议用 `ode15s`（刚性）或 `ode23tb`，固定步长 1e-3 s 以下。
3. **死区模块**：`Dead Zone` 的 Start/End 值设为 $\pm\varepsilon$（Hz 需换算到标幺）。
4. **代数环**：飞轮功率参考依赖频率测量（含延时），反馈回路中已有积分/一阶环节，不会形成代数环；若出现，可在反馈路径加 `Memory` 模块或 1e-6 时间常数的低通。
