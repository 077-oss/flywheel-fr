%% params_example.m —— 飞轮储能调频仿真参数文件（用户可修改）
%  用法：在 fr_sim 中作为参数文件传入，或在脚本里 run 本文件后使用 params。
%  本文件定义 params 结构体，覆盖 default_params 中的全部字段。
%  复制本文件为 params_你的项目.m，改数值即可，无需改核心代码。

params = struct();

% ---- 系统规模 ----
params.nArea = 8;              % 区域数
params.f0    = 50.0;           % 额定频率 Hz

% ---- 发电机参数（每区，1×nArea 行向量）----
params.H  = 5.0 * ones(1,8);   % 惯量常数 s
params.D  = 1.0 * ones(1,8);   % 阻尼系数 pu
params.Rg = 0.05 * ones(1,8);  % 调差系数 pu（5%）
params.Tg = 0.3 * ones(1,8);   % 调速器时间常数 s
params.Pref = 0.8 * ones(8,1); % 初始机械功率参考 pu

% ---- 联络线同步功率系数（pu/rad，环形互联）----
Tij = zeros(8,8);
for i = 1:8
    j = mod(i,8)+1;
    Tij(i,j) = 1.5; Tij(j,i) = 1.5;
end
params.Tij = Tij;

% ---- 初始负荷 ----
params.PL0 = 0.8 * ones(8,1);  % pu

% ---- 飞轮参数（复用 flywheel_sim.m：m=500kg r=0.6m）----
params.Jfw = 90.0;             % 转动惯量 kg*m^2（=0.5*500*0.6^2）
params.wfw_max_rpm = 20000;    % 最大转速 RPM
params.wfw0_rpm    = 15000;    % 初始转速 RPM（75% SOC）
params.wfw_min_rpm = 8000;     % 最低转速 RPM（SOC 下限保护）
params.Sbase = 100e6;          % 系统基准功率 (W)
params.Sfw   = 0.05;           % 飞轮额定功率占系统基准比例（5MW/100MW）
params.p    = 2;               % 极对数
params.psi_f= 0.4;             % 永磁磁链 Wb
params.Ld   = 1e-3;            % d 轴电感 H
params.Lq   = 1e-3;            % q 轴电感 H
params.Temax= 5000;            % 最大电磁转矩 N*m
params.eta  = 0.95;            % 电机效率

% ---- 组合调频控制 ----
params.Hfes   = 3.0;           % 虚拟惯量常数 s
params.Kf     = 0.5;           % 一次调频下垂系数 pu/Hz
params.epsf   = 0.033;         % 一次调频死区 Hz
params.Tm     = 0.1;           % 频率测量延时时间常数 s
params.Kp_agc = 0.0;           % AGC 比例系数
params.Ki_agc = 0.3;           % AGC 积分系数
params.ACE_ref= 0;             % AGC 频率目标偏差
params.Pfw_max= 0.05;          % 飞轮功率限幅 pu（=Sfw）

% ---- 扰动场景 ----
params.scenario  = 'step';     % 'step' / 'renew' / 'both'
params.dist_area = 3;          % 扰动发生区
params.dPL       = 0.10;       % 阶跃负荷幅值 pu
params.t_dist    = 5.0;        % 扰动时刻 s
params.ren_amp   = 0.03;       % 新能源波动幅值 pu
params.ren_f     = 0.2;        % 新能源波动频率 Hz

% ---- 仿真控制 ----
params.tspan = [0 60];         % 仿真时间区间 s
params.tstep = 0.01;           % 输出采样步长 s

% ---- 飞轮接入区 ----
params.fw_area = 1;            % 飞轮接入区号 1~nArea

% ---- 飞轮开关 ----
params.enable_fw = 1;           % 1=启用飞轮调频，0=无飞轮基准场景（对比用）
