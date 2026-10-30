%% default_params.m —— 飞轮储能调频仿真默认典型参数（独立文件，供各脚本调用）
function P = default_params()
P.nArea = 8;
P.f0    = 50.0;             % Hz

% 发电机（8 区等值机，参数取典型火电等值）
P.H  = 5.0 * ones(1,8);     % 惯量常数 s
P.D  = 1.0 * ones(1,8);     % 阻尼系数 pu
P.Rg = 0.05 * ones(1,8);    % 调差系数 5%
P.Tg = 0.3 * ones(1,8);     % 调速器时间常数 s
P.Pref = 0.8 * ones(8,1);   % 初始机械功率参考 pu

% 联络线同步功率系数（pu/rad），环形互联简化
Tij = zeros(8,8);
for i = 1:8
    j = mod(i,8)+1;
    Tij(i,j) = 1.5; Tij(j,i) = 1.5;
end
P.Tij = Tij;

% 初始负荷
P.PL0 = 0.8 * ones(8,1);

% 飞轮（复用 flywheel_sim.m 典型：m=500kg r=0.6m）
mfw = 500; rfw = 0.6;
P.Jfw = 0.5 * mfw * rfw^2;      % 90 kg*m^2
P.wfw_max_rpm = 20000;
P.wfw0_rpm    = 15000;          % 初始 75% 转速
P.wfw_min_rpm = 8000;           % 最低转速（SOC 下限保护，对应 SOC=16%）
P.Sbase = 100e6;                % 系统基准功率 (W)，8区总额定容量
P.Sfw   = 0.05;                 % 飞轮额定功率占系统基准比例（5 MW / 100 MW = 0.05）
P.p    = 2;                     % 极对数
P.psi_f= 0.4;                   % 永磁磁链 Wb
P.Ld   = 1e-3; P.Lq = 1e-3;     % H
P.Temax= 5000;                  % N*m 限幅（5MW飞轮在额定转速1570rad/s对应3185N*m，留裕量）
P.eta  = 0.95;

% 组合调频控制
P.Hfes  = 3.0;                  % 虚拟惯量常数 s
P.Kf    = 0.5;                  % 一次调频下垂 pu/Hz
P.epsf  = 0.033;                % 死区 Hz
P.Tm    = 0.1;                  % 测量延时 s
P.Kp_agc= 0.0;
P.Ki_agc= 0.3;                  % AGC 积分
P.ACE_ref = 0;
P.Pfw_max = 0.05;               % 飞轮功率限幅 pu（=5MW/100MW，与 Sfw 一致）

% 扰动
P.scenario = 'step';
P.dist_area = 3;
P.dPL   = 0.10;                 % 10% 阶跃负荷
P.t_dist= 5.0;
P.ren_amp = 0.03;
P.ren_f   = 0.2;

% 仿真
P.tspan = [0 60];
P.tstep = 0.01;

% 飞轮接入区
P.fw_area = 1;

% 飞轮开关（1=启用飞轮调频，0=无飞轮基准场景）
P.enable_fw = 1;
end
