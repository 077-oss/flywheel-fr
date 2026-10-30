%% fr_sim_run.m —— 飞轮储能调频仿真核心求解器（独立文件）
%  输入参数结构体 P，输出结果结构体 R。
%  供 fr_sim.m / fr_base.m / fr_plot.m / 参数扫描脚本调用。

function R = fr_sim_run(P)
% 核心仿真入口：输入参数结构体 P，输出结果结构体 R。

% ==================== 参数解包 ====================
nArea   = P.nArea;            % 区域数（8）
ws      = 2*pi*P.f0;          % 同步角速度标幺基准 rad/s（标幺下 ws=2*pi*f0）
f0      = P.f0;               % 额定频率 Hz

% 发电机参数（每区）
H     = P.H;                  % 惯量常数 (s)          [1 x nArea]
D     = P.D;                  % 阻尼系数 (pu)         [1 x nArea]
Rg    = P.Rg;                 % 调差系数 (pu)         [1 x nArea]
Tg    = P.Tg;                 % 调速器时间常数 (s)    [1 x nArea]
Tij   = P.Tij;                % 联络线同步功率系数 (pu/rad) [nArea x nArea]

% 负荷基准（标幺）
PL0   = P.PL0;                % 初始负荷 [1 x nArea]

% 飞轮参数
kb      = P.fw_area;          % 飞轮接入区号 (1~nArea)
Jfw     = P.Jfw;              % 飞轮转动惯量 kg*m^2
wfw_max = P.wfw_max_rpm * 2*pi/60;   % 最大角速度 rad/s
Sfw     = P.Sfw;              % 飞轮额定功率基准 MVA（用于标幺）
wfw0    = P.wfw0_rpm * 2*pi/60;      % 初始角速度 rad/s
p       = P.p;                % 极对数
psi_f   = P.psi_f;            % 永磁磁链 Wb
Ld      = P.Ld; Lq = P.Lq;    % 电感 H
Temax   = P.Temax;            % 最大电磁转矩限幅 N*m

% 控制参数
Hfes  = P.Hfes;               % 虚拟惯量常数 (s)
Kf    = P.Kf;                 % 一次调频下垂系数 (pu/Hz)
epsf  = P.epsf;               % 一次调频死区 (Hz)
Tm    = P.Tm;                 % 频率测量延时时间常数 (s)
Kp_agc= P.Kp_agc;             % AGC 比例系数
Ki_agc= P.Ki_agc;             % AGC 积分系数
ACE_ref = P.ACE_ref;          % AGC 频率目标偏差（一般 0）

% 扰动场景
scenario = P.scenario;        % 'step' 阶跃负荷 / 'renew' 新能源波动 / 'both'
dist_area= P.dist_area;       % 扰动发生区
dPL     = P.dPL;              % 阶跃负荷幅值 (pu)
t_dist  = P.t_dist;           % 扰动发生时刻 (s)
ren_amp = P.ren_amp;          % 新能源波动幅值 (pu)
ren_f   = P.ren_f;            % 新能源波动频率 (Hz)

% 仿真控制
tspan  = P.tspan;             % [t0 tf]
tstep  = P.tstep;             % 输出采样步长 (s)

% ==================== 状态变量索引 ====================
% 状态向量 x:
%   [1:nArea]         deltai  (rad)  功角
%   [nArea+1:2nArea]  wi      (pu)   角速度（频率标幺）
%   [2nArea+1:3nArea] Pmi     (pu)   机械功率
%   [3nArea+1]        wf      (rad/s)飞轮角速度
%   [3nArea+2]        fm      (pu)   滤波后频率（测量延时）
%   [3nArea+3]        Pagc    (pu)   AGC 输出功率
%   [3nArea+4]        xi_agc  (状态) AGC 积分项
Nx = 3*nArea + 4;

% ==================== 初始稳态 ====================
delta0 = zeros(nArea,1);
% 由潮流确定初始功角（简化：忽略联络线初始角差，从零开始）
w0     = ones(nArea,1);       % 初始角速度 = 1 pu
% 初始机械功率 = 初始电功率 = 初始负荷（忽略网损）
Pm0    = PL0(:);              % 初始机械功率 = 负荷
wf0    = wfw0;                % 飞轮初始角速度
fm0    = 1.0;                 % 初始滤波频率 = 1 pu
Pagc0  = 0.0;
xi0    = 0.0;

x0 = [delta0; w0; Pm0; wf0; fm0; Pagc0; xi0];

% ==================== 负荷/扰动定义 ====================
% 扰动功率（标幺，注入到 dist_area 区，负荷增加为正）
if strcmp(scenario,'step')
    PL_dist = @(t) dPL * (t >= t_dist);
elseif strcmp(scenario,'renew')
    PL_dist = @(t) ren_amp * sin(2*pi*ren_f*t) * (t >= t_dist);
else % both
    PL_dist = @(t) dPL * (t >= t_dist) + ...
                   ren_amp * sin(2*pi*ren_f*t) * (t >= t_dist);
end

% ==================== 微分方程 ====================
odefun = @(t,x) system_ode(t, x, P, ws, Tij, PL_dist);

% 求解（ode15s 适合可能刚性的多机系统）
opts = odeset('RelTol',1e-6,'AbsTol',1e-8);
sol  = ode15s(odefun, tspan, x0, opts);

% 采样输出
t_out = tspan(1):tstep:tspan(end);
X     = deval(sol, t_out)';

% ==================== 提取结果 ====================
R.t      = t_out';
R.delta  = X(:, 1:nArea);
R.w      = X(:, nArea+1:2*nArea);          % 各机角速度 (pu)
R.f      = R.w * f0;                        % 各机频率 (Hz)
R.Pm     = X(:, 2*nArea+1:3*nArea);         % 机械功率 (pu)
R.wf     = X(:, 3*nArea+1);                 % 飞轮角速度 (rad/s)
R.fm     = X(:, 3*nArea+2);                 % 滤波频率 (pu)
R.Pagc   = X(:, 3*nArea+3);                 % AGC 功率 (pu)

% 飞轮输出功率（标幺，注入电网为正=放电）与 SOC
R.Pfw    = compute_fw_power(X, P, ws);      % [N x 1] 飞轮电功率 pu
R.SOC    = (R.wf / wfw_max).^2;             % SOC

% 各机电磁功率（负荷+联络线）
R.Pe = compute_Pe(X, P, Tij, PL_dist, t_out);

% 频率最低点（接入区及全网平均）
[~, kmin] = min(R.f(:, kb));
R.f_nadir = R.f(kmin, kb);
R.t_nadir = t_out(kmin);
R.f_mean  = mean(R.f, 2);
[~, kmin_mean] = min(R.f_mean);
R.f_nadir_mean = R.f_mean(kmin_mean);
R.t_nadir_mean = t_out(kmin_mean);

% RoCoF（频率变化率，Hz/s）
dt = t_out(2)-t_out(1);
R.RoCoF = [diff(R.f(:,kb))./dt; 0];

% 稳态频差
R.f_ss = R.f(end, kb) - f0;

% 调频里程（飞轮能量贡献的近似度量，标幺·s）
R.mileage = trapz(t_out, abs(R.Pfw));

% ==================== 写指标报告（仅主仿真，无飞轮基准不覆盖报告） ====================
if P.enable_fw
    write_report(R, P);
end

end

%% ==================== 系统微分方程 ====================
function dx = system_ode(t, x, P, ws, Tij, PL_dist)
nArea = P.nArea;
f0    = P.f0;
kb    = P.fw_area;

% 解包状态
delta = x(1:nArea);
w     = x(nArea+1:2*nArea);
Pm    = x(2*nArea+1:3*nArea);
wf    = x(3*nArea+1);
fm    = x(3*nArea+2);      % 滤波频率（标幺）
Pagc  = x(3*nArea+3);
xi_agc= x(3*nArea+4);

% 发电机参数
H  = P.H(:); D = P.D(:); Rg = P.Rg(:); Tg = P.Tg(:);

% 联络线功率流
Ptie = zeros(nArea,1);
for i = 1:nArea
    for j = 1:nArea
        Ptie(i) = Ptie(i) + Tij(i,j)*(delta(i) - delta(j));
    end
end

% 负荷（含扰动）
PL = P.PL0(:);
PL(P.dist_area) = PL(P.dist_area) + PL_dist(t);

% 飞轮注入功率（放电为正），只在接入区
% 虚拟惯量项 = 2*Hfes * dfm（dfm 为滤波频率导数，标幺/s，已在下方计算）
% 为保证方程自治，先计算 dfm，再回代组合调频功率。
Pfw = 0;
if kb >= 1 && kb <= nArea
    dfm = (w(kb) - fm) / P.Tm;   % 滤波频率导数（标幺/s）
    Pfw = fw_power_ref(fm, Pagc, dfm, P);   % 组合调频功率（含惯量+下垂+AGC）
end
% 飞轮开关：enable_fw=0 时飞轮不注入功率（基准场景）
if ~P.enable_fw
    Pfw = 0;
end

% 电磁功率 = 负荷 + 联络线 - 飞轮注入
Pe = PL + Ptie;
Pe(kb) = Pe(kb) - Pfw;

% 摆动方程： 2H * dw/dt = Pm - Pe - D*(w-1)   （标幺角频率）
dwd = (Pm - Pe - D.*(w-1)) ./ (2*H);

% 功角： ddelta/dt = ws*(w-1)
ddelta = ws * (w - 1);

% 调速器： Tg*dPm/dt = Pref - Pm - (1/Rg)*(w-1)
% 设 Pref 初始 = PL0，AGC 修正叠加在飞轮上而非发电机（简化）
dPm = ( -Pm - (1./Rg).*(w-1) + P.Pref(:) ) ./ Tg;

% ---- 飞轮转子动力学 ----
% 电磁转矩 Te = 1.5*p*(psi_f*iq + (Ld-Lq)*id*iq)
% 简化控制：采用 id=0 矢量控制，Te = 1.5*p*psi_f*iq
% 功率参考 Pref_fw 由组合调频给出，iq 由转矩/功率关系确定并限幅
Pref_fw = Pfw;   % 标幺功率（放电为正），已含惯量+下垂+AGC
% 飞轮机械功率（吸收/释放）：P_real = Te * wf
% 功率关系：Pfw_real(W) = Pfw_pu * Sbase(W)，转矩 Te = Pfw_real / wf
eta = P.eta;
if Pref_fw >= 0   % 放电：Te 为正，克服损耗（电功率 < 机械功率）
    Te = (Pref_fw * P.Sbase) / (wf * eta);
else              % 充电：Te 为负
    Te = (Pref_fw * P.Sbase) * eta / wf;
end
% 转矩限幅
Te = max(-P.Temax, min(P.Temax, Te));
% SOC 下限保护：转速低于最低转速时禁止继续放电（飞轮退出调频）
wfw_min = P.wfw_min_rpm * 2*pi/60;
if (wf <= wfw_min) && (Te > 0)
    Te = 0;   % 储能耗尽，停止放电
end
% 转子方程： J*wf*dwf/dt = -Te*wf  =>  dwf/dt = -Te/J
dwf = -Te / P.Jfw;

% ---- 频率测量延时（一阶低通）----
% 取接入区频率标幺 w(kb)
dfm = (w(kb) - fm) / P.Tm;

% ---- AGC 二次调频（积分控制消除稳态频差）----
dxi = (P.ACE_ref - (fm - 1));     % ACE 简化为频率偏差（无联络线偏差项）
dPagc = P.Kp_agc * dxi + P.Ki_agc * xi_agc;

% 组装
dx = [ddelta; dwd; dPm; dwf; dfm; dPagc; dxi];
end

%% ==================== 飞轮组合调频功率参考 ====================
function Pref = fw_power_ref(fm, Pagc, dfm, P)
% 组合调频功率参考（放电为正）：
%   Pref = 虚拟惯量 + 一次调频下垂(含死区) + AGC
% 输入：
%   fm   滤波后频率（标幺，1=额定）
%   Pagc AGC 功率（标幺）
%   dfm  滤波频率导数（标幺/s），用于虚拟惯量 2H·df/dt
% 输出：标幺功率参考（放电为正），已限幅。
f0  = P.f0;
df  = fm - 1;                    % 频率偏差（标幺）
dfHz= df * f0;                   % 频率偏差（Hz）

% 一次调频死区
if abs(dfHz) < P.epsf
    Kf_eff = 0;
else
    Kf_eff = P.Kf;
end

% 一次调频下垂（含死区）：频率偏低(fm<1)时放电为正
Pdroop = Kf_eff * (-dfHz);

% 虚拟惯量：2*Hfes * (dΔf/dt)。dfm 为标幺频率导数，乘以 f0 换算 Hz/s
% ΔP_inertia = 2*Hfes * df/dt，放电为正对应频率下降(df/dt<0)。
P_inertia = 2 * P.Hfes * (-dfm * f0);

% 三项叠加
Pref = P_inertia + Pdroop + Pagc;
% 功率限幅
Pref = max(-P.Pfw_max, min(P.Pfw_max, Pref));
end

%% ==================== 飞轮输出功率计算 ====================
function Pfw = compute_fw_power(X, P, ws)
% 从状态重构飞轮电功率（含虚拟惯量项，与闭环一致）
% 用梯度法从滤波频率序列重构 dfm，使后处理功率与闭环注入功率一致。
nArea = P.nArea; kb = P.fw_area; f0 = P.f0;
N = size(X,1);
fm_all = X(:, 3*nArea+2);   % 滤波频率序列（标幺）
Pagc_all= X(:, 3*nArea+3);  % AGC 功率序列
% 时间轴（由调用方保证等步长；此处用 ode 采样，需要传入时间）
% 由于本函数缺时间轴，改用状态差分近似：dfm ≈ (fm 变化)/(步长)
% 这里通过 P.tstep 获取步长
dt = P.tstep;
dfm_all = [0; diff(fm_all)] / dt;   % 滤波频率导数（标幺/s）
Pfw = zeros(N,1);
for k = 1:N
    Pfw(k) = fw_power_ref(fm_all(k), Pagc_all(k), dfm_all(k), P);
end
end

%% ==================== 电磁功率计算 ====================
function Pe = compute_Pe(X, P, Tij, PL_dist, t_out)
nArea = P.nArea;
N = length(t_out);
delta = X(:, 1:nArea);
PL = P.PL0(:);
Pe = zeros(N, nArea);
for k = 1:N
    t = t_out(k);
    PLk = PL;
    PLk(P.dist_area) = PLk(P.dist_area) + PL_dist(t);
    Ptie = zeros(nArea,1);
    for i = 1:nArea
        for j = 1:nArea
            Ptie(i) = Ptie(i) + Tij(i,j)*(delta(k,i)-delta(k,j));
        end
    end
    Pe(k,:) = PLk' + Ptie';
end
end

%% ==================== 指标报告 ====================
function write_report(R, P)
outdir = fullfile(fileparts(mfilename('fullpath')), 'output');
if ~exist(outdir,'dir'); mkdir(outdir); end
fp = fopen(fullfile(outdir,'report.txt'),'w','n','UTF-8');
fprintf(fp, '===== 飞轮储能调频仿真指标报告 =====\n');
fprintf(fp, '系统：%d 区等值机互联，飞轮接入区 %d\n', P.nArea, P.fw_area);
fprintf(fp, '扰动场景：%s（扰动区 %d）\n', P.scenario, P.dist_area);
fprintf(fp, '--- 频率指标（接入区） ---\n');
fprintf(fp, '频率最低点 (nadir): %.4f Hz @ t=%.2f s\n', R.f_nadir, R.t_nadir);
fprintf(fp, '全网平均频率最低点: %.4f Hz @ t=%.2f s\n', R.f_nadir_mean, R.t_nadir_mean);
fprintf(fp, '最大 RoCoF: %.4f Hz/s\n', max(abs(R.RoCoF)));
fprintf(fp, '稳态频差: %.4f Hz\n', R.f_ss);
fprintf(fp, '--- 飞轮指标 ---\n');
fprintf(fp, '飞轮峰值输出功率: %.4f pu\n', max(abs(R.Pfw)));
fprintf(fp, '飞轮 SOC 初值: %.4f  终值: %.4f\n', R.SOC(1), R.SOC(end));
fprintf(fp, '调频里程(近似): %.4f pu*s\n', R.mileage);
fclose(fp);
fprintf('REPORT_OK\n');
end

