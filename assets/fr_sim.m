%% fr_sim.m —— 飞轮储能参与多区系统频率调节仿真（MATLAB R2025a）
%  8 区等值机互联系统 + 集中式飞轮储能（详细部件模型）+ 组合调频控制。
%  组合控制 = 虚拟惯量(惯性响应) + 一次调频下垂(含死区+测量延时) + 二次调频(AGC)。
%
%  用法：
%   1) 直接运行本脚本，输出全部曲线与指标报告到 output/ 目录；
%   2) 或作为函数调用：[R] = fr_sim('param_file.m') 返回结果结构体。
%
%  模型（SI 单位制，均采用标幺值 per-unit 进入微分方程）：
%   同步发电机摆动方程：  2H_i * d(wi)/dt = Pmi - Pei - Di*(wi - 1)
%   调速器-原动机一阶：    Tgi * d(Pmi)/dt = Prefi - Pmi - (1/Ri)*(wi - 1)
%   联络线同步功率：       Pei = PLi + sum_j Tij*(deltai - deltaj)
%   飞轮转子：             J*wf*dwf/dt = -Te*wf  （Te 为 PMSM 电磁转矩）
%   飞轮 SOC：             SOC = E/Emax = (wf/wfmax)^2
%
%  组合调频功率参考（接入区 kb）：
%     dP_fes = 2*Hfes*df/dt + Kf*(f0-f) + Pagc
%  其中一次调频含死区 eps 与测量延时 Tm；AGC 用积分器实现二次调频。

function R = fr_sim(varargin)
% fr_sim —— 飞轮储能调频仿真入口
% 用法：
%   R = fr_sim();              % 默认参数（含飞轮）
%   R = fr_sim('params_x.m');  % 自定义参数文件
%   R = fr_sim('params_x.m', 0);  % 第2参数 enable_fw=0 关闭飞轮（基准场景）
if nargin >= 1 && ~isempty(varargin{1}) && ischar(varargin{1})
    run(varargin{1});          % 期望该脚本里定义 params 结构体
    P = params;
else
    P = default_params();
end
if nargin >= 2 && ~isempty(varargin{2})
    P.enable_fw = varargin{2};
end
R = fr_sim_run(P);
end
