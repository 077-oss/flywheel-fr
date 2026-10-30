%% fr_plot.m —— 飞轮储能调频仿真论文级配图生成（MATLAB R2025a）
%  读取 fr_sim.m 的仿真结果，生成论文级配图（矢量字体、统一配色、清晰标注）。
%  核心输出：有无飞轮对比图（频率响应 + RoCoF + 飞轮功率/SOC + 各机功率）。
%
%  用法：
%    R = fr_sim();          % 有飞轮
%    R0 = fr_sim_base();    % 无飞轮基准
%    fr_plot(R, R0);        % 生成对比配图
%    % 或只给有飞轮结果： fr_plot(R)

function fr_plot(R, R0)
if nargin < 1 || isempty(R)
    R = fr_sim();
end
% 无飞轮基准：若未传入，则自动重跑（关闭飞轮）
if nargin < 2 || isempty(R0)
    R0 = fr_sim('', 0);   % 第2参数 0 = 关闭飞轮
end

% 输出目录
outdir = fullfile(fileparts(mfilename('fullpath')), 'output');
if ~exist(outdir,'dir'); mkdir(outdir); end

% 字体
font = 'Microsoft YaHei';
if ~any(strcmpi(listfonts, font)); font = 'SimHei'; end
set(0,'DefaultAxesFontName',font);
set(0,'DefaultTextFontName',font);
set(0,'DefaultAxesFontSize',11);

% 配色（论文常用，高对比、打印友好）
c_blue = [0.00 0.45 0.74];
c_red  = [0.85 0.33 0.10];
c_green= [0.47 0.67 0.19];
c_purp = [0.49 0.18 0.56];
c_gray = [0.40 0.40 0.40];

t  = R.t;
f  = R.f;
Pfw= R.Pfw;
SOC= R.SOC;
kb = 1;   % 飞轮接入区（与默认一致）

% ============ 图1：接入区频率响应（有无飞轮对比） ============
figure('Color','w','Position',[50 50 780 480]);
plot(t, f(:,kb), 'Color', c_blue, 'LineWidth', 2.0); hold on;
plot(t, R0.f(:,kb), '--', 'Color', c_gray, 'LineWidth', 1.6);
yline(50, ':k', 'LineWidth', 0.6);
% 标注频率最低点（错开放置，避免重叠）
[fn, in] = min(f(:,kb));
[fn0, in0] = min(R0.f(:,kb));
plot(t(in), fn, 'o', 'MarkerEdgeColor', c_blue, 'MarkerFaceColor', c_blue, 'MarkerSize', 7);
plot(t(in0), fn0, 's', 'MarkerEdgeColor', c_gray, 'MarkerFaceColor', c_gray, 'MarkerSize', 6);
% 手动扩展 y 轴范围，确保标注在可显示区
ylim([min([fn,fn0])-0.02, 50.005]);
% 含飞轮 nadir 标注（左上方）
text(t(in)-1.0, fn+0.006, sprintf('nadir = %.3f Hz', fn), 'Color', c_blue, 'FontSize', 9, ...
    'HorizontalAlignment','right','VerticalAlignment','bottom');
% 无飞轮 nadir 标注（右下方，深灰以醒目）
text(t(in0)+1.0, fn0-0.008, sprintf('nadir = %.3f Hz', fn0), 'Color', [0.25 0.25 0.25], 'FontSize', 9, ...
    'HorizontalAlignment','left','VerticalAlignment','top');
legend('含飞轮','无飞轮','额定频率','Location','northeast');
xlabel('时间 (s)'); ylabel('频率 (Hz)');
title('接入区频率响应：飞轮储能调频效果对比');
xlim([0 t(end)]); grid on; box on;
saveas(gcf, fullfile(outdir,'fig1_frequency_compare.png'));

% ============ 图2：RoCoF 对比 ============
dt = t(2)-t(1);
RoCoF  = [diff(f(:,kb))./dt; 0];
RoCoF0 = [diff(R0.f(:,kb))./dt; 0];
figure('Color','w','Position',[50 50 780 480]);
plot(t, RoCoF, 'Color', c_blue, 'LineWidth', 1.8); hold on;
plot(t, RoCoF0, '--', 'Color', c_gray, 'LineWidth', 1.5);
yline(0, ':k', 'LineWidth', 0.6);
legend('含飞轮','无飞轮','Location','northeast');
xlabel('时间 (s)'); ylabel('RoCoF (Hz/s)');
title('频率变化率 RoCoF 对比：虚拟惯量压制效果');
xlim([0 t(end)]); grid on; box on;
saveas(gcf, fullfile(outdir,'fig2_rocof_compare.png'));

% ============ 图3：飞轮输出功率与 SOC ============
figure('Color','w','Position',[50 50 780 480]);
yyaxis left;
plot(t, Pfw, 'Color', c_green, 'LineWidth', 1.8);
ylabel('飞轮输出功率 (pu)');
yyaxis right;
% SOC 变化量级极小（飞轮储能容量大），显示为百分比偏差以清晰呈现
dSOC = (SOC - SOC(1)) * 100;   % 相对初值的百分比变化
plot(t, dSOC, 'Color', c_purp, 'LineWidth', 1.8);
ylabel('飞轮 SOC 变化量 (%)');
xlabel('时间 (s)');
title('飞轮输出功率与 SOC 变化');
xlim([0 t(end)]); grid on; box on;
legend('输出功率','SOC 变化','Location','best');
saveas(gcf, fullfile(outdir,'fig3_flywheel_power_soc.png'));

% ============ 图4：各机组机械功率分配 ============
figure('Color','w','Position',[50 50 780 480]);
cmap = lines(size(R.Pm,2));
for i = 1:size(R.Pm,2)
    plot(t, R.Pm(:,i), 'Color', cmap(i,:), 'LineWidth', 1.2); hold on;
end
xlabel('时间 (s)'); ylabel('机械功率 (pu)');
title('各区域等值机机械功率');
xlim([0 t(end)]); grid on; box on;
legend(arrayfun(@(i) sprintf('区%d',i), 1:size(R.Pm,2), 'UniformOutput', false), ...
    'Location','eastoutside');
saveas(gcf, fullfile(outdir,'fig4_machine_power.png'));

fprintf('PLOT_OK\n');
end
