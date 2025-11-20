%% 方案一：模式切换型 lam（右转正档，左转负档）
clear; clc;
fprintf('Program started (Mode-Switch lam)\n');

%% ====== 全局变量：给 UI 回调用 ======
global k_level;
k_level = 0;   % 初始为 0：直行

%% ====== 初始化 CoppeliaSim 接口 ======
client = RemoteAPIClient();
sim = client.require('sim');
sim.addLog(sim.verbosity_scriptinfos, "MATLAB client connected " + string(datetime));

% 获取关节句柄
j1 = sim.getObject('/p1');
j2 = sim.getObject('/p2');
j3 = sim.getObject('/p3');
j4 = sim.getObject('/p4');
j5 = sim.getObject('/p5');
j6 = sim.getObject('/p6');
J = [j1 j2 j3 j4 j5 j6];
N = numel(J);%??????
fprintf('Joint handles: '); disp(J);

%% ====== 蜿蜒波参数 ======
L     = 0.275;
Kn    = 1;
delta = 2*pi*Kn/N;
f     = 0.6;                 % 蜿蜒频率 [Hz]
A     = deg2rad(45);         % 摆幅
beta  = 0.0;
phi   = 0.0;
timeStop = 40;               % 仿真总时间 [s]可以按照要求来改
%% ====== lam 模式切换参数（这里调变化时间） ======
LAM_RISE_TIME = 1.0;   % 直行 -> 转弯，lam 从 0→1 的时间!!!写死了，可以改成1/f使得权值覆盖整个周期
LAM_FALL_TIME = 1.0;   % 转弯 -> 直行，lam 从 1→0 的时间

lam= 0;    % 权值lam
lam_dir = 0;    % 0: 静止  +1: 上升  -1: 下降
lam_t0  = 0;    % 本次变化开始时间

%% ====== 启动仿真（步进模式） ======启动仿真对象
sim.stopSimulation(); pause(0.5);
sim.setStepping(true);
sim.startSimulation();
fprintf("Simulation started\n");
%% ====== 创建 UI：k 档位按钮 ======
hFig = figure('Name', '模式切换型：k 控制面板', ...
              'NumberTitle', 'off', ...
              'MenuBar', 'none', ...
              'ToolBar', 'none', ...
              'Position', [100 100 500 160]);
uicontrol('Style', 'text', ...
          'Parent', hFig, ...
          'String', '点击按钮设置 k 档位：正数=右转, 负数=左转, 0=直行', ...
          'FontSize', 11, ...
          'HorizontalAlignment', 'center', ...
          'Units', 'normalized', ...
          'Position', [0.05 0.72 0.9 0.25]);
% 第1行：0, 1, 2, 3, 4, 5, 6
vals_row1 = [0 1 2 3 4 5 6];
for idx = 1:length(vals_row1)
    val = vals_row1(idx);
    uicontrol('Style', 'pushbutton', ...
              'Parent', hFig, ...
              'String', num2str(val), ...
              'FontSize', 11, ...
              'Units', 'normalized', ...
              'Position', [0.05+(idx-1)*0.12 0.4 0.10 0.2], ...
              'Callback', @(src,evt)setKLevel(val));
end
% 第2行：-1, -2, -3, -4
vals_row2 = [-1 -2 -3 -4];
for idx = 1:length(vals_row2)
    val = vals_row2(idx);
    uicontrol('Style', 'pushbutton', ...
              'Parent', hFig, ...
              'String', num2str(val), ...
              'FontSize', 11, ...
              'Units', 'normalized', ...
              'Position', [0.05+(idx-1)*0.12 0.1 0.10 0.2], ...
              'Callback', @(src,evt)setKLevel(val));
end

%% ====== 记录数组（时间、指令角、实际角、lam、k） ======
timeLog      = [];
thetaCmdLog  = [];   % N 列
thetaActLog  = [];   % N 列
lamLog       = [];
kSignedLog   = [];
prev_turning = false;%用于模式切换（转弯与直行）默认当前不转弯，模式切换时触发lam权值切换
t_prev= 0;%更新帧数的时间
%% ====== 主控制循环 ======反复循环这段指令直到超过总时长
while true
    t = sim.getSimulationTime();
    if t >= timeStop
        break;
    end
    dt = t - t_prev;%每帧隔了多久就更新一次
    if dt <= 0
        dt = 0.01;
    end
    t_prev = t;
    % ---- 当前 k 档位与转弯方向 ----
     global k_level
    k_level_now = k_level;           % 0, 1..6, -1..-4
    K_STEP_DEG = 1;% 每档 1°，可按需要改，不易超过2，除非减少档数（限位角度-A）/挡数
    k_deg= K_STEP_DEG * abs(k_level_now);%先变成角度值方便输入改变角度
    k_rad= deg2rad(k_deg);%偏执量统一成弧度制用于计算
    k_sign = sign(k_level_now); % 取k_level_now符号定方向，-1 左, 0 直行, +1 右
    k_signed_rad = k_sign * k_rad;  % 带方向的 k
    turning = (k_level_now ~= 0);   % 是否处于"转弯模式"
    % ---- 检测模式切换，触发 lam 变化 ----
    if (~prev_turning) && turning
        % 直行 -> 转弯：lam 从 0 -> 1
        lam_dir = +1;%lam启动逐渐增加到1
        lam_t0  = t;%记录这次开始上升的时间起点
        fprintf('[lam] Start RISE at t=%.3f s, k_level=%d\n', t, k_level_now);
    elseif prev_turning && (~turning)
        % 转弯 -> 直行：lam 从 1 -> 0
        lam_dir = -1;%lam启动逐渐减小到0
        lam_t0  = t;% 记录这次开始下降的时间起点
        fprintf('[lam] Start FALL at t=%.3f s\n', t);
    end
    prev_turning = turning;
    % ---- 更新 lam 值（只在模式切换时平滑变化）----
    if lam_dir == +1
        tau = t - lam_t0;
        if tau <= LAM_RISE_TIME%在LAM_RISE_TIME内完成权值变化
            lam = tau / LAM_RISE_TIME;%lam均匀增加
        else
            lam = 1;%增加完毕，保持转弯偏置姿态
            lam_dir = 0;%关闭lam值变化
        end
    elseif lam_dir == -1
        tau = t - lam_t0;
        if tau <= LAM_FALL_TIME
            lam = 1 - tau / LAM_FALL_TIME;
        else
            lam = 0;
            lam_dir = 0;
        end
    end
    % lam_dir == 0 时，lam 保持 0 或 1 不变

    % ---- 生成每个关节的目标角度 ----
    thetaCmd = zeros(1, N);
    for i = 1:N
        idx = i - 1;  % 从 0 开始
        base_wave   = beta + A * sin(idx*delta - 2*pi*f*t + phi);
        bias_offset = lam * (i-1) * k_signed_rad;  % 这里用 (i-1)，尾部偏置最大
        theta       = base_wave - bias_offset;     % 正负方向看需求改 +/- 
        thetaCmd(i) = theta;
        sim.setJointTargetPosition(J(i), theta);
    end

    % ---- 推进一步仿真 ----
    sim.step();

    % ---- 读取实际关节角度 ----
    thetaAct = zeros(1, N);
    for i = 1:N
        thetaAct(i) = sim.getJointPosition(J(i));
    end

    % ---- 记录 ----
    timeLog(end+1, 1)      = t;           %#ok<SAGROW>
    thetaCmdLog(end+1, :)  = thetaCmd;    %#ok<SAGROW>
    thetaActLog(end+1, :)  = thetaAct;    %#ok<SAGROW>
    lamLog(end+1, 1)       = lam;
    kSignedLog(end+1, 1)   = k_signed_rad;

    drawnow limitrate;
end

%% ====== 收尾：停止仿真，关闭 UI ======
sim.stopSimulation();
fprintf('Simulation ended (Mode-Switch lam)\n');
if isvalid(hFig)
    close(hFig);
end

%% ====== 画图：每个关节"目标 vs 实际" ======
figure('Name', '方案一：模式切换型 lam - 关节角度对比');
for i = 1:N
    subplot(N, 1, i);
    plot(timeLog, rad2deg(thetaCmdLog(:, i)), '--', ...
         timeLog, rad2deg(thetaActLog(:, i)), '-');
    ylabel(sprintf('J%d [deg]', i));
    if i == 1
        title('方案一：命令角度 vs 实际角度');
    end
    if i == N
        xlabel('时间 [s]');
    end
    legend('cmd', 'act');
    grid on;
end

fprintf('Program ended (Mode-Switch lam)\n');

%% ====== UI 回调函数 ======
function setKLevel(val)
    global k_level;
    k_level = val;
    fprintf('k_level 切换为 %d\n', val);
end
