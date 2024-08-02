clc;
clear;
close all;

% 导入机器人
robot = importrobot('panda.urdf',"DataFormat" , "column");

% 跟踪目标
target='panda_grasptarget';

% 关节物理极限
qdmax=[2.62,2.62,2.62,2.62,5.26,4.18,5.26]';qdlow=-qdmax;
qlow=[-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159]';
qmax=[ 2.7437,  1.7837,  2.9007, -0.1518,  2.8065, 4.5169,  3.0159]';

% 采样时间设置
delta_t = 0.01;duration = 10;steps= duration/delta_t;

% 轨迹生成
start_point_t2=[0.55,0.13,0.6];
[trajectory,td] = generate_square(start_point_t2, duration, steps);%生成心形轨迹
point_c=[0.33,-0.06,0.65];

% 初始构型
start_config=[0.470893845888598,0.127875358808930,1.612361685285212e-08,-1.733620831648753,-9.896810450588840e-09,1.935877135576721,-0.190196085937739]';

% 二次规划系数矩阵，最小速度
H= eye(7);

%存放结果
qSolutions=[start_config,zeros(7,steps-1)];
qdSolutions=zeros(7,steps);

% 循环求解
for s=(1:steps-1)

    J=geometricJacobian(robot,[qSolutions(:,s);0;0], target);
    J=J(4:6,1:7);
    dXMin=max(qdlow,(qlow-qSolutions(:,s))/delta_t);
    dXMax=min(qdmax,(qmax-qSolutions(:,s))/delta_t);

    % 生成速度期望
    v=td(:,s)+100*cartesian_error(robot,qSolutions(:,s),trajectory(:,s),target);

    % 任务 双臂末端位置跟踪
    Aeq=J;
    beq=v;
    A_LIM=eye(7);

    % 求解
    A=[A_LIM;-A_LIM];b=[dXMax;-dXMin];
    options = optimoptions('quadprog','display','None');
    qd=quadprog(H,[],A,b,J,v,[],[],[],options);

    qdSolutions(:,s)=qd; % 更新当前时刻速度
    qSolutions(:,s+1)=qSolutions(:,s)+qdSolutions(:,s)*delta_t;% 更新下一时刻位置
end

% 最后时刻速度设置为0
qdSolutions(:,steps)=zeros(7,1);

% video;
