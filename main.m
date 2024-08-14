clc;
clear;
close all;

% 导入机器人
robot = importrobot('panda.urdf',"DataFormat" , "column");

% 跟踪目标
target='panda_grasptarget';

% 平滑因子
sm=0.2;

% 关节物理极限
dqmax=[2,2,2,2,2,2,2]';dqlow=-dqmax;
% dqmax=[2.62,2.62,2.62,2.62,5.26,4.18,5.26]';dqlow=-dqmax;
qlow=[-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159]';
qmax=[ 2.7437,  1.7837,  2.9007, -0.1518,  2.8065, 4.5169,  3.0159]';

% 关节笛卡尔极限
cartesian_bound_target="panda_link4";
xbound=[[-10;-10;-10],[0.2;10;10]];
dxbound=[[-10;-10;-10],[10;10;10]];

% 采样时间设置
delta_t = 0.01;duration = 10;steps= duration/delta_t;

% 轨迹生成
start_point=[0.55,0.13,0.6];
[trajectory,td] = generate_square(start_point, duration, steps);

% 初始构型
start_config=[0.470893845888598,0.127875358808930,1.612361685285212e-08,-1.733620831648753,-9.896810450588840e-09,1.935877135576721,-0.190196085937739]';

%次要任务
subtask_joint="panda_link4";

% 二次规划系数矩阵，最小速度
H= eye(7);

%存放结果
qSolutions=[start_config,zeros(7,steps-1)];
dqSolutions=zeros(7,steps);

% 循环求解
for s=(1:steps-1)

    J=geometricJacobian(robot,[qSolutions(:,s);0;0], target);
    J=J(4:6,1:7);
    dqMin=max(dqlow,20*(qlow-qSolutions(:,s)));
    dqMax=min(dqmax,20*(qmax-qSolutions(:,s)));
    
    %机械臂笛卡尔极限
    [dxLim,dxMax,dxMin] = cartesian_bound(xbound,dxbound,cartesian_bound_target,robot,qSolutions(:,s));
    
    % 生成速度期望
    position_error=cartesian_error(robot,qSolutions(:,s),trajectory(:,s),target);
    
    %次要任务
    J_subtask=geometricJacobian(robot,[qSolutions(:,s);0;0],subtask_joint );
    J_subtask_y=J_subtask(5,1:7);
    
    %构建输入
    ur={};
    ur{1,1}=zeros(7,1);
    ur{2,1}=zeros(7,1);
    ur{3,1}=zeros(7,1);
    ur{4,1}=zeros(7,1);
    A_LIM=eye(7);
    A={};
    A{1,1}=J(1,:);
    A{2,1}=J(2,:);
    A{3,1}=J(3,:);
    A{4,1}=J_subtask_y;
    bprime={};
    bprime{1,1}=td(1,s)+100*position_error(1,:);
    bprime{2,1}=td(2,s)+100*position_error(2,:);
    bprime{3,1}=td(3,s)+100*position_error(3,:);
    bprime{4,1}=0.2*sin(s/25);
    C={};
    C{1,1}=[A_LIM;dxLim];
    C{2,1}=zeros(1,7);
    C{3,1}=zeros(1,7);
    C{4,1}=zeros(1,7);
    dmax={};
    dmax{1,1}=[dqMax;dxMax];
    dmax{2,1}=0;
    dmax{3,1}=0;
    dmax{4,1}=0;
    dmin={};
    dmin{1,1}=[dqMin;dxMin];
    dmin{2,1}=0;
    dmin{3,1}=0;
    dmin{4,1}=0;

    %求解
    [sdata(:,s),dq] = esns_qp(A,bprime,C,dmax,dmin,ur,sm);
    
    %存储结果
    dqSolutions(:,s) = dq;
    qSolutions(:,s+1) = qSolutions(:,s) + dqSolutions(:,s) * delta_t;

end

% 最后时刻速度设置为0
ddqSolutions(:,steps)=zeros(7,1);

% video;
solution_figure;
