% 关节角
figure('Name', '机器人关节角');
hold on;
for i = 1:7 
    plot(qSolutions(i,:)); 
end

% 关节速度
figure('Name', '关节速度');
hold on;
for i = 1:7 
    plot(dqSolutions(i,:)); 
end

% 跟踪误差
positon_error=zeros(3,steps);
for i = 1:size(qSolutions,2)
    T = getTransform(robot,[qSolutions(:,i);0;0],target, 'panda_link0');
    p = T(1:3, 4)';
    difference = trajectory(:,i) - p';
    positon_error(:,i)=difference;
end
figure('Name', '位置误差');
plot(positon_error')

%缩放因子
figure('Name', '缩放因子');
plot(sdata')


%机械臂笛卡尔极限
x_target=zeros(3,steps);
dx_target=zeros(3,steps);

for i = 1:size(qSolutions,2)
    J=geometricJacobian(robot,[qSolutions(:,i);0;0], cartesian_bound_target);
    J=J(4:6,1:7);
    T = getTransform(robot,[qSolutions(:,i);0;0],cartesian_bound_target, 'panda_link0');
    x_target(:,i) = T(1:3, 4);
    dx_target(:,i)=J*dqSolutions(:,i);
end

figure('Name', '笛卡尔位置');
plot(x_target')
% figure('Name', '笛卡尔速度');
% plot(dx_target')






