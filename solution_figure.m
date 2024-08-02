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
    plot(qdSolutions(i,:)); 
end

% 关节加速度
qddSolutions=zeros(7,steps);
for i = 1:(size(qdSolutions,2)-1)
qddSolutions(:,i)=qdSolutions(:,i+1)-qdSolutions(:,i);
end
figure('Name', '关节加速度');
hold on;
for i = 1:7 
    plot(qddSolutions(i,:)); 
end

% 跟踪误差
positon_error=zeros(3,steps);
for i = 1:size(qSolutions,2)
    T08 = getTransform(robot,[qSolutions(:,i);0;0],target, 'panda_link0');
    p08 = T08(1:3, 4)';
    difference = trajectory(:,i) - p08';
    positon_error(:,i)=difference;
end
figure('Name', '位置误差');
plot(positon_error')




