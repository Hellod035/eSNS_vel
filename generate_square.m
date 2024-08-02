function [trajectory, td] = generate_square(center_point, duration, steps)
    % 长方形的四个顶点
    half_length_z = 0.4 / 2;
    half_length_y = 0.3 / 2;
    
    % 调整顶点的顺序，使轨迹从左上角开始
    points = [
        center_point(1), center_point(2) + half_length_y, center_point(3) - half_length_z;
        center_point(1), center_point(2) + half_length_y, center_point(3) + half_length_z;
        center_point(1), center_point(2) - half_length_y, center_point(3) + half_length_z;
        center_point(1), center_point(2) - half_length_y, center_point(3) - half_length_z;
        
    ];

    % 每段的步数，处理分配剩余的部分
    segment_steps = floor(steps / 4);
    remaining_steps = steps - 4 * segment_steps;
    segment_steps_list = [segment_steps, segment_steps, segment_steps, segment_steps];
    
    % 将剩余的步数均匀分配到各段
    for i = 1:remaining_steps
        segment_steps_list(i) = segment_steps_list(i) + 1;
    end
    
    trajectory = [];
    td = [];
    
    for i = 1:4
        start_point = points(mod(i-1, 4) + 1, :);
        end_point = points(mod(i, 4) + 1, :);
        
        [segment_trajectory, segment_td] = generate_trajectory_segment(start_point, end_point, duration/4, segment_steps_list(i));
        
        % 合并轨迹，不去掉重复点
        trajectory = [trajectory, segment_trajectory];
        td = [td, segment_td];
    end
    
end

function [trajectory, td] = generate_trajectory_segment(start_point, end_point, duration, steps)
    delta_t = duration / steps;
    % 计算每一步的时间间隔
    time = linspace(0, duration, steps);
    
    % 初始化轨迹矩阵
    trajectory = zeros(3, steps);
    trajectory(1, :) = time; % 第一行是时间
    
    % 对每个维度计算轨迹（x, y, z）
    for dim = 1:3
        % 边界条件为起始和结束的位置
        p0 = start_point(dim);
        pf = end_point(dim);
        
        % 基于位置边界条件计算三次多项式系数
        coeffs = calc_quintic_coeffs(p0, pf, duration);
        
        % 计算轨迹
        trajectory(dim, :) = polyval(coeffs, time);
    end

    td = [];
    for i = 1:(size(trajectory, 2) - 1)
        td = [td, (trajectory(1:3, i+1) - trajectory(1:3, i)) / delta_t];
    end
    td = [td, zeros(3, 1)];
end

function coeffs = calc_quintic_coeffs(p0, pf, tf)
    % This function calculates the coefficients of a quintic polynomial
    % given the initial position (p0), final position (pf), and the final time (tf).
    % The polynomial is of the form: p(t) = a*t^5 + b*t^4 + c*t^3 + d*t^2 + e*t + f

    % Boundary conditions:
    % p(0) = p0, p(tf) = pf, p'(0) = 0, p'(tf) = 0, p''(0) = 0, p''(tf) = 0
    
    % Constructing the system of equations based on the boundary conditions
    A = [0, 0, 0, 0, 0, 1;                % p(0) = p0
         tf^5, tf^4, tf^3, tf^2, tf, 1;  % p(tf) = pf
         0, 0, 0, 0, 1, 0;                % p'(0) = 0
         5*tf^4, 4*tf^3, 3*tf^2, 2*tf, 1, 0; % p'(tf) = 0
         0, 0, 0, 2, 0, 0;                % p''(0) = 0
         20*tf^3, 12*tf^2, 6*tf, 2, 0, 0];% p''(tf) = 0

    b = [p0; pf; 0; 0; 0; 0];
    
    % Solving for the coefficients
    coeffs = A\b;
end
