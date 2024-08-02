function position_error = cartesian_error(robot,q,trajectory,target)
config1=[q(1:7);0;0];


T1 = getTransform(robot,config1,target, 'panda_link0');
p1 = T1(1:3, 4)';
position_error=trajectory-p1';

end