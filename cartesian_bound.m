function [ddxLim,ddxMax,ddxMin] = cartesian_bound(xbound,dxbound,target,robot,q)
J=geometricJacobian(robot,[q;0;0], target);
J=J(4:6,1:7);
T = getTransform(robot,[q;0;0],target, 'panda_link0');
x_target = T(1:3, 4);
ddxLim=J;
ddxMax=min(dxbound(:,2),20*(xbound(:,2)-x_target));
ddxMin=max(dxbound(:,1),20*(xbound(:,1)-x_target));
end

