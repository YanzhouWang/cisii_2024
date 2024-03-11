function xi = get_twist(F)
%GET_TWIST Summary of this function goes here
%   Detailed explanation goes here
% R = F(1:3, 1:3);
% t = F(1:3, 4);
% 
% omegatheta = logm(R);
% 
% theta = acos((R(1,1)+R(2,2)+R(3,3)-1)/2);
% if theta < 1e-6
%     omega = [1,1,1];
% else
%     omega = (1/2*sin(theta))*[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)]


% logm

xiHat = logm(F);
v = xiHat(1:3,4);
omega = [xiHat(3,2);xiHat(1,3);xiHat(2,1)];

xi = [v;omega];


end

