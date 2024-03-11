function Ebs = invSE3(Esb)
% Esb = Ebs^-1
%   Detailed explanation goes here

Rsb = Esb(1:3,1:3);
tsb = Esb(1:3,4);

Rbs = Rsb';
tbs = -Rbs*tsb;

Ebs = [Rbs,tbs; 0,0,0,1];
end

