function W = mat_pqr2euler(phi, theta)
%mat_pqr2euler 计算机体角速度向欧拉角速度变换的矩阵
%   W = mat_pqr2euler(phi, theta)
%       phi     滚转角
%       theta   俯仰角

W = [1 tan(theta) * sin(phi) tan(theta) * cos(phi);
    0 cos(phi) -sin(phi);
    0 sin(phi) / cos(theta) cos(phi) / cos(theta)];
end
