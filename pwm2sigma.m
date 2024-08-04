function sigma = pwm2sigma(pwm)
% pwm2sigma PWN占空比转油门指令
%   数学关系为 sigma = (pwm - 1000) / 1000
%   由于可能存在误差因此需要将[0,1]之外的值进行处理

% 归一化得到油门
t = (pwm(:, 2) - 1000) / 1000;
% 异常值处理
t(t < 0) = 0;
t(t > 1) = 1;
% 返回值包含时间戳
sigma = [pwm(:, 1), t];

end
