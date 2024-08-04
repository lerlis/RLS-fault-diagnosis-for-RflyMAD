%% Load data
% in_command = ones(1,1005);
% out_command = [5*ones(1,500),10*ones(1,5),5*ones(1,500)];
clear all;clc;
load_manipulate_data;
diagnosis_method = 1; % 0 for roll error, 1 for pitch error
if diagnosis_method == 0
    in_command = command_re(1, :);
    out_command = euler_re(1, :)';
elseif diagnosis_method == 1
    in_command = command_re(2, :);
    out_command = euler_re(2, :)';
end
% in_command = ones(1,length(z_re));
TotalNum = length(out_command);
%% Initialize
% RLS
error_term = out_command - in_command;
N = 20; % RLS单次使用的数据
C_k = eye(2*N) * 1e5;
theta_k = zeros(2*N, 1);
data_k = zeros(2*N, 1);
for i = 1 : 1 : N
    data_k(2*i-1) = in_command(i);
    data_k(2*i) = error_term(i);
end
% Welford
error_k_stack = [];
M_k = 0;
avg_error_last = 0;
sigma_stack = [];
% Z-score
Z_score_stack = [];
%% Step
for i = N : 1 : TotalNum-1
    % RLS
    error_k = out_command(i) - data_k'* theta_k;
    L_k = C_k * data_k / (1 + data_k'*C_k*data_k);
    % RLS Update
    theta_k = theta_k + L_k * error_k; % 计算theta_{k+1}
    C_k = C_k - L_k*data_k'*C_k; % 计算C_{k+1}
    data_k = UpdateData(data_k, in_command(i+1), error_term(i+1));
    % Welford
    error_k_stack = [error_k_stack, error_k];
    M_k = M_k + (error_k - avg_error_last)*(error_k - mean(error_k_stack));
    avg_error_last = mean(error_k_stack);
    sigma = sqrt(M_k/length(error_k_stack));
    sigma_stack = [sigma_stack, sigma];
    z_score = (error_k -  avg_error_last) / sigma;
    Z_score_stack = [Z_score_stack, z_score];
end
error_k_stack = [zeros(1,N), error_k_stack];
sigma_stack = [zeros(1,N), sigma_stack];
Z_score_stack = [zeros(1,N), Z_score_stack];
Flight_time = (target_timevec(end) - target_timevec(1)) / onesec;
%% Plot
index = find_fault_time(param_re(2, :));
if index == -1
    GTValue = zeros(1, TotalNum);
else
    GTValue = [zeros(1, index), 4.5*ones(1, TotalNum-index)];
end

if diagnosis_method == 0
    figure('Name', 'Roll Error Z-Score', 'Position', [0 0 1000 600]);
elseif diagnosis_method == 1
    figure('Name', 'Pitch Error Z-Score', 'Position', [0 0 1000 600]);
end
plot(target_timevec / onesec, abs(Z_score_stack), 'b', 'linewidth', 1.1);
hold on;grid on;
plot(target_timevec / onesec, 150*sigma_stack, 'm', 'linewidth', 1.4);
plot(target_timevec / onesec, GTValue, 'r', 'linewidth', 1.4);
legend({'Z-Score', 'Standard Deviation', 'Ground Truth'}, 'FontSize',13.5);
xlabel('time(s)', 'FontName', 'Times New Roman', 'FontSize', 20);
ylabel('Z-Score', 'FontName', 'Times New Roman', 'FontSize', 20);
ylim([0 5.5]);
if diagnosis_method == 0
    title('Roll Error Z-Score', 'FontSize', 20);
elseif diagnosis_method == 1
    title('Pitch Error Z-Score', 'FontSize', 20);
end

