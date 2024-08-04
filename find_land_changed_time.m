function time = find_land_changed_time(state, inverse, start)
%find_land_changed_time 确定着陆状态变化时间，根据参数不同可以查找开始起飞、开始或完成降落时间
%   time = find_land_changed_time(state, inverse, start)
%       state   2列的状态变量
%       inverse 是否为逆向查找，用于确定完成开始时间
%       start   起始偏移时间，用于确定完成降落时间，且仅当inverse==false时起作用

if inverse
    % 逆向查找
    value = state(end, 2);
    pos = find(state(:, 2) ~= value, 1, 'last');
else
    % 确定偏移量
    offset = find(state(:, 1) >= start, 1);
    % 正向查找
    value = state(offset, 2);
    pos = offset - 1 + find(state(offset:end, 2) ~= value, 1, 'first');
end

if isempty(pos)
    time = state(end, 1);
else
    time = state(pos, 1);
end

end
