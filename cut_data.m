function new_arr = cut_data(begin_t, end_t, arr)
% cut_data 根据头尾时刻截断数据
%   new_arr = cut_data(begin_t, end_t, arr)
%       begin_t 起始时刻
%       end_t   结束时刻
%       arr     原数据列表

% 找头
k1 = find(arr(:, 1) >= begin_t, 1);

if ~isempty(k1)
    new_begin = k1;
else
    new_begin = 1;
end

% 找尾
k2 = find(arr(:, 1) <= end_t, 1, 'last');

if ~isempty(k2)
    new_end = k2;
else
    new_end = length(arr(:, 1));
end

% 截断
new_arr = arr(new_begin:new_end, :);
end
