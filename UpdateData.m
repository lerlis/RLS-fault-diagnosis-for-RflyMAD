function [out_data] = UpdateData(data,in_data1, in_data2)
% 删除最前两个数据，增添两个新的
    data(1:2) = [];
    new_data = [in_data1, in_data2]';
    out_data = [data;new_data];
end

