function [out_data] = UpdateData(data,in_data1, in_data2)
% ɾ����ǰ�������ݣ����������µ�
    data(1:2) = [];
    new_data = [in_data1, in_data2]';
    out_data = [data;new_data];
end

