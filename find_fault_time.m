function [index] = find_fault_time(param)
%FIND_FAULT_TIME �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
    index = -1;
    flag = 0;
    len = length(param);
    for i = 1 : 1 : len
        if param(i) > 0.5
            flag = 1;
            break;
        end
    end
    if flag == 1
        index = i;
    end
end

