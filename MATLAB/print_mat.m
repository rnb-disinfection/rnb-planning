function [mat] = print_mat(mat)
%PRINT_MAT �� �Լ��� ��� ���� ��ġ
%   �ڼ��� ���� ��ġ
[dx, dy] = size(mat);
disp("[")
for ix = 1:dx
    linestr = "[";
    for iy = 1:dy
        linestr = [linestr + mat(ix,iy) + ","];
    end
    linestr = [linestr + "],"];
    disp(linestr);
end
disp("]")

