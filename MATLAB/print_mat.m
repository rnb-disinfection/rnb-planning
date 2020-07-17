function [mat] = print_mat(mat)
%PRINT_MAT 이 함수의 요약 설명 위치
%   자세한 설명 위치
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

