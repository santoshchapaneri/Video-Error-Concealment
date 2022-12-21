function myJND = ComputeJND(I)

% I = input image using imread

[height width] = size(I);
I = double(I);
bg = func_bg(I); % Average background luminance

myJND = zeros(height, width);

for r = 1:height
    for c = 1:width
        if bg(r,c) <= 127
            myJND(r,c) = 17*(1-sqrt(bg(r,c)/127)+3);
        else
            myJND(r,c) = (3/128)*(bg(r,c)+127)+3;
        end
    end
end

end

function output = func_bg(input)
Mask=[1 1 1 1 1
      1 2 2 2 1
      1 2 0 2 1
      1 2 2 2 1
      1 1 1 1 1];
output = filter2(Mask,input)/32;
% output = input;
end