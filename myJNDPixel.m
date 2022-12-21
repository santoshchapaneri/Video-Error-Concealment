function jndval = myJNDPixel(pixel)

pixel = double(pixel);

if pixel <= 127
    jndval = 17*(1-sqrt(pixel/127)+3);
else
    jndval = (3/128)*(pixel+127)+3;
end