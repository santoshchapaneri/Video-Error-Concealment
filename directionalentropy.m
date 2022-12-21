c = [400 3954 1824 0 0 0 0 0];
sumc = sum(c);

pc = c/sumc;

lenc = length(c);

sumh = 0;

for i=1:lenc
    if (pc(i)~=0)
        sumh = sumh + ((-pc(i))*log2(pc(i)));
    end
end

sumh