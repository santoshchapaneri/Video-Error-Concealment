I=imread('foreman_cif3001.tif');

IMB = I(33:48,49:64);

IAbove4B1 = I(29:32,49:52);
IAbove4B2 = I(29:32,53:56);
IAbove4B3 = I(29:32,57:60);
IAbove4B4 = I(29:32,61:64);

ILeft4B1 = I(33:36,45:48);
ILeft4B2 = I(37:40,45:48);
ILeft4B3 = I(41:44,45:48);
ILeft4B4 = I(45:48,45:48);

IBelow4B1 = I(49:52,49:52);
IBelow4B2 = I(49:52,53:56);
IBelow4B3 = I(49:52,57:60);
IBelow4B4 = I(49:52,61:64);

IRight4B1 = I(33:36,65:68);
IRight4B2 = I(37:40,65:68);
IRight4B3 = I(41:44,65:68);
IRight4B4 = I(45:48,65:68);

% Mostly, this nbr region is pmode 8, i.e. -22.5 angle for all 4B's

% IAbove4B1 - pmode 2 (do nothing)

% IAbove4B2 - pmode 0 (90)
EM4B2 = abs(sum(IAbove4B2(1:4,1)) - sum(IAbove4B2(1:4,4)));


% ILeft4B4 - pmode 8 (-22.5)
EM4B4 = abs(sum(sum(ILeft4B4(1:2,1:2))) - sum(sum(ILeft4B4(3:4,3:4))));

% Find its JND comparison to check if edge is relevant or not
valA = zeros(4,1);
valB = zeros(4,1);
jndM = zeros(4,1);

valA(1) = ILeft4B4(1,3);
valA(2) = ILeft4B4(1,4);
valA(3) = ILeft4B4(2,3);
valA(4) = ILeft4B4(2,4);

valB(1) = ILeft4B4(3,1);
valB(2) = ILeft4B4(3,2);
valB(3) = ILeft4B4(4,1);
valB(4) = ILeft4B4(4,2);

jndM(1) = myJNDPixel(ILeft4B4(1,1));
jndM(2) = myJNDPixel(ILeft4B4(2,2));
jndM(3) = myJNDPixel(ILeft4B4(3,3));
jndM(4) = myJNDPixel(ILeft4B4(4,4));

valAsum = sum(valA);
valBsum = sum(valB);
jndMsum = sum(jndM);

valABDiff = abs(valAsum - valBsum);
if (valABDiff > jndMsum)
    sigEdge = 1;
else
    sigEdge = 0;
end



% IBelow4B2 - pmode 8 (-22.5)
EM4B2 = abs(sum(sum(IBelow4B2(1:2,1:2))) - sum(sum(IBelow4B2(3:4,3:4))));

% Find its JND comparison to check if edge is relevant or not
valA = zeros(4,1);
valB = zeros(4,1);
jndM = zeros(4,1);

valA(1) = IBelow4B2(1,3);
valA(2) = IBelow4B2(1,4);
valA(3) = IBelow4B2(2,3);
valA(4) = IBelow4B2(2,4);

valB(1) = IBelow4B2(3,1);
valB(2) = IBelow4B2(3,2);
valB(3) = IBelow4B2(4,1);
valB(4) = IBelow4B2(4,2);

jndM(1) = myJNDPixel(IBelow4B2(1,1));
jndM(2) = myJNDPixel(IBelow4B2(2,2));
jndM(3) = myJNDPixel(IBelow4B2(3,3));
jndM(4) = myJNDPixel(IBelow4B2(4,4));

valAsum = sum(valA);
valBsum = sum(valB);
jndMsum = sum(jndM);

valABDiff = abs(valAsum - valBsum);
if (valABDiff > jndMsum)
    sigEdge = 1;
else
    sigEdge = 0;
end



% IRight4B2 - pmode 3 (-45)
EM4B2 = abs(sum(sum(IRight4B2(1:2,1:2))) - sum(sum(IRight4B2(3:4,3:4))));

% Find its JND comparison to check if edge is relevant or not
valA = zeros(4,1);
valB = zeros(4,1);
jndM = zeros(4,1);

valA(1) = IRight4B2(1,3);
valA(2) = IRight4B2(1,4);
valA(3) = IRight4B2(2,3);
valA(4) = IRight4B2(2,4);

valB(1) = IRight4B2(3,1);
valB(2) = IRight4B2(3,2);
valB(3) = IRight4B2(4,1);
valB(4) = IRight4B2(4,2);

jndM(1) = myJNDPixel(IRight4B2(1,1));
jndM(2) = myJNDPixel(IRight4B2(2,2));
jndM(3) = myJNDPixel(IRight4B2(3,3));
jndM(4) = myJNDPixel(IRight4B2(4,4));

valAsum = sum(valA);
valBsum = sum(valB);
jndMsum = sum(jndM);

valABDiff = abs(valAsum - valBsum);
if (valABDiff > jndMsum)
    sigEdge = 1;
else
    sigEdge = 0;
end


% IRight4B4 - pmode 3 (-45)
EM4B2 = abs(sum(sum(IRight4B4(1:2,1:2))) - sum(sum(IRight4B4(3:4,3:4))));

% Find its JND comparison to check if edge is relevant or not
valA = zeros(4,1);
valB = zeros(4,1);
jndM = zeros(4,1);

valA(1) = IRight4B4(1,3);
valA(2) = IRight4B4(1,4);
valA(3) = IRight4B4(2,3);
valA(4) = IRight4B4(2,4);

valB(1) = IRight4B4(3,1);
valB(2) = IRight4B4(3,2);
valB(3) = IRight4B4(4,1);
valB(4) = IRight4B4(4,2);

jndM(1) = myJNDPixel(IRight4B4(1,1));
jndM(2) = myJNDPixel(IRight4B4(2,2));
jndM(3) = myJNDPixel(IRight4B4(3,3));
jndM(4) = myJNDPixel(IRight4B4(4,4));

valAsum = sum(valA);
valBsum = sum(valB);
jndMsum = sum(jndM);

valABDiff = abs(valAsum - valBsum);
if (valABDiff > jndMsum)
    sigEdge = 1;
else
    sigEdge = 0;
end

