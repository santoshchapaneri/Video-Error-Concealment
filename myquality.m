% img1=imread('foreman2_orig.bmp','bmp');
% i1=img1(1:144,1:176,1);
% i2=img1(1:144,1:176,2);
% i3=img1(1:144,1:176,3);
% [yorig,u,v] = rgb2yuv(i1,i2,i3,'YUV420_8');
% 
% % img2=imread('foremanout2P_copy.bmp','bmp');
% % i1=img2(1:144,1:176,1);
% % i2=img2(1:144,1:176,2);
% % i3=img2(1:144,1:176,3);
% % [ycopy,u,v] = rgb2yuv(i1,i2,i3,'YUV420_8');
% % 
% % quality = img_qi(double(yorig),double(ycopy));
% 
% img3=imread('foremanout2P_BMA.bmp','bmp');
% i1=img3(1:144,1:176,1);
% i2=img3(1:144,1:176,2);
% i3=img3(1:144,1:176,3);
% [ybma,u,v] = rgb2yuv(i1,i2,i3,'YUV420_8');
% 
% quality = img_qi(double(yorig),double(ybma));
% disp('BMA = '); disp(quality);
% 
% img4=imread('foremanout2P_OBMA.bmp','bmp');
% i1=img4(1:144,1:176,1);
% i2=img4(1:144,1:176,2);
% i3=img4(1:144,1:176,3);
% [yobma,u,v] = rgb2yuv(i1,i2,i3,'YUV420_8');
% 
% quality = img_qi(double(yorig),double(yobma));
% disp('OBMA = '); disp(quality);
% 
% img5=imread('foremanout2P_ABS.bmp','bmp');
% i1=img5(1:144,1:176,1);
% i2=img5(1:144,1:176,2);
% i3=img5(1:144,1:176,3);
% [yabs,u,v] = rgb2yuv(i1,i2,i3,'YUV420_8');
% 
% quality = img_qi(double(yorig),double(yabs));
% disp('ABS = '); disp(quality);
% 
% img6=imread('foremanout2P_ABS_OBMA.bmp','bmp');
% i1=img6(1:144,1:176,1);
% i2=img6(1:144,1:176,2);
% i3=img6(1:144,1:176,3);
% [yabsobma,u,v] = rgb2yuv(i1,i2,i3,'YUV420_8');
% 
% quality = img_qi(double(yorig),double(yabsobma));
% disp('ABS_OBMA = '); disp(quality);
% 
% img7=imread('foremanout2P_ABS_OBMA_OBMC.bmp','bmp');
% i1=img7(1:144,1:176,1);
% i2=img7(1:144,1:176,2);
% i3=img7(1:144,1:176,3);
% [yabsobmaobmc,u,v] = rgb2yuv(i1,i2,i3,'YUV420_8');
% 
% quality = img_qi(double(yorig),double(yabsobmaobmc));
% disp('ABS_OBMA_OBMC = '); disp(quality);
% 
% % Apr 26 4pm 
% %>> myquality
% % BMA = 
% %     0.7961
% % 
% % OBMA = 
% %     0.8055
% % 
% % ABS = 
% %     0.7980
% % 
% % ABS_OBMA = 
% %     0.8062
% % 
% % ABS_OBMA_OBMC = 
% %     0.8077

% TABLE Sequence

img1=imread('tableorig3P','bmp');
i1=img1(1:144,1:176,1);
i2=img1(1:144,1:176,2);
i3=img1(1:144,1:176,3);
[yorig,u,v] = rgb2yuv(i1,i2,i3,'YUV420_8');
quality = img_qi(double(yorig),double(yorig));
disp('3-Original = '); disp(quality);

img1=imread('tableout3P_BMA','bmp');
i1=img1(1:144,1:176,1);
i2=img1(1:144,1:176,2);
i3=img1(1:144,1:176,3);
[y,u,v] = rgb2yuv(i1,i2,i3,'YUV420_8');
quality = img_qi(double(yorig),double(y));
disp('3-BMA = '); disp(quality);

img1=imread('tableout3P_ABS_OBMA_OBMC','bmp');
i1=img1(1:144,1:176,1);
i2=img1(1:144,1:176,2);
i3=img1(1:144,1:176,3);
[y,u,v] = rgb2yuv(i1,i2,i3,'YUV420_8');
quality = img_qi(double(yorig),double(y));
disp('3-ABS-OBMA-OBMC = '); disp(quality);

img1=imread('tableorig5P','bmp');
i1=img1(1:144,1:176,1);
i2=img1(1:144,1:176,2);
i3=img1(1:144,1:176,3);
[yorig,u,v] = rgb2yuv(i1,i2,i3,'YUV420_8');
quality = img_qi(double(yorig),double(yorig));
disp('5-Original = '); disp(quality);

img1=imread('tableout5P_BMA','bmp');
i1=img1(1:144,1:176,1);
i2=img1(1:144,1:176,2);
i3=img1(1:144,1:176,3);
[y,u,v] = rgb2yuv(i1,i2,i3,'YUV420_8');
quality = img_qi(double(yorig),double(y));
disp('5-BMA = '); disp(quality);

img1=imread('tableout5P_ABS_OBMA_OBMC','bmp');
i1=img1(1:144,1:176,1);
i2=img1(1:144,1:176,2);
i3=img1(1:144,1:176,3);
[y,u,v] = rgb2yuv(i1,i2,i3,'YUV420_8');
quality = img_qi(double(yorig),double(y));
disp('5-ABS-OBMA-OBMC = '); disp(quality);

