clc
close all
clear all
%  dd=dir('*.jpeg');
dd=dir('*.PNG');
%  mkdir('resized')
for cntr = 1: length(dd)
img= imread(dd(cntr).name);
imgR = imresize(img, [480 640]);

imwrite(imgR,['resized\' dd(cntr).name]);


end