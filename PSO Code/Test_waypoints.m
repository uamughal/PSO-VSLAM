clear all
clc
close all

w1=1;
w2=1;
w3=2;
ec=[26 14.22 4 45 4 9 32.2490];
fr= [1.6979 1.3763 2.3171 2.4379 1.2916 0.8472 1.1941];
sai=[28 23 20 33 23 10 15];
TAC=[0 0 0 -1 0 0 0 0];
ec=ec/56.6480;
fr=fr/5.7500;
sai=sai/40;

for i=1:length(sai)
Fobv(i)=-w1*ec(i)-w2*fr(i)+w3*sai(i);

fit(i)=Fobv(i)+ TAC(i);
end
t= fit;