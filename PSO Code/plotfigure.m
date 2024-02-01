%************terrain function********
%**********Foxhole Shekel function*********
%Altd(x,y) //return terrain altitude of position (x,y)
%function [alt]=Altd(x,y)
xx=0:0.1:20;
yy=0:0.1:20;
[X,Y]=meshgrid(xx,yy);
% Z=(cos(2*X+1)+2*cos(3*X+2)).*(cos(2*Y+1)+2*cos(3*Y+2));
% surf(Z)
%figure;
A=[4,1,8,6,3,2,5,8,6,7;
   4,1,8,6,7,9,3,1,2,3.6];
C=1/10.*[1,2,2,4,4,6,3,6,4,2];
Q=0;
for i=1:10
  Q=Q+0.1./((X-A(1,i)).*(X-A(1,i))+(Y-A(2,i)).*(Y-A(2,i))+C(i));
end

% A=[4,1,8,6,3,2,5,8,6,7,10,11,15,9,18,12,19,7,17,13;
%     4,1,8,6,7,9,3,1,2,3.6,10,12,15,6,18,4,19,17,20,11];
% C=1/10.*[1,2,2,4,4,6,3,6,4,2,10,15,9,18,5,17,11,19,13,16];
% Q=5;
% for i=1:20
%   Q=Q+0.1./((X-A(1,i)).*(X-A(1,i))+(Y-A(2,i)).*(Y-A(2,i))+C(i));
% end
%for j=1:101
   % for k=1:101
   % x=xx(k);
    %y=yy(k);
%alt=Q(x*10+1,y*10+1);
%************end of the function******************
%load bestpaths.mat
load bestpath_UM_
surf(X,Y,Q)
hold on
a=bestpath1;
a1=bestpath2;
x=a(:,1);
y=a(:,2);
z=a(:,3);
x1=a1(:,1);
y1=a1(:,2);
z1=a1(:,3);
plot3(x,y,z);
hold on
plot3(x1,y1,z1);
xlabel('X')
ylabel('Y')
zlabel('Z')

figure
contour(X,Y,Q)
xlabel('X')
ylabel('Y')
hold on
a=bestpath1;
a1=bestpath2;
x=a(:,1);
y=a(:,2);
z=a(:,3);
x1=a1(:,1);
y1=a1(:,2);
z1=a1(:,3);
plot(x,y);
hold on
plot(x1,y1);
grid on
% rectangle('Position',[0 0 10 10],'edgecolor','r','FaceColor',[1 0 0]);
% legend('Terrain','UAV 1','UAV 2')
legend('UAV 1','UAV 2')
xlabel('X')
ylabel('Y')



rectangle('Position',[7 6 1 1],'edgecolor','b','FaceColor',[1 0 0], 'Linewidth', 2);
rectangle('Position',[11 11 1 1],'edgecolor','b','FaceColor',[1 0 0], 'Linewidth', 2);
rectangle('Position',[9 14  1 1],'edgecolor','b','FaceColor',[1 0 0], 'Linewidth', 2);
rectangle('Position',[13 18 1 1],'edgecolor','b','FaceColor',[1 0 0], 'Linewidth', 2);
rectangle('Position',[18 16 1 1],'edgecolor','b','FaceColor',[1 0 0], 'Linewidth', 2);
rectangle('Position',[16 8 1 1],'edgecolor','b','FaceColor',[1 0 0], 'Linewidth', 2);
rectangle('Position',[2 5 1 1],'edgecolor','b','FaceColor',[1 0 0], 'Linewidth', 2);
rectangle('Position',[8 18 1 1],'edgecolor','b','FaceColor',[1 0 0], 'Linewidth', 2);
rectangle('Position',[19 5 1 1],'edgecolor','b','FaceColor',[1 0 0], 'Linewidth', 2);
rectangle('Position',[15 4 1 1],'edgecolor','b','FaceColor',[1 0 0], 'Linewidth', 2);
rectangle('Position',[10 12 1 1],'edgecolor','b','FaceColor',[1 0 0], 'Linewidth', 2);
rectangle('Position',[14 12 1 1],'edgecolor','b','FaceColor',[1 0 0], 'Linewidth', 2);


% Source 
rectangle('Position',[0 0 1 1],'edgecolor','b','FaceColor',[0 1 0], 'Linewidth', 2);
% Destination
rectangle('Position',[18 19 1 1],'edgecolor','b','FaceColor',[0 1 0], 'Linewidth', 2);
% Restricted Area
rectangle('Position',[2 15 2 2],'edgecolor','r','FaceColor',[.5 .5 .5], 'Linewidth', 2.5);
% RegionSensitivity
rectangle('Position',(RSi),'edgecolor','b','FaceColor',[0 1 0], 'Linewidth', 2);












