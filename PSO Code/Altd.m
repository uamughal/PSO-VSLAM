%************terrain function********
%**********Foxhole Shekel function*********
%Altd(x,y) //return terrain altitude of position (x,y)
function [alt]=Altd(x,y)
xx=0:0.1:20;
yy=0:0.1:20;
[X,Y]=meshgrid(xx,yy);
% Z=(cos(2*X+1)+2*cos(3*X+2)).*(cos(2*Y+1)+2*cos(3*Y+2));
%UM Z=(cos(2*X+1)+2*cos(3*X+2)).*(sin(2*Y+1)+2*sin(3*Y+2));
% surf(Z)
% figure;
A=[4,1,8,6,3,2,5,8,6,7;
   4,1,8,6,7,9,3,1,2,3.6];
C=1/10.*[1,2,2,4,4,6,3,6,4,2];
Q=0;
for i=1:10 % for i goes to 10 in the funtion
  Q=Q+0.1./((X-A(1,i)).*(X-A(1,i))+(Y-A(2,i)).*(Y-A(2,i))+C(i));
end
%for j=1:101
   % for k=1:101
   % x=xx(k);
    %y=yy(k);
alt=Q(x*10+1,y*10+1);
%************end of the function******************

% surf(X,Y,Q)
% figure
% mesh(X,Y,Q)
% figure
% interp3(X,Y,Q)
% figure
% interp3 (X,Y,Q,x,y)

% axis equal
% shading interp
% hold on
% plot3(X,Y,Z, 'o')


% surf(X,Y,Q)
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% figure
% % contour(X,Y,Q)
% % xlabel('X')
% % ylabel('Y')


