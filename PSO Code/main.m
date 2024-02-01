
clear all
close all
clc

%********* Multiple UAVs path planning using PSO
%Nr_d=2;  % the number of UAV=2
%SAI=10*ones(10);
SAI=[1 1 1 7 8 1 1 1 10 1 1 1 1 7 8 1 1 1 1 1;
    1 2 1 1 7 7 5 1 1 1 1 2 1 1 7 7 5 1 1 1 ;
    7 8 1 1 5 1 1 1 1 1 7 8 1 1 5 1 1 1 10 1;
    5 6 1 2 2 8 7 7 1 8 5 1 1 2 2 8 7 7 9 8;
    9 1 1 8 4 1 1 8 7 1 9 1 1 5 4 1 1 8 7 1 ;
    1 4 5 7 1 1 2 7 7 7 1 4 5 7 1 1 2 7 7 7 ;
    5 4 7 8 1 1 2 5 4 3 5 4 7 8 1 1 2 5 4 3 ;
    4 1 1 5 2 1 4 3 1 1 4 1 1 5 2 1 4 3 6 7;
    6 5 1 4 1 1 7 5 3 6 6 5 1 4 1 1 7 5 9 9 ;
    7 6 1 1 5 1 10 4 7 8 7 6 1 1 7 1 10 4 7 8 
    1 10 1 7 8  1 1 1 1 1 1 1 1 7 8  1 1 1 1 1;
    1 2 1 1 7 7 5 1 1 1 1 2 1 1 7 7 5 1 8 1 ;
    7 8 1 1 5 1 1 1 1 1 7 8 1 1 5 1 1 10 1 1;
    5 6 1 2 2 8 7 7 9 8 5 6 1 2 2 8 7 7 9 8;
    9 1 1 8 9 1 1 8 7 1 9 1 1 8 4 1 1 8 7 1 ;
    1 4 5 7 1 1 2 7 7 7 1 4 5 7 1 1 2 7 7 7 ;
    5 4 7 8 1 1 2 5 4 3 5 4 7 8 1 1 2 5 4 3 ;
    4 8 8 5 2 1 4 3 6 7 4 8 8 5 2 1 4 3 6 7;
    6 5 1 4 1 1 7 5 9 9 6 5 1 4 1 1 7 5 9 9 ;
    7 6 1 1 10 1 10 4 7 8 7 6 1 1 10 1 10 4 7 8];
% finding path for each UAV
ht=-1;
while ht<0
ht1=-1;
while ht1<0
    [bestpath1,pbest1,ec1,fr1,sai1]=pso(SAI);
     ht1=pbest1(1);
end
% set the SAI of the cells which are already covered by UAV 1 to zero
for i=1:8
    r1=bestpath1(i,1);
    r2=bestpath1(i,2);
    for i=r1:r1+1
        for j=r2:r2+1
         SAI(i,j)=0;
        end
    end
    % so far,this part not completed because of neighboring cell not
    % considering
    % finish already
end
% Umair
% save('bestpaths.mat','bestpath1','sensitivity','-append')

% planing the trajectory for UAV 2 based on the updated SAI matrix
ht2=-1;
while ht2<0
    [bestpath2,pbest2,ec2,fr2,sai2]=pso(SAI);
     ht2=pbest2(1);
end

% Collision Aviodance Constraint (冲撞检测)
dmax=0.1;
CAC=0;
cac=0;
for i=1:8
    for j=1:8
        A=bestpath1(i,:);
        B=bestpath2(j,:);
        %以下省略的部分是的为了实现两条路径x和y不相交的条件：很浪费运行时间
%         if A(1)==B(1) & A(2)==B(2)
%             cac2=-1;
%         else
%             cac2=0;
%         end
        d=sqrt((A(1)-B(1))^2+(A(2)-B(2))^2+(A(3)-B(3))^2);
        %再这里加上x,y的限制条件来限制
        
        if d>dmax
            cac1=0;
        else
            cac1=-1;
        end  
        cac=cac+cac1; %+cac2;
    end
    CAC=CAC+cac;
end
save('bestpaths.mat','bestpath1','bestpath2','-append')
ht=CAC;
if ht<0
    ht=-1;
else
    ht=0;
end
end

% SAI updated mechanism
% compute the number of surveillence time to cover all area
for i=1:8
    r1=bestpath2(i,1);
    r2=bestpath2(i,2);
    for i=r1:r1+1
        for j=r2:r2+1
         SAI(i,j)=0;
        end
    end
end
% store planning path results of all flight time


