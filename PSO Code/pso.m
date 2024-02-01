%PSO function 
%use to find a optimal path for each UAV
%
function [bestpath,pbest,ec,fr,sai]=pso(SAI)
% bestpath:the optimal path for each UAV using PSO
% pbest: the fitness value vector [each value for each iteration]
% ec: the energy consumption of bestpath
% fr: the flight risk of bestpath
% sai: the total sai of bestpath


% randomly generate particle speed and positions vector
N_p=128;  % particle number
D=3;   % demisionality of each particle// waypoint
N_w=8;  % waypoint number
n=0;
for i=1:N_p  
    for j=1:N_w
        for k=1:D
        switch k
            case 1
                x(j,k)=10*rand;
                v1(j,k)=0.1*rand;
            case 2
                x(j,k)=10*rand;
                v1(j,k)=0.1*rand;
            case 3
                x(j,k)=1.5*rand;
                v1(j,k)=0.1*rand;
        end
        end
        y=x';
        v2=v1';
        par1=y(:);
        v3=v2(:);
%         v3=v2;
        par2=par1';
        v4=v3';
    end
    par(i,:)=par2(:); % par is a particle
    v(i,:)=v4(:);    % speed vector
end 
% compute the fitnes% initialize the particle optimum and swarm optimum
% value of each particle

%SAI=10*rand(10,10); % so far, keep it like this way
% SAI=[1 1 1 7 8  1 1 1 1 1 1 1 1 7 8  1 1 1 1 1;

%     1 2 1 1 7 7 5 1 1 1 1 2 1 1 7 7 5 1 1 1 ;
%     7 8 1 1 5 1 1 1 1 1 7 8 1 1 5 1 1 1 1 1;
%     5 6 1 2 2 8 7 7 9 8 5 6 1 2 2 8 7 7 9 8;
%     9 1 1 8 4 1 1 8 7 1 9 1 1 8 4 1 1 8 7 1 ;
%     1 4 5 7 1 1 2 7 7 7 1 4 5 7 1 1 2 7 7 7 ;
%     5 4 7 8 1 1 2 5 4 3 5 4 7 8 1 1 2 5 4 3 ;
%     4 8 8 5 2 1 4 3 6 7 4 8 8 5 2 1 4 3 6 7;
%     6 5 1 4 1 1 7 5 1 1 6 5 1 4 1 1 7 5 9 9 ;
%     7 6 1 1 10 1 10 4 7 8 7 6 1 1 10 1 10 4 7 8; 
%     1 1 1 7 8  1 1 1 1 1 1 1 1 7 8  1 1 1 1 1;
%     1 2 1 1 7 7 5 1 1 1 1 2 1 1 7 7 5 1 1 1 ;
%     7 8 1 1 5 1 9 9 1 1 7 8 1 1 5 1 1 1 1 1;
%     5 6 1 2 2 8 7 7 9 8 5 6 1 2 2 8 7 7 9 8;
%     9 1 1 8 4 1 1 8 7 1 9 1 1 8 4 1 1 8 7 1 ;
%     1 4 5 7 1 1 2 7 7 7 1 4 5 7 1 1 2 7 7 7 ;
%     5 4 7 8 1 1 2 5 4 3 5 4 7 8 1 1 2 5 4 3 ;
%     4 8 8 5 2 1 4 3 6 7 4 8 8 5 2 1 4 3 6 7;
%     6 5 1 4 1 1 7 5 9 9 6 5 1 4 1 1 7 5 9 9 ;
%     7 6 1 1 10 1 10 4 7 8 7 6 1 1 10 1 10 4 7 8];
for i=1:N_p
    %fit(i)=fitness(par(i,:),SAI);
    [F_obj(i),sum_TOC(i),constr(i),bestpa1,fit1]=fitness(par(i,:),SAI);
    fit(i)=fit1;
    F_obj(i)=F_obj(i);
    sum_TOC(i)=sum_TOC(i);
    constr(i)=constr(i);
    %initialize idvididual optimal position
    p_i(i,:)=par(i,:);
end
%initialize swarm optimization position
p_s=par(N_p,:);
for i=1:N_p-1
     [F_obj,sum_TOC,constr,bestpa1,fit1]=fitness(par(i,:),SAI);
     [F_obj,sum_TOC,constr,bestpa2,fit2]=fitness(p_s,SAI);
    %if fitness(par(i,:),SAI)>fitness(p_s,SAI)
    if fit1>fit2
        p_s=par(i,:);
    end
end

% update the particle's position and speed based on the updating formula
IterNr=50;   % iteration number
for t=1:IterNr
    w=0.7298;
    c1=1.4960;
    c2=1.4960;
    %here the range of each coordinate limination should be added,
    %otherwise Altd function will become infeasible.
    for i=1:N_p
        v(i,:)=w.*v(i,:)+c1.*rand.*(p_i(i,:)-par(i,:))+c2.*rand.*(p_s-par(i,:));
        par(i,:)=par(i,:)+v(i,:);
        %A=par(i,1:3:24);
        %B=par(i,2:3:24);
        %C=par(i,3:3:24);
        % coordinate limitation 
        for k=1:8                 % 8 waypoints
            if par(i,-2+3*k)>10
                par(i,-2+3*k)=10;
            end
            if par(i,-1+3*k)>10
                par(i,-1+3*k)=10;
            end
            if par(i,3*k)>1.5
                par(i,3*k)=1.5;
            end
        end
       for j=1:24            %  8X3
           if par(i,j)<0
                par(i,j)=rand;
            end
       end
   
    
    % update the idvididual optimal and swarm optimal position
    [F_obj,sum_TOC,constr,bestpa1,fit1]=fitness(par(i,:),SAI);
    %if fitness(par(i,:),SAI)>fit(i)
    if fit1>fit(i)
       % fit(i)=fitness(par(i,:),SAI);
         fit(i)=fit1;
        p_i(i,:)=par(i,:);   % update invididual ocptimal position
    end 
    [F_obj,sum_TOC,constr,bestpa2,fit2]=fitness(p_s,SAI);
   % if fit(i)>fitness(par(N_p,:),SAI)
   if fit(i)>fit2
        p_s=p_i(i,:); 
        
   end
end

 
   % pbest(t)=fitness(p_s,SAI);
   if t==50;
   [F_obj(t),sum_TOC(t),constr(t), bestpa(t),pbest(t)]=fitness(p_s,SAI);
   ec=cat(1,bestpa.ec);
   fr=cat(1,bestpa.fr);
   sai=cat(1,bestpa.sai);
%    sense(t)=cat(1,region.RS(t));
%    RS=cat(1,sense(t).sen);
%    RSC=cat(1,sense(t).RSC);
%    sense(t)=region.RS(t); 
      load S_Region.mat
      load S_Region(p_s).mat
  else
    [F_obj(t),sum_TOC(t),constr(t), bestpa(t),pbest(t)]=fitness(p_s,SAI);
    ec=cat(1,bestpa.ec);
   fr=cat(1,bestpa.fr);
   sai=cat(1,bestpa.sai);
%    sense(t)=cat(1,region.RS(t));
%    RS=cat(1,sense(t).sen);
%    RSC=cat(1,sense(t).RSC);
%    sense(t)=region.RS(t); 
      load S_Region.mat
      load S_Region(p_s).mat
   end
end
n=length(p_s)/3;
a=reshape(p_s,3,n);
b=a'; % b is final result of path(including n number of waypoints)
% ceil the x and y coordinate of b]
A1=[round(b(:,1)),round(b(:,2)),b(:,3)];
% mapping the x and y into (10X10) grid
% mapping rule: X=2x-1,Y=2x-1 
A1=[round(b(:,1)),round(b(:,2)),b(:,3)];
a1=A1(:,1);
a2=A1(:,2);
for i=1:8
    if a1(i)==0
        a1(i)=1;
    end
    if a2(i)==0
        a2(i)=1;
    end
end
A=[a1,a2,b(:,3)];
bestpath=[2*A(:,1)-1,2*A(:,2)-1,A(:,3)]; 


