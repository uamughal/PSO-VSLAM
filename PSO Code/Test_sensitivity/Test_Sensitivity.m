
% randperm (20,20)
% for i=20;
%     if s==5;
%         rectangle('Position', [RSi],'edgecolor','b','FaceColor',[1 0 0], 'Linewidth', 2);
%     end
% % end
% [X,Y,Z] = peaks; 
% surf(X,Y,Z)
% grid on
% hold on
% x = linspace(0,10);
% y = sin(x);
% plot(x,y)
% grid on
tic
clear all
close all
clc
np=25;
a=1;
b=10;
r=20;
c=20;
sense_th=9.7;
% sense: sensitivity

% sense=10*rand(20,20); 
sense= a+(b-a).*rand(r,c) % generate random no in the interval (a,b) having row, column matrix of (r,c) 
 
% sense=randi([1 10],20,20); % generate random ingeters from 1-10 [1 10] having 20 by 20 matrix 

  j=sense';
for i=1:length(sense)
  for k=1:length(j);
%  for i=2:length(sense)-1; 
%   for k=2:length(j)-1;
%     s=Sense(i)+ Sense(j);
    
%     if Sense(i),j(k)>15
     if sense(i,k)>sense_th
        SR=sense(i,k);
        region.SR(i,k)=SR;
        sr=cat(1,region.SR);
      else
%           sense(i,k)=0;
    end
  end
end
% plot(sr);
% plot (x(i),y(j))
% rectangle('Position', [i j 1 1],'edgecolor','b','FaceColor',[1 0 0], 'Linewidth', 2);

%     sr=sr*100;
%   [row col value]=find(sr>0,6)
    [row col value]=find(sr>0)
    
    for cntr = 1:length(value)
        h1=max(row)
        h2=max(col)
    if row(cntr) <h1 & col(cntr)<h2 % do not make rectangle on the indexes of 20, 20 because
                                    % it goes out the 20x20 grid due to rentangle lenght of [1 1] 
    if row(cntr)~= 2 && col(cntr)~= 2
    if row(cntr)~= 18 && col(cntr)~= 19
        
    rectangle('Position', [row(cntr) col(cntr) 1 1],'edgecolor','b','FaceColor',[1 0 0], 'Linewidth', 2);
    grid on
    end
    end
    end
    end
   toc 
    
% spath.Xs=[];
% spath.ys=[];
% a=20;
% b=20;
% x=randperm (a,10);
% y=randperm (b,10);
%

   