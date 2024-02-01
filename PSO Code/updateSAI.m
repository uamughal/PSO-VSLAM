function [SAI]=updateSAI(bestpath1,bestpath2)
% initialize the SAI matrix
SAI=[1 1 1 7 8 1 1 1 10 1 1 1 1 7 8 1 1 1 1 1;
    1 2 1 1 7 7 5 1 1 1 1 2 1 1 7 7 5 1 1 1 ;
    7 8 1 1 5 1 1 1 1 1 7 8 1 1 5 1 1 1 10 1;
    5 6 1 2 2 8 7 7 1 8 5 1 1 2 2 8 7 7 9 8;
    9 1 1 8 4 1 1 8 7 1 9 1 1 8 4 1 1 8 7 1 ;
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
for i=1:8
    r1=bestpath2(i,1);
    r2=bestpath2(i,2);
    for i=r1:r1+1
        for j=r2:r2+1
         SAI(i,j)=0;
        end
    end
    % so far,this part not completed because of neighboring cell not
    % considering
    % finish already
end