
% fitness function
function [F_obj,sum_TOC,constr,bestpa,fit]=fitness(par,SAI)
%par=[1.81085 4.49677 0.68603 0.83449 1.72228 0.07559 0.46793 0.81840 0.94773 3.71658 3.89611 0.78121 0.11211 2.99747 0.50725 3.92192 4.47936 0.63885 4.99073  4.15384 0.94326 3.93896 4.91415 0.26898];
%*****************************************************************
%covert the particle coordinate into interger value
%nomailaizing Energy consumption // fec
%nomailaizing Flight risk // ffr
%nomailaizing SAI value   // fsai
%total objective value // obv
%fitness value // fit
%in this part I should consider the value of above there aspect
% Sensitive Region// SR
% fitness function input
% par: the particle demisionality is equal to 24(3X8)
%*****************************************************************

%initialize particle
bestpa.ec=[];
bestpa.fr=[];  
bestpa.sai=[];
region.RS=[];
sense.sen=[];
S_Region_all=[];

%Constraints
constr.TC=[];
constr.FAC=[];
constr.TAC=[];
constr.FSC=[];
constr.RSC=[];

%sum of total objectives and contraints
sum_TOC.Fobv=[];
sum_TOC.Fcons=[];

%obecjective functions
F_obj.fec=[];
F_obj.ffr=[];
F_obj.fsai=[];


% transfer the par into a matrix(8X3);
n=length(par)/3;
a=reshape(par,3,n);
b=a'; % b is final result of path(including n number of waypoints)
% ceil the x and y coordinate of b]
A1=[round(b(:,1)),round(b(:,2)),b(:,3)];   %等下看看是否需要改成向上取整？？？
a1=A1(:,1);
a2=A1(:,2);
%以下的两个循环是解决就近取整造成有0的问题
for i=1:8
    if a1(i)==0
        a1(i)=1;
    end
    if a2(i)==0
        a2(i)=1;
    end
end
A=[a1,a2,b(:,3)];
   
% mapping the x and y into (10X10) grid
% mapping rule: X=2x-1,Y=2x-1
B=[2*A(:,1)-1,2*A(:,2)-1,A(:,3)]; % B is the final waypoint matrix used to compute fitness value

B(1)=0;
B(2)= 0;
B(8)=18;
%*************************fitness function part **********************
%PART I : Constraints Functions

% Terrain Constraint (TC)
TC=0;
for i=1:n
    t=B(i,:);                      % t(1) is x-dimension, t(2) is y-dimension, t(3) is z-dimension, of a UAV
    if t(3)>Altd(t(1),t(2)) % & t(3)<Altd(t(1),t(2))+0.1
        tc=0;
    else 
        tc=-1;
    end
    TC=TC+tc;
%     TC(i)=TC(i);
end
constr.TC=TC;
% Forbidden Area Constraint(FAC)
FAC=0;
for i=1:n
    t=B(1,:);
    %待会儿看看需不需要进行多个禁区的指定
%     if t(1)==1 & t(2)==9 % specify the area covered by waypoint (1,9) as FA   // forbidden area should change.
     if t(1)==2 & t(2)==15
%         t(1)==2 & t(2)==17
%         t(1)==3 & t(2)==15
%         t(1)==3 & t(2)==17
%         
        fac=-1;
    else
        fac=0;
     end
     
    FAC=FAC+fac;
%     FAC(i)=FAC(i);
end
 constr.FAC=FAC;
% Turning Angle Constraint (TAC)
% using asind() //asind(0.5)=30
% modfiy as acosd // 20180601
TAC=0;
thmax=85;  % from zheng et al. paper
for i=2:n-1
    t1=B(i-1,:);
    t2=B(i,:);
    t3=B(i+1,:);
    q1=[t2(1)-t1(1),t2(2)-t1(2)];
    q2=[t3(1)-t2(1),t3(2)-t2(2)];
    num=dot(q1,q2);     
    % fenzi numerator
    %我居然把分母弄成两个长度相加，真的是无语了
    deno=norm(q1)*norm(q2);       % fenmu denominator
    theta=acosd(num/deno);
    if theta<thmax
        tac=0;
    else
        tac=-1;
    end
    TAC=TAC+tac;
%     TAC(i)=TAC(i);
end
constr.TAC=TAC;
% Flight Slope Constraint(FSC)
FSC=0;
angle=pi/6;  % come from zheng et al.
for i=2:n    %  changed the range already
    t1=B(i-1,:);
    t2=B(i,:);
    q=[t2(1)-t1(1),t2(2)-t1(2)];
    r=abs(t2(3)-t1(3))/(norm(q));  % (20180601)I adopt the abs value,so change to single side
    if r<tan(angle)    % check the range of value r, tan(0.5236)=0.00913879836
        fsc=0;
    else
        fsc=-1;
    end
    FSC=FSC+fsc;
%     FSC(i)=FSC(i);
end
constr.FSC=FSC;
%2018年6月5日，限制条件检查完毕

% PART II : Objective Functions
% Energy Consumption (EC)
%待会儿再来看下关于功耗部分的参数设置问题？？？？
ec=0;
pu=20; % UAV power (watt)
ve=10; % UAV velocity (m/sec)
maxx=20;
maxy=20;
maxz=1.5;
for i=1:n-1
    t1=B(i,:);
    t2=B(i+1,:);
    d=sqrt((t1(1)-t2(1))^2+(t1(2)-t2(2))^2+(t1(3)-t2(3))^2);
    e=pu*d/ve;
    ec=ec+e;
end
maxec=(n-1)*pu*sqrt(20^2+20^2+1.5^2)/ve;  %计算最大功耗的时候采用的了20，20，1.5
fec=ec/maxec;
bestpa.ec=ec;
F_obj.fec=fec;

%Flight Risk (FR)
fr=0;
wer=1/2;
war=1/2;  % equal weight
% Environmental Risk (er)
% what risk value range should I choose? 0-10?
% risk value should be saved or randomly generate each time?

% generate 5X5 environmental risk matrix // generate 10X10 matrix instead
envir=[1.5319    3.9961    0.6013    0.6433    1.7060    4.6479    3.1247    3.5341    1.6586    4.9311    1.1467    1.7294    4.4216    4.4014    2.0933 3.8810    4.2709    3.6075    2.5648    1.5057;
    2.6118    2.6742    0.2683    3.4106    3.0486    3.6987    0.0426    2.7718    4.4558    2.3871    0.8107    2.8615    4.3902    3.5754    1.2072 2.1338    1.4828    3.6605    4.7599    1.8768;
    4.4352    3.4176    2.9004    1.2047    4.8644    4.4050    1.3900    1.9266    1.2073    0.2960    2.2879    4.7067    0.9752    3.9568    4.4362 1.9498    1.3133    4.1339    0.9513    4.6646;
    2.3624    2.2998    4.2845    1.6917    2.7211    3.0106    0.4719    1.6412    3.4188    2.1457    3.4720    3.0420    3.6707    1.0044    1.0845 3.3913    3.6641    4.5103    3.9348    3.3617;
    0.5813    4.0103    0.3954    2.7054    3.1657    2.4765    3.2264    1.3460    2.9038    0.6547    0.4871    3.7386    4.0292    1.0470    1.9799 0.9092    2.6576    3.7128    3.6389    1.5167;
    3.7904    3.1404    0.2260    4.0802    1.1588    0.3555    1.6036    2.1676    1.2238    1.7268    1.4140    0.0269    0.1573    4.5871    1.4213 0.4105    3.6737    4.1611    1.3601    3.8779;
    4.3584    4.4072    3.9704    2.8973    2.5964    1.7538    3.5461    0.5483    0.2559    4.6435    0.8194    0.8225    2.2701    3.8369    1.8146 1.7941    3.7388    4.1159    2.6575    2.2232;
    0.9388    0.9905    3.4099    3.8908    4.1935    4.2906    1.3573    3.7574    1.2513    0.0018    1.9570    1.0584    0.1568    1.2712    4.6326 0.3483    2.3012    2.8381    3.8598    4.3605;
    4.4969    3.1617    4.3227    3.6134    1.6456    1.8140    1.4153    1.8484    1.4299    2.5120    3.7022    0.5870    2.6170    2.9203    2.9629 0.3409    0.3920    4.2568    4.1717    4.5283;
    3.9584    3.3888    4.8582    0.3787    4.3758    0.0915    2.8292    3.0038    3.1557    3.5722    3.1497    2.1270    3.0864    3.1017    2.9158 2.7747    4.6591    4.9918    2.9648    4.5862;
    2.2454    1.2108    0.0061    2.1513    2.0131    4.2273    0.6237    2.5732    0.5034    4.9066    3.3977    1.0360    3.6686    3.0972    1.2883 3.1289    4.3217    4.9132    0.3249    0.6055;
    1.4661    2.0049    0.2876    0.1902    2.3020    3.1786    4.3241    4.7437    2.4318    4.2630    0.9217    4.2297    0.2954    1.8000    0.8832 2.2101    2.4234    2.7784    1.1821    0.7106;
    3.9435    0.8122    4.3206    4.6273    4.1039    2.1710    4.0836    2.3891    3.4980    3.8682    0.0933    4.6242    3.5287    4.0690    4.3890 3.1931    2.2442    3.9757    2.5672    2.5807;
    3.2392    3.7823    1.3740    1.7505    0.2368    0.2303    3.3957    3.3102    3.2250    1.6840    3.0868    1.3014    1.4900    1.1110    0.3259 3.9108    1.3174    4.4851    2.0616    1.4265;
    1.2117    0.3674    0.7218    2.5226    3.6284    1.5489    4.8830    2.6870    0.9419    0.6202    0.1147    2.6025    4.6900    1.4727    4.3263 0.6026    1.8402    2.4071    4.8628    2.5124;
    2.2414    2.7340    0.0291    3.4687    2.1852    4.5548    4.9141    4.7260    3.1386    4.7793    4.8094    0.1153    4.3824    4.5218    4.5437 1.3099    2.0986    4.7199    4.5355    0.2665;
    2.3609    4.1939    2.1074    2.2288    1.6598    3.3586    2.7848    3.0649    1.6057    2.7045    2.5540    2.8360    0.1410    3.1725    4.9787 4.6948    3.8783    3.8312    1.4644    2.3555;
    3.3577    3.9822    3.6696    3.3869    0.3590    3.1599    2.1941    0.7101    0.6998    2.5131    1.5936    4.7966    4.5206    1.0197    0.3186 4.1581    2.4199    4.0243    2.9313    4.6505;
    3.1241    3.6384    3.0152    2.6196    0.2451    3.3256    3.2803    4.0153    1.8061    3.2970    3.5998    2.9764    4.8195    1.6535    2.5813 2.5015    0.6534    4.4666    3.1698    1.1111;
    0.1806    4.5465    0.7350    3.8698    2.7465    0.4808    0.8444    0.1876    4.1907    3.0064    2.3134    4.2227    0.8466    3.2076    2.8627 0.5376    3.1291    2.2819    0.4402    1.2377]; % range:0-5; % range:0-5
maxrisk=5;
%Altitude Risk (ar) : define as the altitude difference between two waypoints
for i=1:n-1
    t1=B(i,:);
    t2=B(i+1,:);
    re=envir(t1(1),t1(2))+envir(t2(1),t2(2)); %environmental risk // t1(1) is originally coordinate.
    %待会考虑是否在高度风险值上面加上权重或设计一个函数来变化：re=3.0648 ra=0.3513 ????
    ra=abs(t2(3)-t1(3)); %altitude risk// consider the weight of the ra???
    a=wer*re+war*ra ;
    fr=fr+a;
end
maxfr=(n-1)*(0.5*maxz+maxrisk); %
ffr=fr/maxfr;
bestpa.fr=fr;
F_obj.ffr=ffr;
% Survelliance Area Importance (SAI)
% this part contains updated mechanism
sai=0;
RSC=0;
maxSAI=10;       % the value not decided yet??
sense_th=10;
%SAI=10*rand(10,10);       %define a SAI matrix (10X10)  // how to updated this SAI matrix????

for i=1:n
    t=B(i,:);
    % for j=t(1):t(1)+1;
% Hu-Teng
%     s=SAI(t(1),t(2))+SAI(t(1),t(2)+1)+SAI(t(1)+1,t(2))+SAI(t(1)+1,t(2)+1);
%     sai=sai+s;

% Umair (Sensitivity)
 if SAI(t(1),t(2))<sense_th;
     rsc=0;
 else
     rsc=-1;
     RS= SAI(t(1),t(2));    % RS: Region Sensitivity
     row=t(1);
     col= t(2);
%       region.RS(r,col)= RS;
    load S_Region.mat
    S_Region_all = [S_Region_all; row col RS]; 
    save('S_Region.mat','S_Region_all')

%       sense1=cat(1,region.RS);
%           sense.sen=cat(1,region.RS);
          
 end
 RSC=RSC+rsc;
 constr.RSC=RSC;
%  sense.RSC=RSC;
 s=SAI(t(1),t(2))+SAI(t(1),t(2)+1)+SAI(t(1)+1,t(2))+SAI(t(1)+1,t(2)+1);
    sai=sai+s;
end
% sense.sen=cat(1,region.RS);
%  else
%       s=SAI(t(1),t(2))+ SAI(t(1),t(2)+1);
%     sai=sai+s;
%  end
% end
maxsai=n*4*maxSAI;
fsai=sai/maxsai;
bestpa.sai=sai;
F_obj.fsai=fsai;

%objective function value 
w1=1;
w2=1;
w3=2;

ht=fec+TC+FAC+TAC+FSC+RSC;

Fobv=-w1*fec-w2*ffr+w3*fsai;
Fcons= TC+FAC+TAC+FSC+RSC;

sum_TOC.Fobv=Fobv;
sum_TOC.Fcons=Fcons;
%fitness function value
fit=Fobv+Fcons;








