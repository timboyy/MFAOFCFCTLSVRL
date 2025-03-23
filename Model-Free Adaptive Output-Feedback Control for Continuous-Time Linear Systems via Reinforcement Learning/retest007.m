% <Model-Free Adaptive Output-Feedback Control for Continuous-Time Linear
% Systems via Reinforcement Learning> model2
%run this file and open statecontrol2,the file version is 2022b, and Simulink before this version cannot be opened

%针对retest001文件的细节调整，拟解决论文中对于观测变量进行控制的仿真模拟实验失效的问题
%主要是对于观测变量的模拟，采样周期为0.001s，之前对于延迟采样设计的是按采样周期延迟采样
%现在按采样周期中添加3个模拟点进行模拟延迟采样，如0.00025s,0.0005s,0.00075s
%通过0s和0.0001s的采样数据对中间模拟点数据进行估计，这样就有4维的，不失真的观测数据
%更新版本，添加了快速重启结构
%针对retest002文件的调整，由于002不能做到在simulink中模拟仿真，因此要进行细节上的修改


clear 
clc

%要解决的是x'=A*x+B*u的控制律问题即最优调节问题，找到使得∫x*M*x+u*R*u dt的值最小的控制u
%对于常见状态空间方程输出y=C*x，可以考虑最小化y的二范数，即取M=C'*C，R≈0
%给定初始参数和超参数
A=[0	1	0	0
-218756.663497404	-75.3943158453190	0	0
0	0	0	1
0	0	-425015.189785222	-121.227999776810];
Bru=[1.30358515621439e-08	1.30358515745248e-08
0.000161986493696331	0.000161986493698015
-2.26152627580204e-08	2.26152627898945e-08
-0.000238132799911334	0.000238132799917303];
Brf=[-0.00130142918674408
-8.05097234388870
0.00174521761973298
9.47255980317982];
C=[16.0376601050147	0.425434196872911	-35.4185444812321	-0.584329437860915];
M=10^8;
R=10^-8;
% 给出初始参数，x'=A*x+Bru*u+Brf*δ，其中δ是输入干扰信号，M,R表示二次调节中的参数

% aaa=5
% A=A+aaa*A.*rand(size(A))
% Bru=Bru+aaa*Bru.*rand(size(Bru))
% Brf=Brf+aaa*Brf.*rand(size(Brf))
% 此为原状态空间方程考虑误差的控制，若随机生成的矩阵不满足可控条件会导致控制失效

T=0.001;    
%连续控制离散化的模拟周期
[m1,m2]=size([A,Brf,Bru]);
m0=zeros(max(m1,m2));
m0(1:m1,1:m2)=[A,Bru,Brf];
m0=expm(m0*T);
[a1,a2]=size(A);
[b11,b12]=size(Bru);
[b21,b22]=size(Brf);
A0=m0(1:a1,1:a2);
Bu0=m0(1:a1,a2+1:a2+b12);
Bf0=m0(1:a1,a2+b12+1:m2);
B=Bu0;
%将状态空间方程参数离散化处理，离散时间为T，原理见https://blog.51cto.com/u_15714963/6062070
row0=size(Bru);
u_0=row0(2);
row1=size(A);
x_0=row1(2);
%这两个变量分别用来存储状态变量X和控制输入U的维度,根据输入矩阵自动调整
a_0=[x_0,u_0];
c_0=(x_0+u_0)*(x_0+u_0+1)/2;
%这两个变量分别用来存储act和critic网络权重参数的维度

aerfaa=2;
aerfac=50;
%这两个变量用来存储增益率
zzz=zeros(1,10);

for i=1:10
    while 1
        wa_0=zeros(a_0);%10*rand(a_0);
        wa_00=cell(1);
        wc_0=10*rand(a_0);
        wg_0=R^(-1/2)*rand([2,1]);
        %这两个变量用来存储act和critic网络权重参数的初始值，采用随机变量
        x_01=zeros(x_0,1);
        u_01=zeros(u_0,1);
        y_01=zeros(x_0,1);
        %存储第j个周期的q函数值
        j=1;
        j0=0;
        while 1
            u_01(:,j+1)=wa_0'*y_01(:,j);
            x_01(:,j+1)=A0*x_01(:,j)+B*u_01(:,j+1)+Bf0*(0.5*sin(2*pi*76*j*T)+0.5*sin(2*pi*106*j*T));
            y_01(1,j+1)=C*x_01(:,j+1);
            y_01(2,j+1)=3/4*C*x_01(:,j+1)+1/4*C*x_01(:,j);
            y_01(3,j+1)=2/4*C*x_01(:,j+1)+2/4*C*x_01(:,j);
            y_01(4,j+1)=1/4*C*x_01(:,j+1)+3/4*C*x_01(:,j);
            ea_01=R^-1*wc_0'*y_01(:,j+1);
            wa_detla=-aerfaa*(y_01(:,j+1)*(ea_01-0)');
            wa_0=wa_0+wa_detla;
            wa_00{j}=wa_0;
            if and(mean(abs(y_01(:,j+1)))>=0.1,j-j0>=2)
                
                wa_0=zeros(a_0);%10*rand(a_0);
                wc_0=10*rand(a_0);
                wg_0=R^(-1/2)*rand([2,1]);
                %这两个变量用来存储act和critic网络权重参数的初始值，采用随机变量
                while 1 
                    j=j+1;
                    u_01(:,j+1)=wa_0'*y_01(:,j);
                    x_01(:,j+1)=A0*x_01(:,j)+B*u_01(:,j+1);
                    y_01(1,j+1)=C*x_01(:,j+1);
                    y_01(2,j+1)=3/4*C*x_01(:,j+1)+1/4*C*x_01(:,j);
                    y_01(3,j+1)=2/4*C*x_01(:,j+1)+2/4*C*x_01(:,j);
                    y_01(4,j+1)=1/4*C*x_01(:,j+1)+3/4*C*x_01(:,j);
                    wa_00{j}=wa_0;
                    if mean(abs(y_01(:,j+1)))<=0.001
                        break
                    end
                end
                j0=j-1;
            else 
                j=j+1;
            end
            if j-j0>=50
                break
                    
            end
            
        end
        if j0>0
            break
        end
    end




    qqq=abs(C*x_01);
    zzz(i)=mean(abs(qqq(:)));
    if zzz(i)==min(zzz(~ismember(zzz,0)))
        U0=wa_0;
    end
    %进行10次模拟，得出控制效果最好的K值，并保存在U0中，在simulink模拟中采用的是K=U0'（转置）
end


X1=1:length(y_01);
Y1=y_01(1,:);
%绘制论文figure 6
% 创建 figure
figure1 = figure;

% 创建 axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% 创建 plot
plot(X1,Y1,'DisplayName','','Marker','x','LineWidth',0.3,...
    'Color',[0.168627451 0.380392157 0.976470588]);

% 创建 ylabel
ylabel('Y');

% 创建 xlabel
xlabel('Number of iterations');

box(axes1,'on');
grid(axes1,'on');
hold(axes1,'off');
% 设置其余坐标区属性
set(axes1,'FontSize',12,'LineWidth',1);
% 创建 arrow
annotation(figure1,'arrow',[0.3375 0.158928571428571],...
    [0.306142857142857 0.535714285714286]);

% 创建 arrow
annotation(figure1,'arrow',[0.341071428571429 0.225],...
    [0.301380952380952 0.892857142857143]);

% 创建 arrow
annotation(figure1,'arrow',[0.341071428571429 0.375],...
    [0.30852380952381 0.585714285714286]);

% 创建 arrow
annotation(figure1,'arrow',[0.341071428571429 0.557142857142857],...
    [0.306142857142857 0.504761904761905]);

% 创建 textbox
annotation(figure1,'textbox',...
    [0.261714285714285 0.230952380952381 0.257928571428571 0.0714285714285721],...
    'String',{'Control policy restart'},...
    'FitBoxToText','off');
%figure6

for i=1:length(wa_00)
    kk(i)=norm(wa_00{i}-wa_00{end});
end
figure,plot(1:length(kk),log(kk+1));
annotation('textbox',[.4 .07 0.3 0.0], ...
    'String','Number of iterations','EdgeColor','none')
legend('lg（||k_{k}-k^※||+1）')
text(80,-5,'Number of iterations ', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');

kk=y_01(1,:);
X1=1:length(kk);
YMatrix1=kk;

figure1 = figure;

% 创建 axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% 使用 plot 的矩阵输入创建多个 line 对象
plot1 = plot(X1,YMatrix1,'LineWidth',0.3);
set(plot1(1),'DisplayName',' ','Marker','x ','Color',[0.168627451 0.380392157 0.976470588]);

% 创建 ylabel
ylabel('Y');

% 创建 xlabel
xlabel('Number of iterations');



box(axes1,'on');
grid(axes1,'on');
hold(axes1,'off');
% 设置其余坐标区属性
set(axes1,'FontSize',12,'LineWidth',1);



%绘制论文figure5


% T=0.001;    
% %连续控制离散化的模拟周期
% [m1,m2]=size([A,Brf,Bru]);
% m0=zeros(max(m1,m2));
% m0(1:m1,1:m2)=[A,Bru,Brf];
% m0=expm(m0*T);
% [a1,a2]=size(A);
% [b11,b12]=size(Bru);
% [b21,b22]=size(Brf);
% A0=m0(1:a1,1:a2);
% Bu0=m0(1:a1,a2+1:a2+b12);
% Bf0=m0(1:a1,a2+b12+1:m2);
% B=Bu0;
% clear x_01 y_01 u_01
% x_01=zeros(x_0,1/T);
% u_01=zeros(u_0,1/T);
% y_01=zeros(x_0,1/T+4);
% for j=1:1/T
%     u_01(:,j+1)=U0'*y_01(:,j); 
%     x_01(:,j+1)=A0*x_01(:,j)+B*u_01(:,j+1)+Bf0*(0.5*sin(2*pi*76*j*T)+0.5*sin(2*pi*106*j*T));
%     y_01(1,j+1)=C*x_01(:,j+1);
%     y_01(2,j+1)=3/4*C*x_01(:,j+1)+1/4*C*x_01(:,j);
%     y_01(3,j+1)=2/4*C*x_01(:,j+1)+2/4*C*x_01(:,j);
%     y_01(4,j+1)=1/4*C*x_01(:,j+1)+3/4*C*x_01(:,j);
% end
% plot(0:T:T*(length(y_01)-1),y_01(1,:))
