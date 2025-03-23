%6维算例仿真
% <Model-Free Adaptive Output-Feedback Control for Continuous-Time Linear
% Systems via Reinforcement Learning> model1 
%run this file and open statecontrol1,the file version is 2022b, and Simulink before this version cannot be opened
clear 
clc

%要解决的是x'=A*x+B*u的控制律问题即最优调节问题，找到使得∫x*M*x+u*R*u dt的值最小的控制u
%对于常见状态空间方程输出y=C*x，可以考虑最小化y的二范数，即取M=C'*C，R≈0
%给定初始参数和超参数
A=[-5 2 3 0 0 0
2 -6 0 0 1 3
3 0 -5 2 0 0
0 0 2 -2 0 0
0 1 0 0 -4 3
0 3 0 0 3 -6];
Bru=eye(6);
R=1;
M=30*eye(6);
C=[1,0,0,0,0,0];



% 给出初始参数，x'=A*x+Bru*u+Brf*δ，其中δ是输入干扰信号，M,R表示二次调节中的参数
T=0.001;  
[A0,B0]=discreet(A,Bru,T);
row0=size(Bru);
u_0=row0(2);
row1=size(A);
x_0=row1(2);
a_0=[x_0,u_0];

aerfaa=2;
setat=10;
zzz=zeros(1,10);

for i=1:10
    wa_0=zeros(a_0);%10*rand(a_0);
    wa_00={wa_0};
    wc_0=10*rand(a_0);
    %这两个变量用来存储act和critic网络权重参数的初始值，采用随机变量
    x_01=zeros(x_0,1);
    u_01=zeros(u_0,1);
    y_01=zeros(x_0,1);
    clear w
    %存储第j个周期的q函数值
    for j =1:100
        w(:,j)=setat*sum(sin((rand([100,u_0])*100-50)*j*T));
    end
    j=1;
    while 1
        u_01(:,j+1)=wa_0'*y_01(:,j);
        x_01(:,j+1)=A0*x_01(:,j)+B0*u_01(:,j+1)+B0*w(:,j);
        for k=1:a_0
            y_01(k,j+1)=(x_0-k+1)/x_0*C*x_01(:,j+1)+(k-1)/x_0*C*x_01(:,j);
        end
        ea_01=R^-1*wc_0'*y_01(:,j+1);
        wa_detla=-aerfaa*(y_01(:,j+1)*ea_01');
        wa_0=wa_0+wa_detla;
        wa_00{end+1}=wa_0;
        if and(sum(abs(y_01(:,j+1)))>=sqrt(setat),and(sum(abs(y_01(:,j+1)))>=5*sum(abs(y_01(:,j))),j>=2))
            wa_0=zeros(a_0)%10*rand(a_0);
            wa_00{end+1}=wa_0;
            wc_0=10*rand(a_0);
            %这两个变量用来存储act和critic网络权重参数的初始值，采用随机变量
            x_01=zeros(x_0,1);
            u_01=zeros(u_0,1);
            y_01=zeros(x_0,1);
            j=1;
        else 
            j=j+1;
        end
        if j>=100
            break
        end
    end

    qqq=abs(C*x_01);
    zzz(i)=sum(qqq(:));
    if zzz(i)==min(zzz(~ismember(zzz,0)))
        U0=wa_0;
    end
    %进行10次模拟，得出控制效果最好的K值，并保存在U0中，在simulink模拟中采用的是K=U0'（转置）
end


for i=1:length(wa_00)
    kk(i)=norm(wa_00{i}-wa_00{end});
end
plot(1:length(kk),log(kk+1));
annotation('textbox',[.4 .07 0.3 0.0], ...
    'String','Number of iterations','EdgeColor','none')
legend('lg（||k_{k}-k^※||+1）')

text(80,-5,'Number of iterations ', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');


%绘制论文中figure2
X1=1:length(kk);
YMatrix1=log(kk+1);

figure1 = figure;

% 创建 axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% 使用 plot 的矩阵输入创建多个 line 对象
plot1 = plot(X1,YMatrix1,'LineWidth',0.3);
set(plot1(1),'DisplayName','lg（||k_{k}-k^※||+1）','Marker','x ','Color',[0.168627451 0.380392157 0.976470588]);

% 创建 ylabel
ylabel('Y');

% 创建 xlabel
xlabel('times(t)');



box(axes1,'on');
grid(axes1,'on');
hold(axes1,'off');
% 设置其余坐标区属性
set(axes1,'FontSize',12,'LineWidth',1);
% 创建 legend
legend(axes1,'location','southwest');



