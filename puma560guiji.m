%% 标准DH建立puma560机器人模型
clear;
clc;
%建立机器人模型(标准D-H参数)
theta_dh=[-pi/2 0 -pi/2 0 0 0];
%theta=[0 0 -pi/2 pi/2 pi/2 0];
%               theta             d           a          alpha       offset
L1=Link([   theta_dh(1)          0           0         pi/2            0 ],'standard'); %连杆的D-H参数
L2=Link([   theta_dh(2)          0        0.4318       0               0 ],'standard');
L3=Link([   theta_dh(3)      0.1501	 0.0203     -pi/2            0 ],'standard');
L4=Link([   theta_dh(4)       0.4318      0          -pi/2            0 ],'standard');
L5=Link([   theta_dh(5)          0           0            pi/2            0 ],'standard');
L6=Link([   theta_dh(6)          0           0            0               0 ],'standard');
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','puma560','base' , ...
transl(0, 0, 0)* trotz(0)); %连接连杆，机器人取名puma560
% base	pose of robot's base (4x4 homog xform)
figure(1)
robot.teach() %teach可视化模型并可以单轴驱动，可以查看模型与实际机器人的关节运动是否一致。
robot.display();  %Link 类函数，显示建立的机器人DH参数
%% 求解轨迹中间点的关节角
% 左臂、手肘朝下、手腕翻转(旋转180度)
pos0 = [0.5,0.5,-0.5]; % 起始位置（取点）
pos1 = [0.5,0.5,0.3]; % 转移位置（抬起或离开点）
pos2 = [-0.5,-0.5,0.3]; % 卸下位置（下降点）
posf = [-0.5,-0.5,-0.5]; % 目标位置（放置点）

% 转换为齐次矩阵
T0 = transl(pos0);
T1 = transl(pos1);
T2 = transl(pos2);
Tf = transl(posf);

% 求解关节角
theta_0 = robot.ikine6s(T0,'rdf');
theta_1 = robot.ikine6s(T1,'rdf');
theta_2 = robot.ikine6s(T2,'rdf');
theta_f = robot.ikine6s(Tf,'rdf');

% theta_0 = robot.ikine6s(pos0,'rdf');
% theta_1 = robot.ikine6s(pos1,'rdf');
% theta_2 = robot.ikine6s(pos2,'rdf');
% theta_f = robot.ikine6s(posf,'rdf');

disp("起始位置的关节角：");
disp(theta_0)
disp("转移位置的关节角：");
disp(theta_1)
disp("卸下位置的关节角：");
disp(theta_2);
disp("目标位置的关节角：");
disp(theta_f);

%% 求解七次多项式系数
% 3段时间(单位：秒)
t0 = 0; % 抬起开始时刻
t1 = 2; % 抬起结束时刻
t2 = t1 + 4; % 平移结束时刻
tf = t2 + 3; % 下降结束时刻
% 6个关节的初始条件
theta_0_ = [0 0 0 0 0 0]; % 初始位置速度
theta_0__ = [0 0 0 0 0 0]; % 初始位置加速度
theta_f_ = [0 0 0 0 0 0]; % 目标位置速度
theta_f__ = [0 0 0 0 0 0]; % 目标位置加速度
THETA = [theta_0;theta_0_;theta_0__;theta_1;theta_2;theta_f;theta_f_;theta_f__];
M =...
[1     0    0      0       0        0        0        0
 0     1    0      0       0        0        0        0
 0     0    2      0       0        0        0        0
 1     t1   t1^2   t1^3    t1^4     t1^5     t1^6     t1^7
 1     t2   t2^2   t2^3    t2^4     t2^5     t2^6     t2^7
 1     tf   tf^2   tf^3    tf^4     tf^5     tf^6     tf^7
 0     1    2*tf   3*tf^2  4*tf^3   5*tf^4   6*tf^5   7*tf^6
 0     0    2      6*tf    12*tf^2  20*tf^3  30*tf^4  42*tf^5];
C = M^-1 * THETA;
disp("6个关节各自的七次多项式的系数为(从左到右为第1~6个关节)：");
disp(C);

dt = 0.1; % 时间步长
t = [-dt;0];
figure(2)
set(gcf,'outerposition',get(0,'screensize'),'color','w');
subplot(3,1,1);
axis([0 10 -4 4]);
hold on
title('关节位移');
xlabel('时间t/s');
ylabel('位移s/rad');
grid on
% plot(t,[0;0],'-r',t,[0;0],'-g',t,[0;0],'-b',t,[0;0],'-c',t,[0;0],'-m',t,[0;0],'-y','MarkerSize',15);
% legend('关节1','关节2','关节3','关节4','关节5','关节6','Location','eastoutside');
subplot(3,1,2);
axis([0 10 -1.5 1.5]);
hold on
title('关节速度');
xlabel('时间t/s');
ylabel('速度v/(rad/s)');
grid on
subplot(3,1,3);
axis([0 10 -1.5 1.5]);
hold on
title('关节加速度');
xlabel('时间t/s');
ylabel('加速度a/(rad/s^2)');
grid on

for i = 0:dt:tf
    q = [ones(2,1) t t.^2 t.^3 t.^4 t.^5 t.^6 t.^7] * C; % 关节位移
    v = [zeros(2,1) ones(2,1) 2*t 3*t.^2 4*t.^3 5*t.^4 6*t.^5 7*t.^6] * C; % 速度
    a = [zeros(2,1) zeros(2,1) 2*ones(2,1) 6*t 12*t.^2 20*t.^3 30*t.^4 42*t.^5] * C; % 加速度
    T_ = robot.fkine(q(2,:)); % 正运动学求解
    traj = transl(T_); % 转为齐次矩阵
%     figure(1)
%     plot3(T_.t(1),T_.t(2),T_.t(3),'*','LineWidth',1); % 在三维视图中绘制出运动轨迹
%     robot.plot(q);%动画演示
%     hold on
    
    qq = q';
    vv = v';
    aa = a';
    figure(2)
    hold on
    subplot(3,1,1);
    plot(t,qq(1,:),'-r',t,qq(2,:),'-g',t,qq(3,:),'-b',t,qq(4,:),'-c',t,qq(5,:),'-m',t,qq(6,:),'-y','MarkerSize',15);
    %legend('关节1','关节2','关节3','关节4','关节5','关节6','Location','eastoutside');
    subplot(3,1,2);
    plot(t,vv(1,:),'-r',t,vv(2,:),'-g',t,vv(3,:),'-b',t,vv(4,:),'-c',t,vv(5,:),'-m',t,vv(6,:),'-y','MarkerSize',15);
    %legend('关节1','关节2','关节3','关节4','关节5','关节6','Location','eastoutside');
    subplot(3,1,3);
    plot(t,aa(1,:),'-r',t,aa(2,:),'-g',t,aa(3,:),'-b',t,aa(4,:),'-c',t,aa(5,:),'-m',t,aa(6,:),'-y','MarkerSize',15);
    %legend('关节1','关节2','关节3','关节4','关节5','关节6','Location','eastoutside');
    
%     str=[ '\leftarrow' '(' num2str(t1) ',' num2str(theta1(1)) ')'];
%     text(t1,theta1(1),cellstr(str));
%     str=[ '\leftarrow' '(' num2str(t2) ',' num2str(theta2(1)) ')'];
%     text(t2,theta2(1),cellstr(str));
    
%     grid on;
    
%     subplot(3,1,2);
%     plot(tt,v);
% %     grid on;
%     
%     subplot(3,1,3);
%     plot(tt,a);
%     grid on;
    t = t + dt;
%     pause(0.01);
end
