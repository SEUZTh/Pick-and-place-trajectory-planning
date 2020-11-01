%% 标准DH建立puma560机器人模型
clear;
clc;
% %建立机器人模型(标准D-H参数)
% theta_dh=[-pi/2 0 -pi/2 0 0 0];
% %theta=[0 0 -pi/2 pi/2 pi/2 0];
% %               theta             d           a          alpha       offset
% L1=Link([   theta_dh(1)          0           0         pi/2            0 ],'standard'); %连杆的D-H参数
% L2=Link([   theta_dh(2)          0        0.4318       0               0 ],'standard');
% L3=Link([   theta_dh(3)      0.1501	 0.0203     -pi/2            0 ],'standard');
% L4=Link([   theta_dh(4)       0.4318      0          -pi/2            0 ],'standard');
% L5=Link([   theta_dh(5)          0           0            pi/2            0 ],'standard');
% L6=Link([   theta_dh(6)          0           0            0               0 ],'standard');
% robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','puma560','base' , ...
% transl(0, 0, 0)* trotz(0)); %连接连杆，机器人取名puma560
% % base	pose of robot's base (4x4 homog xform)
% % figure(1)
% % robot.teach() %teach可视化模型并可以单轴驱动，可以查看模型与实际机器人的关节运动是否一致。
% robot.display();  %Link 类函数，显示建立的机器人DH参数
mdl_puma560
%% 求解轨迹中间点的关节角
% 左臂、手肘朝下、手腕翻转(旋转180度)
pos0 = [-0.5,0.5,-0.5]; % 起始位置（取点）
pos1 = [-0.5,0.5,0.3]; % 转移位置（抬起或离开点）
pos2 = [0.5,-0.5,0.3]; % 卸下位置（下降点）
posf = [0.5,-0.5,-0.5]; % 目标位置（放置点）
pos = [pos0;pos1;pos2;posf];
% 转换为齐次矩阵
T0 = transl(pos0);
T1 = transl(pos1);
T2 = transl(pos2);
Tf = transl(posf);

% 求解关节角
theta_0 = p560.ikine6s(T0,'rdf');
theta_1 = p560.ikine6s(T1,'rdf');
theta_2 = p560.ikine6s(T2,'rdf');
theta_f = p560.ikine6s(Tf,'rdf');

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
subplot(3,2,[1,3,5]);
axis([-1 1 -1 1 -1 1]);
hold on
plot3(pos(:,1),pos(:,2),pos(:,3));
plot_sphere(pos', 0.05, 'y');
subplot(3,2,2);
axis([0 10 -4 4]);
hold on
title('关节位移');
xlabel('时间t/s');
ylabel('位移s/rad');
grid on
% plot(t,[0;0],'-r',t,[0;0],'-g',t,[0;0],'-b',t,[0;0],'-c',t,[0;0],'-m',t,[0;0],'-y','MarkerSize',15);
% legend('关节1','关节2','关节3','关节4','关节5','关节6','Location','eastoutside');
subplot(3,2,4);
axis([0 10 -1.5 1.5]);
hold on
title('关节速度');
xlabel('时间t/s');
ylabel('速度v/(rad/s)');
grid on
subplot(3,2,6);
axis([0 10 -1.5 1.5]);
hold on
title('关节加速度');
xlabel('时间t/s');
ylabel('加速度a/(rad/s^2)');
grid on

[~, count] = size(0:dt:tf);
q3 = zeros(2,6,count);
v3 = zeros(2,6,count);
a3 = zeros(2,6,count);
T_2 = zeros(3,count);
traj3 = zeros(4,4,count);
k=1;
for i = 0:dt:tf
    q = [ones(2,1) t t.^2 t.^3 t.^4 t.^5 t.^6 t.^7] * C; % 关节位移
    v = [zeros(2,1) ones(2,1) 2*t 3*t.^2 4*t.^3 5*t.^4 6*t.^5 7*t.^6] * C; % 速度
    a = [zeros(2,1) zeros(2,1) 2*ones(2,1) 6*t 12*t.^2 20*t.^3 30*t.^4 42*t.^5] * C; % 加速度
    q3(:,:,k) = q; % 保存数据
    v3(:,:,k) = v; % 保存数据
    a3(:,:,k) = a; % 保存数据
    T_ = p560.fkine(q(2,:)); % 正运动学求解
    T_2(:,k) = T_.t; % 保存数据
%     traj = transl(T_); % 转为齐次矩阵
%     traj3(:,:,k) = traj; % 保存数据
    k=k+1;
    
%     p560.plot3d(q(2,:), 'view', [138 8]);
%     plot_sphere(T_.t, 0.05, 'y');
       t = t + dt;
%     pause(1);
end

k=1;
t = [-dt;0];
for i = 0:dt:tf
    subplot(3,2,[1,3,5])
    plot3(T_2(1,k),T_2(2,k),T_2(3,k),'*','LineWidth',1); % 在三维视图中绘制出运动轨迹
    hold on
    p560.plot(q3(2,:,k));%动画演示
    
    qq = q3(:,:,k)';
    vv = v3(:,:,k)';
    aa = a3(:,:,k)';
    subplot(3,2,2);
    plot(t,qq(1,:),'-r',t,qq(2,:),'-g',t,qq(3,:),'-b',t,qq(4,:),'-c',t,qq(5,:),'-m',t,qq(6,:),'-y','MarkerSize',15);
    legend('关节1','关节2','关节3','关节4','关节5','关节6','Location','eastoutside');
    subplot(3,2,4);
    plot(t,vv(1,:),'-r',t,vv(2,:),'-g',t,vv(3,:),'-b',t,vv(4,:),'-c',t,vv(5,:),'-m',t,vv(6,:),'-y','MarkerSize',15);
    legend('关节1','关节2','关节3','关节4','关节5','关节6','Location','eastoutside');
    subplot(3,2,6);
    plot(t,aa(1,:),'-r',t,aa(2,:),'-g',t,aa(3,:),'-b',t,aa(4,:),'-c',t,aa(5,:),'-m',t,aa(6,:),'-y','MarkerSize',15);
    legend('关节1','关节2','关节3','关节4','关节5','关节6','Location','eastoutside');
    t = t + dt;
    k = k + 1;
    pause(0.01);
end