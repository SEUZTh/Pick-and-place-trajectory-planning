%% ��׼DH����puma560������ģ��
clear;
clc;
% %����������ģ��(��׼D-H����)
% theta_dh=[-pi/2 0 -pi/2 0 0 0];
% %theta=[0 0 -pi/2 pi/2 pi/2 0];
% %               theta             d           a          alpha       offset
% L1=Link([   theta_dh(1)          0           0         pi/2            0 ],'standard'); %���˵�D-H����
% L2=Link([   theta_dh(2)          0        0.4318       0               0 ],'standard');
% L3=Link([   theta_dh(3)      0.1501	 0.0203     -pi/2            0 ],'standard');
% L4=Link([   theta_dh(4)       0.4318      0          -pi/2            0 ],'standard');
% L5=Link([   theta_dh(5)          0           0            pi/2            0 ],'standard');
% L6=Link([   theta_dh(6)          0           0            0               0 ],'standard');
% robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','puma560','base' , ...
% transl(0, 0, 0)* trotz(0)); %�������ˣ�������ȡ��puma560
% % base	pose of robot's base (4x4 homog xform)
% % figure(1)
% % robot.teach() %teach���ӻ�ģ�Ͳ����Ե������������Բ鿴ģ����ʵ�ʻ����˵Ĺؽ��˶��Ƿ�һ�¡�
% robot.display();  %Link �ຯ������ʾ�����Ļ�����DH����
mdl_puma560
%% ���켣�м��Ĺؽڽ�
% ��ۡ����⳯�¡�����ת(��ת180��)
pos0 = [-0.5,0.5,-0.5]; % ��ʼλ�ã�ȡ�㣩
pos1 = [-0.5,0.5,0.3]; % ת��λ�ã�̧����뿪�㣩
pos2 = [0.5,-0.5,0.3]; % ж��λ�ã��½��㣩
posf = [0.5,-0.5,-0.5]; % Ŀ��λ�ã����õ㣩
pos = [pos0;pos1;pos2;posf];
% ת��Ϊ��ξ���
T0 = transl(pos0);
T1 = transl(pos1);
T2 = transl(pos2);
Tf = transl(posf);

% ���ؽڽ�
theta_0 = p560.ikine6s(T0,'rdf');
theta_1 = p560.ikine6s(T1,'rdf');
theta_2 = p560.ikine6s(T2,'rdf');
theta_f = p560.ikine6s(Tf,'rdf');

disp("��ʼλ�õĹؽڽǣ�");
disp(theta_0)
disp("ת��λ�õĹؽڽǣ�");
disp(theta_1)
disp("ж��λ�õĹؽڽǣ�");
disp(theta_2);
disp("Ŀ��λ�õĹؽڽǣ�");
disp(theta_f);

%% ����ߴζ���ʽϵ��
% 3��ʱ��(��λ����)
t0 = 0; % ̧��ʼʱ��
t1 = 2; % ̧�����ʱ��
t2 = t1 + 4; % ƽ�ƽ���ʱ��
tf = t2 + 3; % �½�����ʱ��
% 6���ؽڵĳ�ʼ����
theta_0_ = [0 0 0 0 0 0]; % ��ʼλ���ٶ�
theta_0__ = [0 0 0 0 0 0]; % ��ʼλ�ü��ٶ�
theta_f_ = [0 0 0 0 0 0]; % Ŀ��λ���ٶ�
theta_f__ = [0 0 0 0 0 0]; % Ŀ��λ�ü��ٶ�
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
disp("6���ؽڸ��Ե��ߴζ���ʽ��ϵ��Ϊ(������Ϊ��1~6���ؽ�)��");
disp(C);

dt = 0.1; % ʱ�䲽��
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
title('�ؽ�λ��');
xlabel('ʱ��t/s');
ylabel('λ��s/rad');
grid on
% plot(t,[0;0],'-r',t,[0;0],'-g',t,[0;0],'-b',t,[0;0],'-c',t,[0;0],'-m',t,[0;0],'-y','MarkerSize',15);
% legend('�ؽ�1','�ؽ�2','�ؽ�3','�ؽ�4','�ؽ�5','�ؽ�6','Location','eastoutside');
subplot(3,2,4);
axis([0 10 -1.5 1.5]);
hold on
title('�ؽ��ٶ�');
xlabel('ʱ��t/s');
ylabel('�ٶ�v/(rad/s)');
grid on
subplot(3,2,6);
axis([0 10 -1.5 1.5]);
hold on
title('�ؽڼ��ٶ�');
xlabel('ʱ��t/s');
ylabel('���ٶ�a/(rad/s^2)');
grid on

[~, count] = size(0:dt:tf);
q3 = zeros(2,6,count);
v3 = zeros(2,6,count);
a3 = zeros(2,6,count);
T_2 = zeros(3,count);
traj3 = zeros(4,4,count);
k=1;
for i = 0:dt:tf
    q = [ones(2,1) t t.^2 t.^3 t.^4 t.^5 t.^6 t.^7] * C; % �ؽ�λ��
    v = [zeros(2,1) ones(2,1) 2*t 3*t.^2 4*t.^3 5*t.^4 6*t.^5 7*t.^6] * C; % �ٶ�
    a = [zeros(2,1) zeros(2,1) 2*ones(2,1) 6*t 12*t.^2 20*t.^3 30*t.^4 42*t.^5] * C; % ���ٶ�
    q3(:,:,k) = q; % ��������
    v3(:,:,k) = v; % ��������
    a3(:,:,k) = a; % ��������
    T_ = p560.fkine(q(2,:)); % ���˶�ѧ���
    T_2(:,k) = T_.t; % ��������
%     traj = transl(T_); % תΪ��ξ���
%     traj3(:,:,k) = traj; % ��������
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
    plot3(T_2(1,k),T_2(2,k),T_2(3,k),'*','LineWidth',1); % ����ά��ͼ�л��Ƴ��˶��켣
    hold on
    p560.plot(q3(2,:,k));%������ʾ
    
    qq = q3(:,:,k)';
    vv = v3(:,:,k)';
    aa = a3(:,:,k)';
    subplot(3,2,2);
    plot(t,qq(1,:),'-r',t,qq(2,:),'-g',t,qq(3,:),'-b',t,qq(4,:),'-c',t,qq(5,:),'-m',t,qq(6,:),'-y','MarkerSize',15);
    legend('�ؽ�1','�ؽ�2','�ؽ�3','�ؽ�4','�ؽ�5','�ؽ�6','Location','eastoutside');
    subplot(3,2,4);
    plot(t,vv(1,:),'-r',t,vv(2,:),'-g',t,vv(3,:),'-b',t,vv(4,:),'-c',t,vv(5,:),'-m',t,vv(6,:),'-y','MarkerSize',15);
    legend('�ؽ�1','�ؽ�2','�ؽ�3','�ؽ�4','�ؽ�5','�ؽ�6','Location','eastoutside');
    subplot(3,2,6);
    plot(t,aa(1,:),'-r',t,aa(2,:),'-g',t,aa(3,:),'-b',t,aa(4,:),'-c',t,aa(5,:),'-m',t,aa(6,:),'-y','MarkerSize',15);
    legend('�ؽ�1','�ؽ�2','�ؽ�3','�ؽ�4','�ؽ�5','�ؽ�6','Location','eastoutside');
    t = t + dt;
    k = k + 1;
    pause(0.01);
end