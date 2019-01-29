load 'hw3_3.txt'

t=hw3_3(:,1);

x_d_1 = hw3_3(:,2);
y_d_1 = hw3_3(:,3);
z_d_1 = hw3_3(:,4);
x_c_1 = hw3_3(:,5);
y_c_1 = hw3_3(:,6);
z_c_1 = hw3_3(:,7);
x_d_2 = hw3_3(:,8);
y_d_2 = hw3_3(:,9);
z_d_2 = hw3_3(:,10);
x_c_2 = hw3_3(:,11);
y_c_2 = hw3_3(:,12);
z_c_2 = hw3_3(:,13);
h1 = hw3_3(:,14);
h2 = hw3_3(:,15);
    
wall = 0.58*ones(size(t,1),1);





subplot(5,1,1);
plot(t,wall,'linewidth',3);
hold on
plot(t,x_c_1,'linewidth',3);
title('x end-effector','fontweight','bold','fontsize', 25)
ylabel('위치(m)','fontweight','bold','fontsize', 20)
legend('wall','current')
grid on


subplot(5,1,2);
plot(t,h1,'linewidth',3);
title('h1','fontweight','bold','fontsize', 25)
grid on


subplot(5,1,3);
plot(t,x_d_2,'linewidth',3);
hold on
plot(t,x_c_2,'linewidth',3);
title('x CoM4','fontweight','bold','fontsize', 25)
ylabel('위치(m)','fontweight','bold','fontsize', 20)
legend('desried','current')
grid on

subplot(5,1,4);
plot(t,y_d_2,'linewidth',3);
hold on
plot(t,y_c_2,'linewidth',3);
title('y CoM4','fontweight','bold','fontsize', 25)
ylabel('위치(m)','fontweight','bold','fontsize', 20)
legend('desried','current')
grid on

subplot(5,1,5);
plot(t,z_d_2,'linewidth',3);
hold on
plot(t,z_c_2,'linewidth',3);
title('z CoM4','fontweight','bold','fontsize', 25)
xlabel('시간(초)','fontweight','bold','fontsize', 20)
ylabel('위치(m)','fontweight','bold','fontsize', 20)
legend('desried','current')
grid on