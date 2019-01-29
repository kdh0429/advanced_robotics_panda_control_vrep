load 'hw3_2.txt'

t=hw3_2(:,1);

x_d_1 = hw3_2(:,2);
y_d_1 = hw3_2(:,3);
z_d_1 = hw3_2(:,4);
x_c_1 = hw3_2(:,5);
y_c_1 = hw3_2(:,6);
z_c_1 = hw3_2(:,7);
x_d_2 = hw3_2(:,8);
y_d_2 = hw3_2(:,9);
z_d_2 = hw3_2(:,10);
x_c_2 = hw3_2(:,11);
y_c_2 = hw3_2(:,12);
z_c_2 = hw3_2(:,13);


subplot(3,1,1)
plot(t,x_d_2,'linewidth',3);
hold on
plot(t,x_c_2,'linewidth',3);
title('x','fontweight','bold','fontsize', 25)
xlabel('시간(초)','fontweight','bold','fontsize', 20)
ylabel('위치(m)','fontweight','bold','fontsize', 20)
legend('desired','current')
grid on

subplot(3,1,2)
plot(t,y_d_2,'linewidth',3);
hold on
plot(t,y_c_2,'linewidth',3);
title('y','fontweight','bold','fontsize', 25)
xlabel('시간(초)','fontweight','bold','fontsize', 20)
ylabel('위치(m)','fontweight','bold','fontsize', 20)
legend('desired','current')
grid on

subplot(3,1,3)
plot(t,z_d_2,'linewidth',3);
hold on
plot(t,z_c_2,'linewidth',3);
title('z','fontweight','bold','fontsize', 25)
xlabel('시간(초)','fontweight','bold','fontsize', 20)
ylabel('위치(m)','fontweight','bold','fontsize', 20)
legend('desired','current')
grid on