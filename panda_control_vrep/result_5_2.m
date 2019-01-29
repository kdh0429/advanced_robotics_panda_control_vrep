load 'hw5_2.txt'

t=hw5_2(:,1);

x_d = hw5_2(:,2);
y_d = hw5_2(:,3);
z_d = hw5_2(:,4);
x_c = hw5_2(:,5);
y_c = hw5_2(:,6);
z_c = hw5_2(:,7);

xdot_d = hw5_2(:,8);
ydot_d = hw5_2(:,9);
zdot_d = hw5_2(:,10);
xdot_c = hw5_2(:,11);
ydot_c = hw5_2(:,12);
zdot_c = hw5_2(:,13);

q1 = hw5_2(:,14);
q2 = hw5_2(:,15);
q3 = hw5_2(:,16);
q4 = hw5_2(:,17);
q5 = hw5_2(:,18);
q6 = hw5_2(:,19);
q7 = hw5_2(:,20);



subplot(3,1,1)
plot(t,x_d,'linewidth',3);
hold on
plot(t,x_c,'linewidth',3);
title('x','fontweight','bold','fontsize', 25)
xlabel('시간(초)','fontweight','bold','fontsize', 20)
ylabel('위치(m)','fontweight','bold','fontsize', 20)
legend('desired','current')
grid on

subplot(3,1,2)
plot(t,y_d,'linewidth',3);
hold on
plot(t,y_c,'linewidth',3);
title('y','fontweight','bold','fontsize', 25)
xlabel('시간(초)','fontweight','bold','fontsize', 20)
ylabel('위치(m)','fontweight','bold','fontsize', 20)
legend('desired','current')
grid on

subplot(3,1,3)
plot(t,z_d,'linewidth',3);
hold on
plot(t,z_c,'linewidth',3);
title('z','fontweight','bold','fontsize', 25)
xlabel('시간(초)','fontweight','bold','fontsize', 20)
ylabel('위치(m)','fontweight','bold','fontsize', 20)
legend('desired','current')
grid on






% subplot(2,4,1)
% plot(t,q1,'linewidth',3);
% title('q1','fontweight','bold','fontsize', 25)
% xlabel('시간(초)','fontweight','bold','fontsize', 20)
% ylabel('위치(rad)','fontweight','bold','fontsize', 20)
% grid on
% 
% subplot(2,4,2)
% plot(t,q2,'linewidth',3);
% title('q2','fontweight','bold','fontsize', 25)
% xlabel('시간(초)','fontweight','bold','fontsize', 20)
% ylabel('위치(rad)','fontweight','bold','fontsize', 20)
% grid on
% 
% subplot(2,4,3)
% plot(t,q3,'linewidth',3);
% title('q3','fontweight','bold','fontsize', 25)
% xlabel('시간(초)','fontweight','bold','fontsize', 20)
% ylabel('위치(rad)','fontweight','bold','fontsize', 20)
% grid on
% 
% subplot(2,4,4)
% plot(t,q4,'linewidth',3);
% title('q4','fontweight','bold','fontsize', 25)
% xlabel('시간(초)','fontweight','bold','fontsize', 20)
% ylabel('위치(rad)','fontweight','bold','fontsize', 20)
% grid on
% 
% subplot(2,4,5)
% plot(t,q5,'linewidth',3);
% title('q5','fontweight','bold','fontsize', 25)
% xlabel('시간(초)','fontweight','bold','fontsize', 20)
% ylabel('위치(rad)','fontweight','bold','fontsize', 20)
% grid on
% 
% subplot(2,4,6)
% plot(t,q6,'linewidth',3);
% title('q6','fontweight','bold','fontsize', 25)
% xlabel('시간(초)','fontweight','bold','fontsize', 20)
% ylabel('위치(rad)','fontweight','bold','fontsize', 20)
% grid on
% 
% subplot(2,4,7)
% plot(t,q7,'linewidth',3);
% title('q7','fontweight','bold','fontsize', 25)
% xlabel('시간(초)','fontweight','bold','fontsize', 20)
% ylabel('위치(rad)','fontweight','bold','fontsize', 20)
% grid on