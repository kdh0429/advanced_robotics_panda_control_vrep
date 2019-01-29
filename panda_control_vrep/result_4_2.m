load 'hw4_2.txt'

t=hw4_2(:,1);

q_d = hw4_2(:,2)*180/pi;
%q_d = -25*ones(size(t,1),1);
q = hw4_2(:,3)*180/pi;

plot(t,q_d)
hold on 
plot(t,q)
title('관절4')
xlabel('시간(초)')
ylabel('위치(도)')
legend('desired','current')
grid on
hold off