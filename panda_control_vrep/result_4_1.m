load 'hw4_1.txt'

t=hw4_1(:,1);

q_d = -25*ones(size(t,1),1);
q = hw4_1(:,3)*180/pi;

plot(t,q_d)
hold on 
plot(t,q)
title('관절4')
xlabel('시간(초)')
ylabel('위치(도)')
legend('desired','current')
grid on
hold off