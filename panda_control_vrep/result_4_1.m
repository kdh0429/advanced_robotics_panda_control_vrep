load 'hw4_1.txt'

t=hw4_1(:,1);

q_d = -25*ones(size(t,1),1);
q = hw4_1(:,3)*180/pi;

plot(t,q_d)
hold on 
plot(t,q)
title('����4')
xlabel('�ð�(��)')
ylabel('��ġ(��)')
legend('desired','current')
grid on
hold off