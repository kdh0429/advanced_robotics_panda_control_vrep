load 'hw2-3.txt'

t_2_3=hw2_3(:,1);

x_d_2_3 = hw2_3(:,2);
y_d_2_3 = hw2_3(:,3);
z_d_2_3 = hw2_3(:,4);
x_c_2_3 = hw2_3(:,5);
y_c_2_3 = hw2_3(:,6);
z_c_2_3 = hw2_3(:,7);
x_e_2_3 = hw2_3(:,8);
y_e_2_3 = hw2_3(:,9);
z_e_2_3 = hw2_3(:,10);
xori_e_2_3 = hw2_3(:,11);
yori_e_2_3 = hw2_3(:,12);
zori_e_2_3 = hw2_3(:,13);
xdot_d_2_3 = hw2_3(:,14);
ydot_d_2_3 = hw2_3(:,15);
zdot_d_2_3 = hw2_3(:,16);


plot(t_2_3,x_e_2_3);
%plot(t,x_d)
%hold on
%plot(t,x_c)