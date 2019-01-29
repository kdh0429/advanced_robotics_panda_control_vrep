load 'hw2-1.txt'

t_2_1=hw2_1(:,1);

x_d_2_1 = hw2_1(:,2);
y_d_2_1 = hw2_1(:,3);
z_d_2_1 = hw2_1(:,4);
x_c_2_1 = hw2_1(:,5);
y_c_2_1 = hw2_1(:,6);
z_c_2_1 = hw2_1(:,7);
x_e_2_1 = hw2_1(:,8);
y_e_2_1 = hw2_1(:,9);
z_e_2_1 = hw2_1(:,10);
xori_e_2_1 = hw2_1(:,11);
yori_e_2_1 = hw2_1(:,12);
zori_e_2_1 = hw2_1(:,13);
xdot_d_2_1 = hw2_1(:,14);
ydot_d_2_1 = hw2_1(:,15);
zdot_d_2_1 = hw2_1(:,16);


plot(t_2_1,xori_e_2_1);
%plot(t,x_d)
%hold on
%plot(t,x_c)