load 'hw6_1.txt'

t=hw6_1(:,1);

x_d = hw6_1(:,2);
y_d = hw6_1(:,3);
z_d = hw6_1(:,4);
x_c = hw6_1(:,5);
y_c = hw6_1(:,6);
z_c = hw6_1(:,7);
x_obs = hw6_1(:,8);
y_obs = hw6_1(:,9);
z_obs = hw6_1(:,10);
 
[obs_boundary_x, obs_boundary_y, obs_boundary_z] = sphere;

plot3(x_d,y_d,z_d,'linewidth',3)
hold on
plot3(x_c,y_c,z_c,'linewidth',3)
plot3(x_obs, y_obs, z_obs,'Marker', 'o', 'MarkerSize' ,10) 
xlabel('x(m)','fontweight','bold','fontsize', 20)
ylabel('y(m)','fontweight','bold','fontsize', 20)
zlabel('z(m)','fontweight','bold','fontsize', 20)
title('End Effector Trajectory','fontweight','bold','fontsize', 25)
grid on

