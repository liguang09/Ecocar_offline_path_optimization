function states_update= VehicleKinematic(u, state_now)
%% variables initial
dt= 0.001;
L=1.516;
m= 210;

u_steer= u(0);
u_f_drive= u(1);

x_now= state_now(1);
y_now= state_now(2);
theta_now= state_now(3);
v_now= state_now(4);
beta_now= state_now(5);
omega_now= state_now(6);

%% Kinematic model

% drag resistiance
f_drag=  0.5* 0.14* 1.15* 1.7577* v_now^2* 5;

% driving dimension effort
f_x= f_dri+ (-f_drag/2)* cos(u_steer)- f_drag/2;

% States update
x= x_now;
y= y_now;
theta= theta_now;
v= v_now;
beta= beta_now;
omega= omega_now;

theta= theta+ v* tan(u_steer)/L *dt;
v= v+ fx/m *dt;
x= x+ v*cos(theta) *dt;
y= y+ v*sin(theta) *dt;

states_update= [x, y, theta, v, beta, omega];

end

