function [path_states, path_info, u]= PathsGenerator(state0, dt, u_steer)
% parameters
L=1.516;
m= 210;
dv= 0;

path_states=[];
path_info= [];
u=[];

v_gear= 20/3.6;
v_off= 25/3.6;
v_on= 15/3.6;

f_dri_scale= 1000;

% iterate time slot
for i=1:length(u_steer)
    
    x_now= state0(1);
    y_now= state0(2);
    theta_now= state0(3);
    v_now= state0(4);
    
    % Engine on & off
    if (v_now<= v_on)
        burn= 1;
    elseif (v_now> v_on && v< v_off)
        if dv>=0
            burn= 1;
        elseif dv< 0
            burn=0;
        end
    elseif (v_now >= v_off )
        burn= 0;
    end
    
    % Select torque vs speed
    if burn== 1
        if (v_now< v_gear)
            f_dri= f_dri_scale;
        elseif (v_now>=v_gear && v<v_off)
            f_dri= 0.5* f_dri_scale;
        end
        
    elseif burn== 0
        f_dri= 0;
    end
    
    % resistance & x dimension effort
    f_drag= 0.5*0.14*1.15*1.7577*v_now^2*10;
    fx= (-f_drag/2)* cos(u_steer(i))+ f_dri- f_drag/2;
    
    % Update states based on kinematic model
    x= x_now;
    y= y_now;
    theta= theta_now;
    v= v_now;
    
    theta= theta+ v* tan(u_steer(i))/L *dt;
    v= v+ fx/m *dt;
    x= x+ v*cos(theta) *dt;
    y= y+ v*sin(theta) *dt;
    
    ds= norm([x- x_now, y- y_now], 2);
    dydx= (y- y_now)/ (x- x_now);
    
    % store results
    path_states=[path_states; [x, y, theta, v]];
    path_info= [path_info; [ds, dydx]];
    u= [u; [f_dri/f_dri_scale, burn, u_steer(i)]];
end

end