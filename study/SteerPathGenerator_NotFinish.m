function [path_states, path_info, u]= SteerPathGenerator(state0, dt, u_steer, s_stop)

% This function is to generate the path for each steer angle

% parameters
L=1.516;
m= 210;
dv= 0;

path_states=[];
path_info= [];
u=[];

v_gear= 25/3.6;
v_off= 30/3.6;
v_on= 10/3.6;

f_dri_scale= 1000;
s_cum= 0;

states_list=[];

% iterate time slot
%for i=1:length(u_steer)
while (s_cum< s_stop)
    
    x_now= state0(1);
    y_now= state0(2);
    theta_now= state0(3);
    v_now= state0(4);
    
    % Engine on & off
    if (v_now<= v_on)
        burn= 1;
        
        while(v_now<= v_gear)
            f_dri= f_dri_scale;
            [states, ds]= StateUpdate(states_now, steer, f_dri, dt);
            s_cum= s_cum+ ds;
            
            v_now= states(4);
            states_list= [states_list; states];
        end
        
    elseif (v_now> v_on && v_now< v_off)
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
        elseif (v_now>=v_gear && v_now<v_off)
            f_dri= 0.5* f_dri_scale;
        end
        
    elseif burn== 0
        f_dri= 0;
    end
    
    % resistance & x dimension effort
    f_drag= 0.5*0.14*1.15*1.7577*v_now^2;
    fx= (-f_drag/2)* cos(u_steer)+ f_dri- f_drag/2;
    
    % Update states based on kinematic model
    x= x_now;
    y= y_now;
    theta= theta_now;
    v= v_now;
    
    theta= theta+ v* tan(u_steer)/L *dt;
    v= v+ fx/m *dt;
    x= x+ v*cos(theta) *dt;
    y= y+ v*sin(theta) *dt;
    
    ds= norm([x- x_now, y- y_now], 2);
    dydx= (y- y_now)/ (x- x_now);
    
    state0= [x, y, theta, v];
    s_cum= s_cum+ ds;
    
    % store results
    path_states=[path_states; [x, y, theta, v]];
    path_info= [path_info; [ds, dydx]];
    u= [u; [f_dri/f_dri_scale, burn, u_steer]];
end

end

function [states, ds]= StateUpdate(states_now, steer, f_dri, dt)
    
    f_drag= 0.5*0.14*1.15*1.7577*v_now^2;
    fx= (-f_drag/2)* cos(u_steer)+ f_dri- f_drag/2;
    
    % Update states based on kinematic model
    x= states_now(1);
    y= states_now(2);
    theta= states_now(3);
    v= states_now(4);
    
    theta= theta+ v* tan(steer)/L *dt;
    v= v+ fx/m *dt;
    x= x+ v*cos(theta) *dt;
    y= y+ v*sin(theta) *dt;
    
    ds= norm([x- states_now(1), y- states_now(2)], 2);
    
    states=[x, y, theta, v];
end