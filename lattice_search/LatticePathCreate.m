function [path_states, u, num_iter, steer_vector]= LatticePathCreate(state0, dt, steer_goal, dsteer, v_last, steer_last, s_stop)

% This function is to generate a path after steer angle index is determined

%%  parameters
L=1.516;
m= 210;
dv= state0(4)- v_last;
num_iter= 0; % iteration for each steer angle
f_dri_scale= 1000;
f_brk_scale= 50;
s_cum= 0;
iter= 0;

u_steer= steer_last;

path_states=[];
path_info= [];
u=[];

v_gear= 20/3.6;
v_off= 25/3.6;
v_on= 15/3.6;

f_dri_last= 0;

%% select the steer angle change rate
if steer_goal- steer_last>0
    steer_vector= [steer_last: dsteer: steer_goal ];
elseif steer_goal- steer_last<0
    steer_vector= [steer_last: -dsteer: steer_goal ];
elseif steer_goal- steer_last==0
    steer_vector= [steer_goal];
end

%% look ahead path
while (s_cum< s_stop)
    
    iter= iter+1;
    
    if iter<= length(steer_vector)
        u_steer= steer_vector(iter);
    else
        u_steer= steer_vector(end);
    end
    
    x_now= state0(1);
    y_now= state0(2);
    theta_now= state0(3);
    v_now= state0(4);
    
    % Engine on & off
    if (v_now<= v_on)
        burn= 1;
    elseif (v_now> v_on && v_now< v_off)
        if dv>=0
            burn= 1;
        elseif dv< 0
            burn= 0;
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
    
    if v_now> v_off || (f_dri_last==  f_dri_scale && f_dri_last- f_dri== 0.5*f_dri_scale)
        f_brk= -0* f_brk_scale;
    else
        f_brk=0;
    end
    
    
    % resistance & x dimension effort
    f_drag= 0.5*0.14*1.15*1.7577*v_now^2*5;
    fx= (f_brk/2-f_drag/2)* cos(u_steer)+ f_dri- f_drag/2+f_brk/2;
    
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
    dv= v- v_now;
    
    state0= [x, y, theta, v];
    s_cum= s_cum+ ds;
    
    f_dri_last= f_dri;
    
    % store results
    path_states=[path_states; [x, y, theta, v]];
    % path_info= [path_info; [ds, dydx]];
    u= [u; [f_dri/f_dri_scale, burn, u_steer, f_brk/f_brk_scale]];
end
num_iter= iter;
end