function [state, u] = StateUpdate_dt(state0, u_steer_dis, t, dt, v_last_iter)

% initialize
L=1.516;
m= 210;
v_gear= 15/3.6;
v_off= 25/3.6;
v_on= 10/3.6;
f_dri_scale= 100;

state=[];
u=[];

% iterate steer angle
for j= 1: length(u_steer_dis)
    x= state0(1);
    y= state0(2);
    theta= state0(3);
    v= state0(4);
    
    % iterate time slot
    for i=1:length(t)
        
        % Engine on & off
        if (v<= v_on || (v_on<v <v_off && v>v_last_iter))
            burn= 1;
        elseif (v>= v_off || (v_on<v <v_off && v<v_last_iter))
            burn= 0;
        end
        
        % Select torque vs speed
        if burn== 1
            if (v< v_gear)
                f_dri= f_dri_scale;
            elseif (v>=v_gear && v<v_off)
                f_dri= 0.5* f_dri_scale;
            end
        else
            f_dri= 0;
        end
        
        % initialize 
        steer= u_steer_dis(j);
        v_tmp= v;
        theta_tmp= theta;
        
        % resistance & x dimension effort
        f_drag= 0.8*1.17*1.04*v_tmp^2*1;
        fx= (-f_drag*L/2)* cos(steer)+ f_dri- f_drag*L/2;
        
        % Update states
        theta= theta_tmp+ v_tmp*tan(steer)*dt/L;
        v= v_tmp+ fx*dt/m;
        x= x+ v_tmp*cos(theta_tmp)*dt;
        y= y+ v_tmp*sin(theta_tmp)*dt;
        
        
        % store results
        state=[state, [x; y; theta; v]];
        u=[u, [steer; f_dri/f_dri_scale]];
    end
end

end