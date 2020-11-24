function [state, steer_list, f_dri_list, burn_list, iteration] = StateUpdate_dt(state0, u_steer_dis, t, dt, v_last_iter)

% initialize
L=1.516;
m= 210;
v_gear= 20/3.6;
v_off= 25/3.6;
v_on= 15/3.6;
f_dri_scale= 1000;

state=[];
u=[];

f_dri_list=[];
burn_list=[];
steer_list= [];

iteration= 1;

% iterate steer angle
for j= 1: length(u_steer_dis)
    x= state0(1);
    y= state0(2);
    theta= state0(3);
    v= state0(4);
    steer= u_steer_dis(j);
    
    % iterate time slot
    for i=1:length(t)
        
        % Engine on & off
        if (v<= v_on || (v_on< v <v_off && v>v_last_iter))
            burn= 1;
            iteration= iteration+1;
            
            burn_list= [burn_list, burn];
        end
        
        if (v>= v_off || (v_on<v <v_off && v<v_last_iter))
            burn= 0;
            burn_list= [burn_list, burn];
        end
        
        
        
        % Select torque vs speed
        if burn== 1
            if (v< v_gear)
                f_dri= f_dri_scale;
            end
            
            if (v>=v_gear && v<v_off)
                f_dri= 0.5* f_dri_scale;
            end
            f_dri_list= [f_dri_list, f_dri/f_dri_scale];
        end
        
        if burn== 0
            f_dri= 0;
            f_dri_list= [f_dri_list, f_dri/f_dri_scale];
        end
        
        
        
        % initialize 
        v_tmp= v;
        theta_tmp= theta;
        
        % resistance & x dimension effort
        f_drag= 0.8*1.17*1.04*v_tmp^2*1;
        fx= (-f_drag/2)* cos(steer)+ f_dri- f_drag/2;
        
        % Update states
        theta= theta_tmp+ v_tmp*tan(steer)/L *dt;
        v= v_tmp+ fx/m *dt;
        x= x+ v_tmp*cos(theta_tmp) *dt;
        y= y+ v_tmp*sin(theta_tmp) *dt;
        
        
        % store results
        state=[state, [j; x; y; theta; v]];
        steer_list= [steer_list, steer];
    end
end

end