function [state, u] = StateUpdate_ds(state0, u_steer_dis, s, ds, sf)

% initialize
L=1.516;
m= 210;
v_gear= 20/3.6;
v_off= 22/3.6;
v_on= 18/3.6;
f_dri_scale= 1000;
burn= 1; 
state=[];
u=[];

% iterate steer angle
for j= 1: length(u_steer_dis)
    x= state0(1);
    y= state0(2);
    theta= state0(3);
    v= state0(4);
    
    f_dri= f_dri_scale;
    
    % iterate time slot
    for i=1:length(s)
        steer= u_steer_dis(j);
        v_tmp= v;
        theta_tmp= theta;
        
        % resistance & x dimension effort
        f_drag= 0.8*1.17*1.04*v_tmp^2*1;
        fx= (-f_drag*L/2)* cos(steer)+ f_dri- f_drag*L/2;
        
        % Update states
        dtds= 1/(v_tmp*sf(i));
        theta= theta_tmp+ v_tmp*tan(steer)*ds(i)/L* dtds;
        v= v_tmp+ fx*ds(i)/m* dtds;
        x= x+ v_tmp*cos(theta_tmp)*ds(i)* dtds;
        y= y+ v_tmp*sin(theta_tmp)*ds(i)* dtds;
        
        % Engine on & off
        v_sel= v/dtds;
        
        if (v_sel<= v_on)
            burn= 1;
        elseif (v_sel>= v_off)
            burn= 0;
        end
        
        % Select torque vs speed
        if burn== 1
            if (v_sel< v_gear)
                f_dri= f_dri_scale;
            elseif (v_sel>=v_gear && v_sel<v_off)
                f_dri= 0.5* f_dri_scale;
            end
        else
            f_dri= 0;
        end
         % store results
        state=[state, [x; y; theta; v]];
        u=[u, [steer; f_dri/f_dri_scale]];
    end
end

end