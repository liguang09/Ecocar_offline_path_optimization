function [states, f_dri_list, burn_list] = EngineModel(state0, u_steer, t, dt, ds)

% parameters
L=1.516;
m= 210;
dv= 0;

states=[];
u=[];

f_dri_list=[];
burn_list=[];

v_gear= 20/3.6;
v_off= 25/3.6;
v_on= 15/3.6;

f_dri_scale= 1000;

x= state0(1);
y= state0(2);
theta= state0(3);
v= state0(4);

s_run_cum= 0;

% iterate time slot
for i=1:length(t)
%while(s_run_cum< ds)
    
    % Engine on & off
    if (v<= v_on)
        burn= 1;
    elseif (v> v_on && v< v_off)
        if dv>=0
            burn= 1;
        elseif dv< 0
            burn=0;
        end
    elseif (v >= v_off )
        burn= 0;
    end
    
    burn_list= [burn_list, burn];
    
    % Select torque vs speed
    if burn== 1
        if (v< v_gear)
            f_dri= f_dri_scale;
        elseif (v>=v_gear && v<v_off)
            f_dri= 0.5* f_dri_scale;
        end
        
    elseif burn== 0
        f_dri= 0;
    end
    
    f_dri_list= [f_dri_list, f_dri/f_dri_scale];
    
    % initialize
    v_tmp= v;
    theta_tmp= theta;
    x_tmp= x;
    y_tmp= y;
    
    % resistance & x dimension effort
    f_drag= 0.5*0.14*1.15*1.7577*v_tmp^2*10;
    fx= (-f_drag/2)* cos(u_steer)+ f_dri- f_drag/2;
    
    % Update states based on kinematic model
    theta= theta_tmp+ v_tmp*tan(u_steer)/L *dt;
    v= v_tmp+ fx/m *dt;
    x= x+ v_tmp*cos(theta_tmp) *dt;
    y= y+ v_tmp*sin(theta_tmp) *dt;
    
    dv= v- v_tmp;
    s_run= norm([x-x_tmp, y-y_tmp], 2);
    s_run_cum= s_run_cum+ s_run;
    
    
    % store results
    states=[states, [x; y; theta; v]];
    
end

end