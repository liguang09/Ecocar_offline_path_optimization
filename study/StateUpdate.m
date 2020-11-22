function [state, u] = StateUpdate(state0, u_steer, t, dt)

% initial states
x= state0(1);
y= state0(2);
theta= state0(3);
v= state0(4);

L=1.516;
m= 210;

v_gear= 20/3.6;
v_off= 22/3.6;
v_on= 18/3.6;
f_dri_scale= 1000;

state=[];
u=[];

f_dri= f_dri_scale;
f_brk= 0;
steer= 0;


for i=1:length(t)
    steer= u_steer(i);
    
    v_tmp= v;
    theta_tmp= theta;
    
    f_drag= 0.8*1.17*1.04*v_tmp^2*1;
    
    fx= (-f_drag*L/2)* cos(steer)+ f_dri- f_drag*L/2;
    
    theta= theta_tmp+ v_tmp*tan(steer)*dt/L;
    v= v_tmp+ fx*dt/m;
    x= x+ v_tmp*cos(theta_tmp)*dt;
    y= y+ v_tmp*sin(theta_tmp)*dt;
    
    if (v<= v_on)
        burn= 1;
    elseif (v>= v_off)
        burn= 0;
    end
    
    if burn== 1
        if (v< v_gear)
            f_dri= f_dri_scale;
        elseif (v>=v_gear && v<v_off)
            f_dri= 0.3* f_dri_scale;
        end
    else
        f_dri= 0;
    end
    
    state=[state, [x; y; theta; v]];
    u=[u, [steer; f_dri/f_dri_scale]];
end

end