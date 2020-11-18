function s = StateUpdate(s0, u, t, dt)

% initial states
x= s0(1);
y= s0(2);
theta= s0(3);
v= s0(4);

L=1.516;
m= 210;

v_gear= 15/3.6;
v_off= 25/3.6;

s=[];

f_dri= 1000;
f_brk= 0;
steer= 0;

    for i=1:length(t)
        steer= u(i,1);
        f_brk= u(i,2);

        v_tmp= v;
        theta_tmp= theta;

        f_drag= 0.8*1.17*1.04*v_tmp^2;

        theta= theta_temp+ v_tmp*tan(steer)*dt/L;
        v= v_temp+ (f_dri+f_brk-f_drag)*dt/m;
        x= x+ v_temp*cos(theta_tmp)*dt;
        y= y+ v_temp*sin(theta_tmp)*dt;

        if (v< v_gear)
            f_dri= 1000;
        elseif (v>=v_gear && v<v_off)
            f_dri= 500;
        elseif (v>= v_off)
            f_dri= 0;
        end
        
        s=[s, [x; y; theta; v]];
    end
    
end