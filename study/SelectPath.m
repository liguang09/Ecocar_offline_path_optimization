function [select_path_index, states_select, f_dri_select, burn_select] = SelectPath(u_steer, xy_ref,  state0, t, dt)

x_ref= xy_ref(1);
y_ref= xy_ref(2);

d_err_list=[];

for i=1: length(u_steer)
    
    states = EngineModel(state0, u_steer(i), t, dt);
    
    d_err_sum=0;
    
    for k= 1: length(states)
        d_err_tmp= norm([x_ref- states(1, k), y_ref- states(2,k)],2);
        d_err_sum= d_err_sum+ d_err_tmp;
    end
    
    d_err_list= [d_err_list; d_err_sum];
    
end

select_path_index= find(d_err_list==(min(d_err_list)));

[states_select, f_dri_select, burn_select] = EngineModel(state0, u_steer(select_path_index), t, dt);

end

