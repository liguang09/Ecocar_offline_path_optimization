function [burn, f_drive] = EngineModel(v_on, v_off, v_gear, v_now, v_pre)

f_drive_cons= 385;  %N
dv= v_now- v_pre;

if (v_now<= v_on)
    burn= 1;
end

if (v_now> v_on &&  v_now< v_off && dv>=0)
    burn= 1;
end

if (v_now> v_on &&  v_now< v_off && dv<0)
    burn= 0;
end

if (v_now >= v_off )
    burn= 0;
end

if burn== 1
    if (v_now< v_gear)
        f_drive= f_drive_cons;
    elseif (v_now>=v_gear && v_now<v_off)
        f_drive= 0.5* f_drive_cons;
    end
end
    
if burn== 0
    f_drive= 0;
end

end

