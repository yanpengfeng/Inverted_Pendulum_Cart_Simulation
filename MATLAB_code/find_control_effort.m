function Uout = find_control_effort(t,X,desired_trajectory, p)
% Compute the control effort for each controller 
%获得输出参数更新
%小车参考位置参数
r_array = desired_trajectory(:,1); % reference signal of cart position

t_prev = 0;
integral_term_1_prev = 0;
integral_term_2_prev = 0;
Uout = zeros(length(t),1);

    
for i_data = 1:length(t)    
    %取时间参数长度
    r = r_array(i_data);
    x = X(i_data, 1);
    theta = X(i_data,2);
    x_dot = X(i_data,3);
    theta_dot = X(i_data,4);

    [A,B,C,D] = compute_cart_pole_linear_system(X(i_data,:),r,p);

    if p.flag_ctrl == 0
        %设计lqr控制器获得输出
        [K, Nbar, r_new] = lqr_controller(X(i_data,:)',r,p);
        Uout(i_data) = r_new * Nbar - K * X(i_data,:)'; 
        
    elseif p.flag_ctrl == 1
        Kp1 = p.Kp1;
        Ki1 = p.Ki1;
        Kd1 = p.Kd1;

        Kp2 = p.Kp2;
        Ki2 = p.Ki2;
        Kd2 = p.Kd2;


        %取得当前时刻参数和上一时刻参数
        t_cur = t(i_data);
        if i_data > 1  
            t_prev = t(i_data - 1);
        end

        %更新角度和位置参数，通过误差计算得到
        integral_term_1_new = (t_cur - t_prev) * (r-x);
        integral_term_2_new = (t_cur- t_prev) * (-theta);

        %更新角度和位移参数输出的控制器，双控制器设计
        u_pid = Kp1 * (r - x) + Kd1 * (-x_dot) + Ki1 * (integral_term_1_new + integral_term_1_prev);
        u_pid = u_pid + Kp2 * (-theta) + Kd2 * (-theta_dot) + Ki2 * (t_cur- t_prev) * (integral_term_2_new + integral_term_2_prev);


        %更新参数，当前参数加旧参数
        integral_term_1_prev = integral_term_1_new + integral_term_1_prev;

        integral_term_2_prev = integral_term_2_new + integral_term_2_prev;
        
        if p.saturate_motor == 1
            u_pid_new = saturate_voltage(u_pid,p);
        elseif p.saturate_motor == 0
            u_pid_new = u_pid;
        end


        Uout(i_data) = u_pid_new;

    elseif p.flag_ctrl == 2 


    end
end

end
