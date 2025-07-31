function [t_settle, t_rise, overshoot, steady_state_error] = analyze_response(tout, y, y_final, flag_step)
%定义了函数名为analyze_response,输入参数为(tout,y,y_final,flag_step),输出参数为[t_settle,t_rise,overshoot,steady_state_error]
% Compute step responses give time (tout), response (y), steady state
% (y_final).数据理想位置
% flag_step is 1 if the response is a step response.
% flag_step is 0 if the resposne is NOT a step response.


if flag_step == 1
    % Compute rise time (t_rise)
    %通过实际位置和理想位置的差值绝对值小于0.01获得数据
    i_90 = find(abs(y - 0.9 * y_final) < 0.01);
    i_10 = find(abs(y - 0.1 * y_final) < 0.01);

    %使用tout来获得系统上升小于百分之十和大于百分之九十的时间，即系统的上升时间阶段
    t_90 = tout(i_90(1));
    t_10 = tout(i_10(1));

    %在阶跃响应下的系统上升时间
    t_rise = t_90 - t_10;
elseif flag_step == 0
    %非阶跃响应，无上升时间
    t_rise = 0;
end


% Compute overshoot（计算溢出）
%取得实际位置的最大值和最大值出现的位置，并与理想位置相减得到超调量
[peak, peak_loc] = max(y);
overshoot = (peak - y_final);



% compute steady state error
%稳态误差计算
steady_state_error = (y(end) - y_final);



% Compute settle time (t_settle)
%系统稳定时间计算
%非阶跃偏置设置为0.05
if flag_step == 1
    non_step_bias = 0;
elseif flag_step == 0 
   non_step_bias = 0.05; 
end


for i = length(y):-1:1 %反向遍历整个输出结果
    if (abs(y(i) - y_final)) > (0.05 * y_final + non_step_bias)
        i_settle = i;
        break
    else
        i_settle = 1;
    end
    
end

t_settle = tout(i_settle);



end
