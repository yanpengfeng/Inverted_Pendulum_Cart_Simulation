function cot = compute_cot(avg_power, x_desired, t_settle,p)

%预期位置除以稳定时间得到速度
speed = x_desired/t_settle(1); 
%cot为加速度或者力存疑？平均功率除以重力乘以速度
cot = avg_power/(p.g * (p.m + p.M) * speed);

end
