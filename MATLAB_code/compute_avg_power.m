function avg_power = compute_avg_power(Uout, x_dot, p)
%Uout为系统的输出，x_dot为系统位置，p为参数集
N = length(Uout);
current = zeros(N,1);
power = zeros(N,1);

for i = 1:N
    %小车位移除以皮带轮盘半径等于转的圈数
    omega = x_dot(i) / p.r_pulley;
    %输出减去电机速度常数乘以圈数然后再除以电机电阻得到电流
    current(i) = (Uout(i) - p.Kv * omega)/p.Rw;
    %然后由电流乘以输出得到功率输出
    power(i) = current(i) * Uout(i);
end

avg_power = mean(power);

end
