%***********************************************************************

%

%       CONVERTS A RANDOMLY SAMPLED SIGNAL SET INTO AN EVENLY SAMPLED

%       SIGNAL SET (by interpolation)

%       

%	By	:	Haldun KOMSUOGLU

%	Start	:	07/23/1999

%	Last	:	07/23/1999

%	Statue  :	Neural Model Research Material

%	

%	Inputs:	

%              t : A column vector that contains the time values for the

%                  corresponding computed state trajectory points

%              x : A matrix in which each row is a state value on the 

%                  solution trajectory that corresponds to the time value in

%                  vector t with the same row value. Format of each row:

%                          row = [x1 x2 ... xN]

%              Fs: The sampling frequency (1/sec) for the evenly sampled 

%                  set to be generated.

%

%       Outputs:

%              Et : Even sampling instants. This is a column vector in the

%                   same format with "t".

%              Ex : A matrix of the same form with "x" that contains the

%                   state values corresponding to the time instants in Et

%

%***********************************************************************

function [Et, Ex] = even_sample(t, x, Fs, type)



if nargin < 4, type = 'linear'; end


%对时间t求导
dt = diff(t);
%获得导数是否为零，确保其不为零
dt = dt + (dt==0)*1e-5;
%时间取值未知从何而来，相当于对时间添加偏移量？
t = [t(1);t(1)+cumsum(dt)];



% Obtain the process related parameters

%取得状态值的列数
N = size(x, 2);    % number of signals to be interpolated
%取得时间的行数
M = size(t, 1);    % Number of samples provided

t0 = t(1,1);       % Initial time

tf = t(M,1);       % Final time

EM = (tf-t0)*Fs;   % Number of samples in the evenly sampled case with

                   % the specified sampling frequency
%对结果进行四舍五入,生成一个等间距数值的向量
Et = linspace(t0, tf, round(EM))';



% Using linear interpolation (used to be cubic spline interpolation)

% and re-sample each signal to obtain the evenly sampled forms

for s = 1:N,

  %获得差值后的连续状态函数
  Ex(:,s) = interp1(t(:,1), x(:,s), Et(:,1),type); 


end;
