function [t,y] = RK4(ODESet,TimeSpan,InitialValue,varargin)
% 2019       V1
% 2020/08/25 V2

switch nargin
    case 3
        h = 0.001; % Default step size
    case 4
        h  = varargin{1};
end

% ... User Given
t0 = TimeSpan(1);
tf = TimeSpan(2);
y0 = InitialValue;

% ... RK4
t = t0:h:tf;
n = size(y0,1);
m = length(t);

y     = zeros(n,m);
y(:,1) = y0;
for i = 1:m-1
    yi = y(:,i);
    ti = t(i);
    f1 = ODESet(ti,yi);
    f2 = ODESet(ti+0.5*h,yi+0.5*h*f1);
    f3 = ODESet(ti+0.5*h,yi+0.5*h*f2);
    f4 = ODESet(ti+h,yi+h*f3);
    y(:,i+1) = yi + h*( 1/6*f1 + 1/3*f2 + 1/3*f3 + 1/6*f4 );
end

y = y.'; 
 % end of RK4

