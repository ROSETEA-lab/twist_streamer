clear all

% Generate data
t = 0:0.01:10;
linVel = 4*ones(size(t));
angVel = 2*sin(5*t);

% Write data to file
csvwrite('data.csv',[t', linVel', angVel']);
