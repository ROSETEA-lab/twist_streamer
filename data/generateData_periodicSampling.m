clear all

% Generate data
t = 0:0.01:10;
linVel = 4*sawtooth(2*pi*0.2*t);
angVel = 2*sin(5*t);

% Plot data
figure,
subplot(2,1,1),plot(t,linVel),xlabel('Time [s]'),ylabel('Linear velocity [m/s]')
subplot(2,1,2),plot(t,angVel),xlabel('Time [s]'),ylabel('Angular velocity [rad/s]')

% Write data to file
csvwrite('data_periodic.csv',[t', linVel', angVel']);
