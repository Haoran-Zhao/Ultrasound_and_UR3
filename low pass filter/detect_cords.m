clear
close

data = readmatrix('GFG.csv');
t = 0:150/1222:150;
t = t(1:end-1);
data = [data, t'];
last = data(1,:);
dist_hist = [0];
distsm_hist = [0];

n = length(data);
x = [data(1,1)];
y = [data(1,2)];
xs = [data(1,1)];
ys = [data(1,2)];
last_x = data(1,1);
last_y = data(1,2);
alpha = 0.3;
figure(1)
set(gcf, 'Position',  [700, 100, 650, 600]);
subplot(4,1,1)
axis([min(data(:,1)-10) max(data(:,1)+10) min(data(:,2)-10) max(data(:,2)+10) 0, n+1])
h1 = plot3(data(1,1), data(1,2), data(1,3), 'o');
zlabel('Time [s]')
xlabel('x [mm]')
ylabel('y [mm]')
view([0 90 0 ])
subplot(4,1,2)
axis([0 1222 0 20])
h2 = plot(dist_hist, 'b');
hold on
h3 = plot(distsm_hist,'r');
ylabel('delta D [mm]');
xlabel('Time [s]');
legend('raw data', 'filtered data');

subplot(4,1,3)
h4 = plot(x,'b');
hold on
h5 = plot(xs, 'r');
ylabel('X [mm]');
xlabel('Time [s]');
legend('raw data', 'filtered data');

subplot(4,1,4)
h6 = plot(y, 'b');
hold on
h7 = plot(ys, 'r');
ylabel('Y [mm]');
xlabel('Time [s]');
legend('raw data', 'filtered data');

for i = 2:n
    set(h1,'Xdata', data(1:i,1), 'YData', data(1:i,2), 'ZData', data(1:i,3));
    dist = sqrt((last(1)-data(i,1))^2 + (last(2)-data(i,2))^2)*0.283;
    last = data(i,:);
    dist_hist = [dist_hist , dist];
    set(h2,'Xdata', data(1:i,3), 'YData', dist_hist(1, 1:length(dist_hist)));

    dx = lpf(data(i,1), last_x, alpha);
    dy = lpf(data(i,2), last_y, alpha);
    distsm = sqrt((last_x-dx)^2 + (last_y-dy)^2)*0.283;
    last_x = dx;
    last_y = dy;
    x = [x, data(i,1)];
    y = [y, data(i,2)];
    xs = [xs, dx];
    ys = [ys, dy];
    distsm_hist = [distsm_hist , distsm];
    set(h3,'Xdata', data(1:i,3), 'YData', distsm_hist);
    set(h4,'Xdata', data(1:i,3), 'YData', x);
    set(h5,'Xdata', data(1:i,3), 'YData', xs);
    set(h6,'Xdata', data(1:i,3), 'YData', y);
    set(h7,'Xdata', data(1:i,3), 'YData', ys);

    drawnow
%     pause(0.1)
end

function [output] = lpf(input, last, alpha)
    output = last + alpha*(input - last);
end