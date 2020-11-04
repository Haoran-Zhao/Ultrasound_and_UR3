%% 
clear
close all

dt = xlsread('Force record.xlsx');
t = dt(:,1);
fx = dt(:,5);
n = length(fx);
%% 
raw = [0];
filtered = [0];
last = NaN;
alpha = 0.05;

figure(1)
h1 = plot(raw, 'b');
hold on
h2 = plot(filtered, 'r');

for i = 1:n
    if isnan(last)
        last = fx(i);
        raw = [fx(i)];
        filtered = [last];
    else
        if abs(last) - abs(fx(i)) >=1.5
            raw = [raw, last];
            filtered = [filtered, last];
            continue
        end
        d = lpf(fx(i), last, alpha);
        last = d;
        raw = [raw, fx(i)];
        filtered = [filtered, last];
    end
    
    set(h1, 'XData', dt(1:i), 'YData', raw(:));
    set(h2, 'XData', dt(1:i), 'YData', filtered(:));
    drawnow;
    
end

function [output] = lpf(input, last, alpha)
    output = last + alpha*(input - last);
end