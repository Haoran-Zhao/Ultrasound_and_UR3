clear all
close all

pts = [10.164, 33.237; 10.574, 28.747; 11.852, 24.426; 13.95, 20.432; 16.785, 16.926; 20.25, 14.041; 24.211, 11.888; 28.516, 10.548; 33, 10.075; 43.5, 10;
    54, 10; 58.487 10.442; 62.802 11.751; 66.778 13.876; 70.263 16.737; 73.124 20.222; 75.249 24.198; 76.558 28.513; 77 33; 76.558 37.487; 75.249 41.802; 73.124 45.778; 70.262 49.265;
    66.778 52.124; 62.802 54.249; 58.487 55.558; 54 56; 43.663 55.999; 33.327 56.073; 28.837 55.663; 24.513 54.385; 20.522 52.288; 17.018 49.454; 14.131 45.988; 11.977 42.026; 10.638 37.721; 10.164, 33.237];
orig = [43.582 32.999];
pts = pts - orig;

nbLandmarks = 60;
stepLengths = sqrt(sum(diff(pts,[],1).^2,2));
stepLengths = [0; stepLengths]; % add the starting point
cumulativeLen = cumsum(stepLengths); % Cumulative sum of all items --> last item contains total length
finalStepLocs = linspace(0,cumulativeLen(end), nbLandmarks);
finalPathXY = interp1(cumulativeLen, pts, finalStepLocs);
save_arry(finalPathXY);
plot(pts(:,1), pts(:,2), 'o', finalPathXY(:,1), finalPathXY(:,2), ':.');


function save_arry(pts)
    fid = fopen('2D_traj.txt', 'wt');
    for r = 1: size(pts,1)
        fprintf(fid, '%f, %f; ', pts(r,1), pts(r,2));
    end
    fclose(fid);
end