clearvars; 

filein = './test/rawdata.csv';
data = readmatrix(filein);

%% rosneuro filtered data
file_laplacian = './test/laplacian_simloop.csv';
file_laplacian_mask = './test/laplacian_mask_32.mat';

rosneuro_laplacian = readmatrix(file_laplacian);
mask = load(file_laplacian_mask);
mask = mask.lap;

%% MATLAB filtered data
SampleRate = 512;
framesize  = 32;
nsamples   = size(data, 1);
channelId  = 21;

start = 1:framesize:nsamples;
stop  = start + framesize - 1;
nframes = length(start);

for fId = 1:nframes
    cstart = start(fId);
    cstop  = stop(fId);
    matlab_laplacian(cstart:cstop, :) = data(cstart:cstop, :) * mask;
end

%% Visualization

t = 0:1/SampleRate:nsamples/SampleRate - 1/SampleRate;

figure;
subplot(3, 1, [1 2]);
hold on; 
plot(t, rosneuro_laplacian(:, channelId), 'b', 'LineWidth', 1); 
plot(t, matlab_laplacian(:, channelId), 'r'); 
plot(t, data(:, channelId), 'k'); 
hold off; 
grid on;
xlabel('time [s]');
ylabel('amplitude [uV]');
legend('rosneuro', 'matlab', 'data');
title(['Laplacian filter | channel=' num2str(channelId)]);

subplot(3, 1, 3)
bar(t, abs(matlab_laplacian(:, channelId)- rosneuro_laplacian(:, channelId)));
grid on;
xlabel('time [s]');
ylabel('amplitude [uV]');
title('Difference')

sgtitle('Evaluation Laplacian simloop')

