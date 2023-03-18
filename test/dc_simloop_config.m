clearvars; 

filein = './test/rawdata.csv';
data = readmatrix(filein);

%% rosneuro filtered data
file_dc = './test/dc_simloop_config.csv';
rosneuro_dc = readmatrix(file_dc);


%% MATLAB filtered data
SampleRate = 512;
framesize  = 32;
nsamples   = size(data, 1);
channelId  = 2;


start = 1:framesize:nsamples;
stop  = start + framesize - 1;
nframes = length(start);

for fId = 1:nframes
    cstart = start(fId);
    cstop  = stop(fId);
    matlab_dc(cstart:cstop, :) = dc(data(cstart:cstop, :));
end

%% Visualization

t = 0:1/SampleRate:nsamples/SampleRate - 1/SampleRate;

figure;
subplot(3, 1, [1 2]);
hold on; 
plot(t, rosneuro_dc(:, channelId), 'b', 'LineWidth', 1); 
plot(t, matlab_dc(:, channelId), 'r'); 
plot(t, data(:, channelId), 'k'); 
hold off; 
grid on;
xlabel('time [s]');
ylabel('amplitude [uV]');
legend('rosneuro', 'matlab', 'data');
title(['Dc filter | channel=' num2str(channelId)]);

subplot(3, 1, 3)
bar(t, abs(matlab_dc(:, channelId)- rosneuro_dc(:, channelId)));
grid on;
xlabel('time [s]');
ylabel('amplitude [uV]');
title('Difference')

sgtitle('Evaluation DC simloop')

%% Functions
function dataout = dc(datain)
 	idc = mean(datain, 1);
    dataout = datain - repmat(idc, size(datain, 1), 1);
end
