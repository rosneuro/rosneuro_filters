clearvars; 

filein = './test/rawdata.csv';
data = readmatrix(filein);

%% rosneuro filtered data
file_blackman = './test/blackman_window.csv';
rosneuro_blackman = readmatrix(file_blackman);


%% MATLAB filtered data
SampleRate = 512;
nsamples   = size(data, 1);
nchannels  = size(data, 2);
channelId  = 2;

matlab_blackman = data .* repmat(blackman(nsamples), [1 nchannels]);


%% Visualization

t = 0:1/SampleRate:nsamples/SampleRate - 1/SampleRate;

figure;
subplot(3, 1, [1 2]);
hold on; 
plot(t, rosneuro_blackman(:, channelId), 'b', 'LineWidth', 1); 
plot(t, matlab_blackman(:, channelId), 'r'); 
plot(t, data(:, channelId), 'k'); 
hold off; 
grid on;
xlabel('time [s]');
ylabel('amplitude [uV]');
legend('rosneuro', 'matlab', 'data');
title(['Dc filter | channel=' num2str(channelId)]);

subplot(3, 1, 3)
bar(t, abs(matlab_blackman(:, channelId)- rosneuro_blackman(:, channelId)));
grid on;
xlabel('time [s]');
ylabel('amplitude [uV]');
title('Difference')

sgtitle('Evaluation Blackman window')

