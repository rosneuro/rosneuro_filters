clearvars; 

filein = './test/rawdata.csv';
data = readmatrix(filein);

%% rosneuro filtered data
file_hamming = './test/hamming_window.csv';
rosneuro_hamming = readmatrix(file_hamming);


%% MATLAB filtered data
SampleRate = 512;
nsamples   = size(data, 1);
nchannels  = size(data, 2);
channelId  = 2;

matlab_hamming = data .* repmat(hamming(nsamples), [1 nchannels]);


%% Visualization

t = 0:1/SampleRate:nsamples/SampleRate - 1/SampleRate;

figure;
subplot(3, 1, [1 2]);
hold on; 
plot(t, rosneuro_hamming(:, channelId), 'b', 'LineWidth', 1); 
plot(t, matlab_hamming(:, channelId), 'r'); 
plot(t, data(:, channelId), 'k'); 
hold off; 
grid on;
xlabel('time [s]');
ylabel('amplitude [uV]');
legend('rosneuro', 'matlab', 'data');
title(['Hamming window | channel=' num2str(channelId)]);

subplot(3, 1, 3)
bar(t, abs(matlab_hamming(:, channelId)- rosneuro_hamming(:, channelId)));
grid on;
xlabel('time [s]');
ylabel('amplitude [uV]');
title('Difference')

sgtitle('Evaluation Hamming window')

