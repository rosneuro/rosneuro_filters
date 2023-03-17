clearvars; 

filein = './test/input.csv';
data = readmatrix(filein);

%% rosneuro filtered data
file_rtfilter_lp = './test/butterworth_simloop_configuration_outlp.csv';
file_rtfilter_hp = './test/butterworth_simloop_configuration_outhp.csv';

rtfilter_lp = readmatrix(file_rtfilter_lp);
rtfilter_hp = readmatrix(file_rtfilter_hp);

%% MATLAB filtered data
SampleRate = 512;
channelId  = 2;
nsamples   = size(data, 1);
nchannels  = size(data, 2);

order_lp  = 4;
order_hp  = 4;  
cutoff_lp = 10;
cutoff_hp = 1;

[b_lp, a_lp] = butter(order_lp, cutoff_lp/(SampleRate/2), 'low');
[b_hp, a_hp] = butter(order_hp, cutoff_hp/(SampleRate/2), 'high');

matlab_lp = filter(b_lp, a_lp, data);
matlab_hp = filter(b_hp, a_hp, data);

%% Visualization

t = 0:1/SampleRate:nsamples/SampleRate - 1/SampleRate;

figure;
subplot(3, 2, [1 3]);
hold on; 
plot(t, rtfilter_lp(:, channelId), 'b', 'LineWidth', 1); 
plot(t, matlab_lp(:, channelId), 'r'); 
plot(t, data(:, channelId), 'k'); 
hold off; 
grid on;
xlabel('time [s]');
ylabel('amplitude [uV]');
legend('rtfilter', 'matlab', 'data');
title(['Low-pass | order= ' num2str(order_lp) ', cutoff= ' num2str(cutoff_lp), ' | channel=' num2str(channelId)]);

subplot(3, 2, 5)
bar(t, abs(matlab_lp(:, channelId)- rtfilter_lp(:, channelId)));
grid on;
xlabel('time [s]');
ylabel('amplitude [uV]');
title('Difference')

subplot(3, 2, [2 4]);
hold on; 
plot(t, rtfilter_hp(:, channelId), 'b', 'LineWidth', 1);  
plot(t, matlab_hp(:, channelId), 'r'); 
plot(t, data(:, channelId), 'k'); 
hold off; 
grid on;
xlabel('time [s]');
ylabel('amplitude [uV]');
legend('rtfilter', 'matlab', 'data');
title(['High-pass | order= ' num2str(order_hp) ', cutoff= ' num2str(cutoff_hp), ' | channel=' num2str(channelId)]);

subplot(3, 2, 6)
bar(t, abs(matlab_hp(:, channelId)- rtfilter_hp(:, channelId)));
grid on;
xlabel('time [s]');
ylabel('amplitude [uV]');
title('Difference')

sgtitle('Evaluation butterworth simloop YAML configuration')
