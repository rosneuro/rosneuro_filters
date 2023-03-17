clearvars; 

filein = './test/input.csv';
data = readmatrix(filein);

%% rosneuro filtered data
file_rtfilter_lp = './test/butterworth_simloop_outlp.csv';
file_rtfilter_hp = './test/butterworth_simloop_outhp.csv';
file_rtfilter_bp = './test/butterworth_simloop_outbp.csv';

rtfilter_lp = readmatrix(file_rtfilter_lp);
rtfilter_hp = readmatrix(file_rtfilter_hp);
rtfilter_bp = readmatrix(file_rtfilter_bp);

%% MATLAB filtered data
SampleRate = 512;
channelId  = 2;

order_lp  = 4;
order_hp  = 4;  
cutoff_lp = 10;
cutoff_hp = 1;

[b_lp, a_lp] = butter(order_lp, cutoff_lp/(SampleRate/2), 'low');
[b_hp, a_hp] = butter(order_hp, cutoff_hp/(SampleRate/2), 'high');

matlab_lp = filter(b_lp, a_lp, data);
matlab_hp = filter(b_hp, a_hp, data);
matlab_bp = filter(b_hp, a_hp, matlab_lp);

%% Visualization
figure;
subplot(1, 3, 1);
hold on; 
plot(rtfilter_lp(:, channelId), 'b', 'LineWidth', 1); 
plot(matlab_lp(:, channelId), 'r'); 
plot(data(:, channelId), 'k'); 
hold off; 
legend('rtfilter', 'matlab', 'data');
title(['Low-pass | order= ' num2str(order_lp) ', cutoff= ' num2str(cutoff_lp)]);

subplot(1, 3, 2);
hold on; 
plot(rtfilter_hp(:, channelId), 'b', 'LineWidth', 1);  
plot(matlab_hp(:, channelId), 'r'); 
plot(data(:, channelId), 'k'); 
hold off; 
legend('rtfilter', 'matlab', 'data');
title(['High-pass | order= ' num2str(order_hp) ', cutoff= ' num2str(cutoff_hp)]);

subplot(1, 3, 3);
hold on; 
plot(rtfilter_bp(:, channelId), 'b', 'LineWidth', 1);  
plot(matlab_bp(:, channelId), 'r'); 
plot(data(:, channelId), 'k'); 
hold off; 
legend('rtfilter', 'matlab', 'data');
title(['Band-pass | order= [' num2str(order_lp) ' ' num2str(order_hp) '], cutoff= [' num2str(cutoff_lp) ' ' num2str(cutoff_hp) ']']);


