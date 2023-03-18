clearvars; 

filein = './test/rawdata.csv';
data = readmatrix(filein);

%% rosneuro filtered data
file_rtfilter_bp = './test/butterworth_simloop_chain_bp.csv';

rtfilter_bp = readmatrix(file_rtfilter_bp);

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
matlab_bp = filter(b_hp, a_hp, matlab_lp);


%% Visualization

t = 0:1/SampleRate:nsamples/SampleRate - 1/SampleRate;

figure;
subplot(3, 1, [1 2]);
hold on; 
plot(t, rtfilter_bp(:, channelId), 'b', 'LineWidth', 1); 
plot(t, matlab_bp(:, channelId), 'r'); 
plot(t, data(:, channelId), 'k'); 
hold off; 
grid on;
xlabel('time [s]');
ylabel('amplitude [uV]');
legend('rtfilter', 'matlab', 'data');
title(['Band-pass | order= [' num2str(order_lp) ' ' num2str(order_hp) '], cutoff= [' num2str(cutoff_hp) ' ' num2str(cutoff_lp) '] | channel=' num2str(channelId)]);

subplot(3, 1, 3)
bar(t, abs(matlab_bp(:, channelId)- rtfilter_bp(:, channelId)));
grid on;
xlabel('time [s]');
ylabel('amplitude [uV]');
title('Difference')

sgtitle('Evaluation butterworth simloop FilterChain and YAML configuration')
