clearvars; 

filein = './test/rawdata.csv';
data = readmatrix(filein);

%% rosneuro filtered data
file_car = './test/car_simloop_config.csv';
rosneuro_car = readmatrix(file_car);


%% MATLAB filtered data
SampleRate = 512;
nsamples   = size(data, 1);
channelId  = 2;

matlab_car = car(data);

%% Visualization

t = 0:1/SampleRate:nsamples/SampleRate - 1/SampleRate;

figure;
subplot(3, 1, [1 2]);
hold on; 
plot(t, rosneuro_car(:, channelId), 'b', 'LineWidth', 1); 
plot(t, matlab_car(:, channelId), 'r'); 
plot(t, data(:, channelId), 'k'); 
hold off; 
grid on;
xlabel('time [s]');
ylabel('amplitude [uV]');
legend('rosneuro', 'matlab', 'data');
title(['Car filter | channel=' num2str(channelId)]);

subplot(3, 1, 3)
bar(t, abs(matlab_car(:, channelId)- rosneuro_car(:, channelId)));
grid on;
xlabel('time [s]');
ylabel('amplitude [uV]');
title('Difference')

sgtitle('Evaluation CAR simloop')

%% Functions
function dataout = car(datain)
    icar = mean(datain, 2);
    dataout = datain - icar * ones(1, size(datain, 2));
end