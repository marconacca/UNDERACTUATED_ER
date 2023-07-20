% Read data from CSV file
data = readtable('./plots/torque.csv');

% Extract x and y values from the table
x = data.X;
%y = data.Y;
y = data{:, 2:end};

% Plot the data
%plot(x, y);

figure;
hold on;
for i = 1:size(y, 2)
    plot(x, y(:, i));
end
hold off;

xlabel('X-axis');
ylabel('Y-axis');
title('Plotting Points');
legend(data.Properties.VariableNames(2:end));

grid on;