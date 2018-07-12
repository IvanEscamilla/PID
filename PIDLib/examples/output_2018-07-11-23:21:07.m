Array=csvread('output_2018-07-11-23:21:07.csv');
col1 = Array(:, 1);
col2 = Array(:, 2);
col3 = Array(:, 3);
col4 = Array(:, 4);
plot(col1, col2, 'r', col1, col3, 'g',col1, col4, 'b');
title('output_2018-07-11-23:21:07.csv');
xlabel({'Time','(in ms)'});
ylabel('Time');
grid on
