clear

data = csvread('data_10in1.csv', 1 , 0 , [1 0 411 5]);
data = [data; csvread('data_10in2.csv', 1 , 0 , [1 0 410 5])];
data = [data; csvread('data_10in2.csv', 1 , 0 , [1 0 408 5])];
data = [data; csvread('data_10in2.csv', 1 , 0 , [1 0 409 5])];
data = [data; csvread('data_10in2.csv', 1 , 0 , [1 0 409 5])];