close all
clear all
clc

%% Data
charge10 = [11.5526864 10.0931453 7.9324151	8.5310821 8.6150987];
charge10 = [charge10 9.9238869 7.7016635 10.0891706	10.1583958 10.4692493];
charge10 = [charge10 11.3084214 9.5489728 8.3539128 10.6535072 10.0318237];
charge10 = [charge10 13.5662058	10.6770811 8.7706028 9.6637085	12.7630808];
charge10 = [charge10 13.8564879	9.7636577 9.1883664	12.8712061	9.2367736];

charge20 = [30.222804	23.9626144	24.5493172	23.9517265	24.3574228];
charge20 = [charge20 29.5093195	30.296065	24.0503228	20.5554796	20.8132757];
charge20 = [charge20 23.570738	27.0106993	23.183332	18.6004794	19.59446];
charge20 = [charge20 29.643098	28.4429191	24.2567321	26.8585328	22.4027332];
charge20 = [charge20 29.2641021	24.2075009	23.8930573	21.7971516	30.5425231];

charge30 = [26.7537631	29.3615659	26.4316573	32.7409815	35.2697119];
charge30 = [charge30 33.5293476	33.0904541	35.8078262	40.7671422	37.2882354];
charge30 = [charge30 39.6917387	30.8196607	32.2427608	40.0850827	34.6501597];
charge30 = [charge30 29.5439499	31.0888252	37.4411117	41.8562389	27.2611678];
charge30 = [charge30 27.9925283	28.4305086	38.7950627	27.6839009	38.1653594];

%% Find mean and error

mean10 = mean(charge10);
mean20 = mean(charge20);
mean30 = mean(charge30);

stdev10 = std(charge10);
stdev20 = std(charge20);
stdev30 = std(charge30);

%% Plot
figure
hold on
errorbar(10,mean10,stdev10,'rx')
errorbar(20,mean20,stdev20,'rx')
errorbar(30,mean30,stdev30,'rx')
ymin = min(charge10);
ymax = max(charge30);
axis([0 40 ymin ymax])
title('Average charge per length of drive','FontSize',14)
xlabel('Distance (in)','FontSize',12)
ylabel('Charge (Coulombs)','FontSize',12)







