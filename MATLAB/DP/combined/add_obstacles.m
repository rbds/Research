function [ costs, P_tr, obst, n_rows, n_cols ] = add_obstacles(env )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% close all

if strcmp(env, 'pipeline')
    %use pipeline simulation. 
    ha = axes('units','normalized','position',[0 0 1 1]);
    uistack(ha,'bottom');

    I=imread('pipeline_map.png');
    hi = imagesc(I);
    colormap gray

    set(ha,'handlevisibility','off','visible','off')
   
    n_rows = 50; % must be at least 2x2.
    n_cols = 20;
    axes('position',[0 0 .99 .99])
    axis([0 n_rows 0 n_cols])
    axis off
%     axis equal
    
    obst = zeros(10,3);    
    costs = max(.1,abs(0.5 + randn(n_rows,n_cols).*0.2.*ones(n_rows, n_cols)));
%     costs = 0.2.*ones(n_rows, n_cols);
%     P_tr = 0.95*ones(n_rows, n_cols);
    P_tr = min(1,.9 + 0.5.*abs(randn(n_rows, n_cols)));   %*ones(n_rows, n_cols);
    
%     costs(1:125,105:117) = .5;
    
    
elseif strcmp(env, 'field')
        %use pipeline simulation. 
    ha = axes('units','normalized','position',[0 0 1 1]);
    uistack(ha,'bottom');

    I=imread('pipeline_map.png');
    hi = imagesc(I);
    colormap gray

    set(ha,'handlevisibility','off','visible','off')
   
    n_rows = 50; % must be at least 2x2.
    n_cols = 50;
    axes('position',[0 0 .99 .99])
    axis([0 n_rows 0 n_cols])
    axis off
    
    obst = zeros(10,3);    
    costs = 0.2*ones(n_rows, n_cols);
    P_tr = 0.99*ones(n_rows, n_cols);
        
    costs(17:40,8:14) = .5;
    costs(27:38, 14:25) = .6;
    costs(6:32, 25:32) = .4;
    costs(10:24, 32:45) = .6;

else
    %sample environment
    ha = axes('units','normalized','position',[0 0 1 1]);
    uistack(ha,'bottom');

    I=imread('sample_map.png');
    hi = imagesc(I);
    colormap gray

    set(ha,'handlevisibility','off','visible','off')

    axes('position',[0 0 .99 .99])
    axis off
    axis equal
    n_rows = 15; % must be at least 2x2.
    n_cols = 15;
    
    costs =[01 .1 .1 01 01 01 .6 .6 01 01 01 01 01 01 01;
            01 .1 .1 01 01 .3 .7 .6 01 01 01 01 .3 01 01;
            01 .1 .1 01 01 .8 .8 .8 .9 01 01 .3 .6 .5 .4;
            01 01 .1 .1 01 01 .9 .9 01 01 01 .3 .5 .5 .5;
            01 01 01 .1 .1 01 01 01 01 01 01 01 01 .3 01;
            01 01 01 01 .1 .1 .1 01 01 01 01 01 .9 .5 01;
            01 .3 .3 .3 .3 01 .1 .1 01 01 01 .9 .9 .9 .5;
            01 .3 .4 .6 .6 .3 .3 .1 01 01 01 .7 .9 .9 .9;
            01 .3 .3 .7 .7 .3 .3 .3 .1 01 01 01 .8 .9 .9;
            01 01 01 .6 .6 .5 01 01 .1 01 01 01 01 .5 .8;
            01 01 01 01 01 01 01 .1 .1 01 01 01 01 .5 .7;
            01 01 01 01 01 01 01 .1 01 01 01 01 01 01 01;
            .1 .1 .1 .1 .1 .1 .1 01 01 01 01 01 .6 .4 01;
            01 01 01 01 01 01 01 01 01 01 01 .3 .6 .6 01;
            01 01 01 01 01 01 01 01 01 01 01 01 01 .4 01];

    costs = flipud(costs);

    % road low P_tr (should avoid)
    P_tr=sqrt([.99 .50 .50 .99 .99 .99 .75 .75 .99 .99 .99 .99 .99 .99 .99;
               .99 .50 .50 .99 .99 .75 .75 .75 .85 .99 .99 .99 .85 .99 .99;
               .99 .50 .50 .90 .99 .99 .75 .65 .75 .99 .99 .80 .75 .80 .95;
               .99 .99 .65 .50 .50 .99 .99 .99 .99 .99 .99 .80 .75 .75 .75;
               .99 .99 .99 .85 .50 .99 .99 .99 .75 .99 .99 .99 .90 .80 .99;
               .99 .99 .99 .75 .50 .50 .75 .50 .99 .99 .99 .99 .99 .95 .99;
               .99 .99 .99 .90 .99 .50 .50 .90 .99 .99 .99 .99 .99 .99 .99;
               .99 .99 .99 .99 .99 .75 .75 .50 .75 .99 .99 .99 .99 .99 .99;
               .99 .99 .99 .90 .99 .75 .99 .50 .50 .99 .99 .99 .99 .99 .99;
               .99 .99 .75 .75 .75 .99 .99 .99 .50 .99 .99 .99 .99 .99 .99;
               .99 .99 .99 .75 .75 .99 .99 .50 .50 .99 .99 .99 .99 .99 .99;
               .99 .99 .99 .99 .99 .99 .50 .50 .99 .99 .99 .99 .99 .80 .99;
               .50 .50 .50 .50 .50 .50 .50 .99 .99 .99 .99 .99 .75 .75 .99;
               .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .75 .75;
               .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .75 .75]);

    P_tr = flipud(P_tr);   

    r = 0.25; %radius of trees
    points = [2.2,2.5;
              4, 6; 
              4 6.75;
              4, 7.25;
              4.25, 7.75;
              3.5, 6.5;
              3.5, 7.25;
              3.75, 7.75;
              4.5 6.25;
              4.5 6.75;
              4.75 7.25;
              4 6.5;
              5 7;
              5.25 7.25;
              7.5 13.5; 
              7.5 12.75;
              7.5 14.5;
              6.5 13;
              6.5 14;
              7 13;
              7 14;
              12.5 2.5;
              13 2.5;
              13 3;
              13.5 2.5;
              13.5 1.75;
              13.5 3;
              13.25 3.75;
              14 1.25;
              14 2;
              12 12.5;
              12.5 13; 
              12.5 12;
              13.25 12.25;
              13 13;
              13 13.5;
              13.75 13;
              13.25 13.75;
              14 11.5; 
              14.25 13;
              14.25 12.25;
              15 12;
              14.75 13;
              9, 4;
              9 4.5;
              9.3 4.4;
              8.9 3.8]; %centers of trees

    obst = zeros(length(points),3);       
    for ii = 1:length(points)
        obst(ii,:) = [points(ii,:), r];
    end

end


end

