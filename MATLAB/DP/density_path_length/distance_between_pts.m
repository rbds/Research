stds = 1:2:100;         % std_devs to test
thing = zeros(length(stds),1);  %storage
for kk = 1:length(stds) %for each standard deviation
    N = 1000;           %number of samples
    pts = stds(kk)*randn(N,2);  %generate samples from normal distribution
    % dists = zeros(factorial(N-1),1);
    dists =  [];    
    for ii = 1:N         %for each sample, calculate distance to all other samples (that hasn't already been calculated
       for jj = ii+1:N
           dists(end+1) = norm(pts(ii,:) - pts(jj,:));
       end
    end
   thing(kk) =  mean(dists); %store the mean distance between points
end

plot(stds, thing)

mean(stds'./thing) %find the slope of that line.