clear


x = zeros(4, 1000, 50);
%TO DO
%return covariances
%plot calculated cov. for every 100 timesteps

for i = 1:10
   state(:,:,i) = estimation(); 
end

Ey2 = mean(state(:,100,:).^2,3);
E_cross2 = mean(state(1,100,:).*state(2,100,:),3);
C = [Ey2(1) 0; 0 Ey2(2)];
means = mean(state(1:2,100,:).^2,3);
tt=linspace(0,2*pi,50)';
x = cos(tt); y=sin(tt);
ap = [x(:) y(:)]';
[v,d]=eig(C); 
d = 1 * sqrt(d); % convert variance to sdwidth*sd
bp = (v*d*ap) + repmat(means, 1, size(ap,2)); 
h = plot(bp(1,:), bp(2,:), 'c-');

