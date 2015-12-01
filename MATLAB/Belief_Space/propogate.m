function [ success, x_new] = propogate( e, v, x, constraint, obs )
%Takes in the path between two points and checks the chance constraint.
%first, draws the ellipse around each intermediate point. 
%then, check if any obstacles are within that ellipse.

for i=1:length(e)
    pts(:,i) = e(i).p; %#ok<AGROW>
end
%draw ellipses
for i= 1:length(pts)-1
    if (size(pts) >= [3, i] == [1,1]) % should probably remove this if statement, removes an occasional error.
        h = pts(1,i);
        k = pts(2,i);
        theta = pts(3,i);
        a = sqrt(v.sigma(1))+abs(1*randn(1));  %uncertainty in ellipse, plus some new uncertainty
        b = sqrt(v.sigma(2))+abs(1*randn(1));

        collision(i) = ellipse(h,k,theta,a,b, constraint, obs); %#ok<AGROW>
    end
end
x_new.sigma = [a;b];    %set uncertainty for new point.
x_new.x = x;
if isempty(collision(collision==1))
    success = 1;
else
    success = 0;
end
%generate 3 samples from a 0 mean normal distribution with std =.01: 
% r = 0 +.01*randn(3,1);
