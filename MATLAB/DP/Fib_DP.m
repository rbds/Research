%Fibonacci DP
function [Fn] = Fib_DP(n)
f = zeros(1,n);
f(1) = 1;
f(2) = 1;
if n>2
    for k=3:n
       f(k) = f(k-1) + f(k-2);
    end
end
Fn = f(n);

end