function [estimates, model] = fitcurve(xdata, ydata, p0)
% Call fminsearch with a random starting point.
start_point = p0;
model = @hyperbfun;
estimates = fminsearch(model, start_point, optimset('MaxFunEvals', 1e9));
% expfun accepts curve parameters as inputs, and outputs sse,
% the sum of squares error for A*exp(-lambda*xdata)-ydata,
% and the FittedCurve. FMINSEARCH only needs sse, but we want
% to plot the FittedCurve at the end.
    function [sse, FittedCurve] = hyperbfun(params)
        
        B = params(2);
        A = params(1);
        D = params(4);
        C = params(3);
        FittedCurve = A*xdata + B.*xdata.*ydata + C*ydata + D;
        ErrorVector = FittedCurve;
        sse = sum(ErrorVector .^ 2);
    end
end