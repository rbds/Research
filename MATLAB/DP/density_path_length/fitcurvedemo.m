function [estimates, model] = fitcurvedemo(xdata, ydata, p0)
% Call fminsearch with a random starting point.
start_point = p0;
model = @expfun;
estimates = fminsearch(model, start_point, optimset('MaxFunEvals', 1e9));
% expfun accepts curve parameters as inputs, and outputs sse,
% the sum of squares error for A*exp(-lambda*xdata)-ydata,
% and the FittedCurve. FMINSEARCH only needs sse, but we want
% to plot the FittedCurve at the end.
    function [sse, FittedCurve] = expfun(params)
        A = params(2);
        lambda = params(3);
        C = params(1);
        FittedCurve = C + A .* exp(-lambda * xdata);
        ErrorVector = FittedCurve - ydata;
        sse = sum(ErrorVector .^ 2);
    end
end