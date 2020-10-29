function history = runlsqcurvefit(f, param_vec0, xdata0, ydata)
    % Description: wrapper function to run nonlinear least squares fit
    % 
    % Inputs:
    %   @f - function handle representing forward model
    %   @param_vec0 - initial guess of parameters
    %   @xdata0 - independent variables/input data (e.g. world coordinates)
    %   @ydata - dependent variables/output data (e.g. pixel coordinates)
    %
    % Output:
    %   history - optimization history
    %
    % History parameter
    history.x = [];
    history.residual = [];
    % LM
    options = optimoptions('lsqcurvefit','Algorithm','levenberg-marquardt','FiniteDifferenceType',...
    'central','Display','Iter','OutputFcn',@outfun);
    lb = [];
    ub = [];
    lsqcurvefit(f, param_vec0, xdata0, ydata,lb,ub,options);    
    % Auxiliary function to store optimization history
    function stop = outfun(x, optVal, state)
        stop = false;
        if strcmp(state,'iter')
            history.x = [history.x; x];
            history.residual = [history.residual; optVal.resnorm];
        end
    end
end