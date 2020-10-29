function history = runlsqcurvefit(f, param_vec0, xdata0, ydata)
    % History parameter
    history.x = [];
    history.residual = [];
    % LM
    options = optimoptions('lsqcurvefit','Algorithm','levenberg-marquardt','FiniteDifferenceType',...
    'central','Display','Iter','OutputFcn',@outfun);
    lb = [];
    ub = [];
    lsqcurvefit(f, param_vec0, xdata0, ydata,lb,ub,options);    
    function stop = outfun(x, optVal, state)
        stop = false;
        if strcmp(state,'iter')
            history.x = [history.x; x];
            history.residual = [history.residual; optVal.resnorm];
        end
    end
end