function new_constants = performance_evaluation(simOut, performance_targets)

    x     = simOut.yout{1};
    y     = simOut.yout{2};
    z     = simOut.yout{3};
    phi   = simOut.yout{4};
    theta = simOut.yout{5};
    psi   = simOut.yout{6};
    p     = simOut.yout{12};
    q     = simOut.yout{13};
    r     = simOut.yout{14};
    u     = simOut.yout{15};
    v     = simOut.yout{16};
    w     = simOut.yout{17};
    
    
    heave_angle = simOut.yout{7};
    pitch_angle = simOut.yout{8};
    roll_angle   = simOut.yout{9};
    rpm_top     = simOut.yout{10};
    rpm_bot     = simOut.yout{11};

    targets     = simOut.yout{18};

    
    perf_metrics(x.time, x.values, target(1,1) )
    asda

    new_constants = [];

end




function metrics = perf_metrics(t, y, y_ref, tol)
    % PERF_METRICS  Compute time‐domain performance metrics
    %
    %   metrics = PERF_METRICS(t, y, y_ref)
    %   metrics = PERF_METRICS(t, y, y_ref, tol)
    %
    %   Inputs:
    %     t     = time vector (Nx1)
    %     y     = response vector (Nx1)
    %     y_ref = constant reference/target value (scalar)
    %     tol   = settling‐time tolerance (fraction, default 0.02 for ±2%)
    %
    %   Output (struct):
    %     metrics.Ts   = settling time [s]
    %     metrics.Mp   = maximum overshoot [%]
    %     metrics.ess  = static (steady‐state) error
    %     metrics.Tr   = rise time 10→90% [s]
    %     metrics.Tp   = peak time [s]
    %     metrics.IAE  = integral of absolute error

    if nargin<4 || isempty(tol)
        tol = 0.02;  % default ±2%
    end

    % ensure column vectors
    t = t(:);
    y = y(:);

    % steady‐state value: average of last 1% of samples
    N = numel(y);
    M = max(1,round(0.01*N));
    y_ss = mean( y(end-M+1:end) );

    % static error
    metrics.ess = y_ref - y_ss;

    % maximum overshoot (peak response above reference)
    [y_peak, idx_peak] = max(y);
    metrics.Mp = max(0, (y_peak - y_ref)/abs(y_ref) ) * 100;
    metrics.Tp = t(idx_peak);

    % rise time: first crossing of 0.1→0.9 of y_ref
    y10 = y_ref * 0.10;
    y90 = y_ref * 0.90;
    i10 = find(y>=y10,1,'first');
    i90 = find(y>=y90,1,'first');
    if isempty(i10) || isempty(i90)
        metrics.Tr = NaN;
    else
        metrics.Tr = t(i90) - t(i10);
    end

    % settling time: first time after which y stays within ±tol·|y_ref|
    err = abs(y - y_ref);
    band = tol * abs(y_ref);
    % find all indices where error > band
    i_out = find(err > band);
    if isempty(i_out)
        metrics.Ts = 0;      % never leaves band
    elseif i_out(end)==N
        metrics.Ts = NaN;    % still out at end of sim
    else
        metrics.Ts = t( i_out(end)+1 );
    end

    % integral of absolute error
    metrics.IAE = trapz(t, err);
end