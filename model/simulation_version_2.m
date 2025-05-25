
% simulation  —  Tune helicopter controller gains to meet x/y/z specs.
%
%   best_theta = simulation()
%
%   Defines targets for settling time, max overshoot, and max velocity
%   in the x, y, and z axes, then uses fmincon to adjust your controller
%   parameters (theta) to drive the model as close as possible to those
%   specs.  All helper functions are nested below so they share the same
%   workspace (and you won’t get “cleared variable fn” errors).

    close all;  clear; clc;
    addpath("controllers");
    addpath("helicopter_functions");
    addpath("swash_plate");
    load("variables.mat")

    %% 1) Simulation / tuning settings
    model   = "helicopter";
    runtime = 60;             % [s]
    Ts      = 0.1;            % [s]
    t       = 0:Ts:runtime;   % time vector

    % Reference position [x_ref, y_ref, z_ref]
    performance_targets.ref = [3, 3, 5];

    % Desired settling time [s], overshoot [%], max velocity [m/s]
    performance_targets.Ts   = [10.0, 10.0, 10.0];
    performance_targets.Mp   = [0.5, 0.5, 0.5];
    performance_targets.Vmax = [2.0, 2.0, 1.0];

    %% 2) Load your existing controller gains into theta0
    S  = load("controllers/controller_constants.mat");
    fn = sort(fieldnames(S));
    theta0 = cellfun(@(f) S.(f), fn)';

    %% 3) Call fmincon
    opts = optimoptions("fmincon", ...
                        "Display","iter", ...
                        "TolX",1e-4, ...
                        "TolFun",1e-4);
    best_theta = fmincon(@objective_fun, theta0, [],[],[],[],[],[],[], opts);

    %% 4) Save tuned gains back to your .mat
    for k = 1:numel(fn)
        S.(fn{k}) = best_theta(k);
    end
    save("controllers/controller_constants.mat", "-struct", "S");
    fprintf("\nTuning complete. Best theta:\n");
    disp(best_theta);


    %% --------------------------------------------------------------------
    function J = objective_fun(theta)
    % Assign theta → base workspace, run sim, compute scalar cost J

        load_variables();  % reload any constants your controller needs

        % push each parameter into base so Simulink sees it
        for k = 1:numel(fn)
            assignin('base', fn{k}, theta(k));
        end

        simOut = simulate_model();
        J      = compute_cost(simOut);
    end


    %% --------------------------------------------------------------------
    function simOut = simulate_model()
    % Run the Simulink model with a constant [x_ref,y_ref,z_ref]

        U = repmat(performance_targets.ref, numel(t), 1);
        inports = createInputDataset(model);
        for j = 1:3
            inports{j} = timeseries(U(:,j), t, 'Name', inports{j}.Name);
        end

        in = Simulink.SimulationInput(model);
        set_param(model, 'LoadExternalInput','on', ...
                        'StopTime', num2str(runtime));
        in = in.setExternalInput(inports);

        simOut = sim(in);
    end


    %% --------------------------------------------------------------------
    function J = compute_cost(simOut)
    % For each axis (x=1,y=2,z=3) compute:
    %   • settling-time error
    %   • overshoot   error
    %   • max-velocity error
    % then sum the squared, normalized errors into one scalar J

        J = 0;
        for ax = 1:3
            sig = simOut.yout{ax}.Values;
            m   = perf_metrics(sig.Time, sig.Data, performance_targets.ref(ax));

            % finite-difference max velocity
            vel  = diff(sig.Data) / Ts;
            Vmax = max(abs(vel));

            % normalized errors
            eTs = (m.Ts   - performance_targets.Ts(ax))   / performance_targets.Ts(ax);
            eMp = (m.Mp   - performance_targets.Mp(ax))   / performance_targets.Mp(ax);
            eV  = (Vmax  - performance_targets.Vmax(ax)) / performance_targets.Vmax(ax);

            J = J + eTs^2 + eMp^2 + eV^2;
        end
    end


    %% --------------------------------------------------------------------
    function metrics = perf_metrics(t_, y_, y_ref, tol)
    % Compute step-response metrics:
    %   Ts  = settling time (to within ±tol·ref)
    %   Mp  = max overshoot (%)
    %   Tp  = time of peak
    %   Tr  = rise time (10→90%)
    %   IAE = ∫|error|

        if nargin<4 || isempty(tol)
            tol = 0.02;  % ±2%
        end
        t_ = t_(:);  y_ = y_(:);

        % steady-state
        N = numel(y_);
        M = max(1, round(0.01*N));
        y_ss = mean(y_(end-M+1:end));
        metrics.ess = y_ref - y_ss;

        % overshoot & peak time
        [y_peak, idx_peak] = max(y_);
        metrics.Mp = max(0,(y_peak - y_ref)/abs(y_ref))*100;
        metrics.Tp = t_(idx_peak);

        % rise time 10→90%
        y10 = 0.1*y_ref;  y90 = 0.9*y_ref;
        i10 = find(y_ >= y10, 1, 'first');
        i90 = find(y_ >= y90, 1, 'first');
        metrics.Tr = NaN;
        if ~isempty(i10) && ~isempty(i90)
            metrics.Tr = t_(i90) - t_(i10);
        end

        % settling time
        err  = abs(y_ - y_ref);
        band = tol * abs(y_ref);
        i_out = find(err > band);
        if isempty(i_out)
            metrics.Ts = 0;          % always within band
        elseif i_out(end) == N
            metrics.Ts = NaN;        % still out at end
        else
            metrics.Ts = t_(i_out(end)+1);
        end

        % integral of absolute error
        metrics.IAE = trapz(t_, err);
    end


