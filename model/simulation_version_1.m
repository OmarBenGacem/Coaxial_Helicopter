close all;
clear;
clc;
addpath("controllers\");
addpath("helicopter_functions\");
addpath("swash_plate\");


load_variables()
model = "helicopter";

runtime = 60;
Ts = 0.1;
t = 0 : Ts : runtime;
samples = length(t);
tune_parameters = 1;



if tune_parameters

    old_params = load("controllers/controller_constants.mat");
    fn         = fieldnames(old_params);

    % ensure order stays constant
    fn = sort(fn);
    theta = zeros(1,numel(fn));
    for k = 1:numel(fn)
        theta(k) = old_params.(fn{k});
    end

    best_params = fmincon(@function_handler, theta); 

    disp("Found Best Parameters:")
    best_params
    
    for k = 1:numel(fn)
        old_params.(fn{k}) = best_params(k);
    end
    
    % now dump all of those fields back into the .mat
    save("controllers/controller_constants.mat", "-struct", "old_params");







else






    load_system(model);
    display = true;
    
    z_is_ten = [ 2, 2, 10 ];
    U = repmat(z_is_ten, samples, 1);
    
    
    x_target = U(:,1);
    y_target = U(:,2);
    z_target = U(:,3);
    
    
    
    input_signal = timeseries(U, t, "Name", "input_signal"); % used for plotting only
    set_param(model, 'StopTime', num2str(runtime));
    
    
    % https://uk.mathworks.com/matlabcentral/answers/458511-setexternalinput-the-number-of-external-inputs-must-be-equal-to-the-number-of-root-level-input-port
    inports=createInputDataset(model);
    inports{1}=timeseries(x_target,t,'Name',inports{1}.Name);
    inports{2}=timeseries(y_target,t,'Name',inports{2}.Name);
    inports{3}=timeseries(z_target,t,'Name',inports{3}.Name);

    in = Simulink.SimulationInput(model);
    set_param(model,'LoadExternalInput','on');
    in = in.setExternalInput(inports);
    
    simOut = sim(in);
    
    
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
    roll_angle  = simOut.yout{9};
    rpm_top     = simOut.yout{10};
    rpm_bot     = simOut.yout{11};
    
    targets     = simOut.yout{18};
    
 
    
    if display
        figure;
        tiledlayout(3,2); 
        nexttile;
        plot(heave_angle.Values.Time, heave_angle.Values.Data, 'LineWidth', 1.5);
        ylabel('meters'); title('Heave Angle');
        grid on;
        
        
        nexttile;
        % plot(theta.Values.Time, theta.Values.Data, 'g', 'LineWidth', 1.5);
        ylabel('\theta'); title('IGNORE');
        grid on;
        nexttile;
        plot(roll_angle.Values.Time, rad2deg(roll_angle.Values.Data), 'LineWidth', 1.5);
        ylabel('Degrees'); title('Swash Plate Roll');
        grid on;
        rpm_top_plt = nexttile;
        plot(rpm_top.Values.Time, rpm_top.Values.Data, 'LineWidth', 1.5);
        ylabel('RPM'); xlabel('Time (s)'); title('RPM Top Rotor');
        grid on;
    
    
    
    
    
        nexttile;
        plot(pitch_angle.Values.Time, rad2deg(pitch_angle.Values.Data), 'r', 'LineWidth', 1.5);
        ylabel('Degrees'); title('Swash Plate Pitch');
        grid on;
    
        
        rpm_bot_plt = nexttile;
        plot(rpm_bot.Values.Time, rpm_bot.Values.Data, 'b', 'LineWidth', 1.5);
        ylabel('RPM'); xlabel('Time (s)'); title('RPM Bottom Rotor');
        grid on;
        
        linkaxes([rpm_top_plt, rpm_bot_plt], 'y');
        
        
    
    
    
        figure;
        tiledlayout(3,2); 
        nexttile;
        plot(p.Values.Time, rad2deg(p.Values.Data), 'LineWidth', 1.5);
        ylabel('Degrees/Second'); title('p');
        grid on;
    
        nexttile;
        plot(u.Values.Time, u.Values.Data, 'g', 'LineWidth', 1.5);
        ylabel('m/s'); title('u');
        grid on;
    
        nexttile;
        plot(q.Values.Time, rad2deg(q.Values.Data), 'r', 'LineWidth', 1.5);
        ylabel('Degrees/Second'); title('q');
        grid on;
    
    
        rpm_top_plt = nexttile;
        plot(v.Values.Time, v.Values.Data, 'LineWidth', 1.5);
        ylabel('m/s'); xlabel('Time (s)'); title('v');
        grid on;
    
    
        nexttile;
        plot(r.Values.Time, rad2deg(r.Values.Data), 'LineWidth', 1.5);
        ylabel('Degrees/Second'); title('r');
        grid on;
    
    
    
        rpm_bot_plt = nexttile;
        plot(w.Values.Time, w.Values.Data, 'b', 'LineWidth', 1.5);
        ylabel('m/s'); xlabel('Time (s)'); title('w');
        grid on;
    
    
    
        
        
            
        figure;
        tiledlayout(3,2); 
    
        xaxis = nexttile;
        plot(x.Values.Time, x.Values.Data, 'LineWidth', 1.5); hold on;
        plot(input_signal.Time, input_signal.Data(:,1), 'k--', 'LineWidth', 1);
        ylabel('x'); title('X Position');
        grid on;
        
      nexttile;
        plot(psi.Values.Time, rad2deg(psi.Values.Data), 'r', 'LineWidth', 1.5);
        ylabel('\psi (degrees)'); title('Roll');
        grid on;
    
        yaxis = nexttile;
        plot(y.Values.Time, y.Values.Data, 'LineWidth', 1.5); hold on;
        plot(input_signal.Time, input_signal.Data(:,2), 'k--', 'LineWidth', 1);
        ylabel('y'); title('Y Position');
        grid on;
        
        nexttile;
        plot(theta.Values.Time, rad2deg(theta.Values.Data), 'g', 'LineWidth', 1.5);
        ylabel('\theta (degrees)'); title('Pitch');
        grid on;
        
        zaxis = nexttile;
        plot(z.Values.Time, z.Values.Data, 'LineWidth', 1.5); hold on;
        plot(input_signal.Time, input_signal.Data(:,3), 'k--', 'LineWidth', 1);
        ylabel('z'); xlabel('Time (s)'); title('Z Position');
        grid on;
        
        nexttile;
        plot(phi.Values.Time, rad2deg(phi.Values.Data), 'b', 'LineWidth', 1.5);
        ylabel('\phi (degrees)'); xlabel('Time (s)'); title('Yaw');
        hold on;
        xLimits = xlim; % Get current x-axis limits
        % plot(xLimits, [rad2deg(0.2) rad2deg(0.2)], 'k--', 'LineWidth', 1.5);
        hold off;
        grid on;
    
        linkaxes([xaxis, yaxis, zaxis], 'y');
    end


end


function perf_metrics = function_handler(theta)
    % addpath("controllers\");
    % addpath("helicopter_functions\");
    % addpath("swash_plate\");
    % 
    
    load_variables()
    model = "helicopter";
    
    runtime = 60;
    Ts = 0.1;
    t = 0 : Ts : runtime;
    samples = length(t);
    tune_parameters = 1;
        performance_targets.settling_time = 1;
        
        load_system(model); 

        z_is_ten = [ 3, 3, 5 ];
        U = repmat(z_is_ten, samples, 1);
        x_target = U(:,1);
        y_target = U(:,2);
        z_target = U(:,3);
        set_param(model, 'StopTime', num2str(runtime));


        % https://uk.mathworks.com/matlabcentral/answers/458511-setexternalinput-the-number-of-external-inputs-must-be-equal-to-the-number-of-root-level-input-port
        inports=createInputDataset(model);
        inports{1}=timeseries(x_target,t,'Name',inports{1}.Name);
        inports{2}=timeseries(y_target,t,'Name',inports{2}.Name);
        inports{3}=timeseries(z_target,t,'Name',inports{3}.Name);

        in = Simulink.SimulationInput(model);
        set_param(model,'LoadExternalInput','on');
        in = in.setExternalInput(inports);

        simOut = sim(in);
        eval = performance_evaluation(simOut, performance_targets)
        asda
        perf_metrics = eval * eval'; 
end




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