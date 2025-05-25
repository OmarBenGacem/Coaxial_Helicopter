% Interactive 3D visualization with sliders controlling theta₁, θ₂, θ₃
function servo_swash_slider(s,n,r)
    %% Parameters
    r_value     = 1;
    theta        = [0; 0; 0];         % initial angles

    %% Precompute static geometry
    % symbolic s,n must be in workspace or defined above
    local_s = double(subs(s, r, r_value));  
    local_n = double(subs(n, r, r_value));  

    % in‐plane basis u (tangent) and v (cross)
    u = [ -local_s(2,:); local_s(1,:); zeros(1,3) ];
    u = u ./ vecnorm(u,2,1);
    v = cross(local_n, u, 1);

    %% Create figure & axes
    hFig = figure('Name','Servo Swashplate','NumberTitle','off','Position',[100 100 800 600]);
    hAx  = axes('Parent',hFig);
    hold(hAx,'on'); axis(hAx,'equal'); view(hAx,3); grid(hAx,'on');
    xlabel(hAx,'X'); ylabel(hAx,'Y'); zlabel(hAx,'Z');
    title(hAx,'Points, Normals, Planes, and Horn Vectors');

    %% Plot static elements: points, normals, planes
    scatter3(hAx, local_s(1,:), local_s(2,:), local_s(3,:), 100, 'k', 'filled');
    quiver3(hAx, local_s(1,:), local_s(2,:), local_s(3,:), ...
            local_n(1,:), local_n(2,:), local_n(3,:), 0.5, ...
            'LineWidth',2,'MaxHeadSize',0.5);

    colors = lines(3);
    L      = 1.5 * r_value;
    for i = 1:3
        Bi = null(local_n(:,i)');
        corners = [ ...
            local_s(:,i) +  L*Bi(:,1) +  L*Bi(:,2), ...
            local_s(:,i) +  L*Bi(:,1) -  L*Bi(:,2), ...
            local_s(:,i) -  L*Bi(:,1) -  L*Bi(:,2), ...
            local_s(:,i) -  L*Bi(:,1) +  L*Bi(:,2) ]';
        patch('Parent',hAx,'Vertices',corners,'Faces',[1 2 3 4], ...
              'FaceColor',   colors(i,:), ...
              'FaceAlpha',   0.3, ...
              'EdgeColor',   'none');
    end

    %% Initial dynamic plot: horn vectors & tips
    [delta, q] = computeDeltaQ(theta);
    hQuiv = quiver3(hAx, local_s(1,:), local_s(2,:), local_s(3,:), ...
                    delta(1,:), delta(2,:), delta(3,:), 0, ...
                    'LineWidth',2,'MaxHeadSize',0.5);
    hTip   = scatter3(hAx, q(1,:), q(2,:), q(3,:), 80, 'r', 'filled');

    %% Create sliders
    % normalized positions: [left bottom width height]
    sliderPos = [0.15 0.05 0.7 0.03;   % θ₁
                 0.15 0.01 0.7 0.03;   % θ₂
                 0.15 0.09 0.7 0.03];  % θ₃
    hSliders = gobjects(3,1);
    for i = 1:3
        hSliders(i) = uicontrol( ...
            'Parent',   hFig, ...
            'Style',    'slider', ...
            'Min',      -0.35, ...
            'Max',      0.35, ...
            'Value',    theta(i), ...
            'Units',    'normalized', ...
            'Position', sliderPos(i,:), ...
            'Callback', @sliderCallback );
        % optional: add a text label
        uicontrol('Parent', hFig, 'Style','text', ...
            'Units','normalized', ...
            'Position',[sliderPos(i,1)-0.10 sliderPos(i,2) 0.08 0.03], ...
            'String', sprintf('θ_%d',i));
    end

    %% Callback and helper
    function sliderCallback(~,~)
        % read slider values
        theta = [hSliders(1).Value; hSliders(2).Value; hSliders(3).Value];
        [delta, q] = computeDeltaQ(theta);
        % update horn‐vector arrows
        set(hQuiv, 'UData', delta(1,:), 'VData', delta(2,:), 'WData', delta(3,:));
        % update horn‐tip points
        set(hTip,  'XData', q(1,:),     'YData', q(2,:),     'ZData', q(3,:));
    end

    function [delta, q] = computeDeltaQ(th)
        % map 2D offsets into 3D for given theta = [θ1;θ2;θ3]
        servo2D = [ r_value*cos(th)'; r_value*sin(th)' ];  % 2×3
        delta   = u .* servo2D(1,:) + v .* servo2D(2,:);
        q       = local_s + delta;
    end
end
