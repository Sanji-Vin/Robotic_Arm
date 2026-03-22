clc; clear; close all;


L1 = 1.22917;
L2 = 0.35;
L3 = 0.46471;
L4 = 0.20286;
Lg = 0.12458;


rotz = @(t)[ cos(t) -sin(t) 0;
             sin(t)  cos(t) 0;
             0       0      1];
roty = @(t)[ cos(t) 0 sin(t);
             0      1 0;
            -sin(t) 0 cos(t)];


fk = @(q) compute_fk(q, L1, L2, L3, L4, Lg, rotz, roty);


pick_position  = [ 0.5;  0.3;  1.2];
place_position = [-0.4; -0.2;  1.0];
home_position  = [ 0.0;  0.5;  1.5];   

q_start = [0 0 0 0];
q_pick  = inverse_kinematics(pick_position,  fk, q_start);
q_place = inverse_kinematics(place_position, fk, q_pick);
q_home  = inverse_kinematics(home_position,  fk, q_place);

fprintf('q_start : [%.4f  %.4f  %.4f  %.4f]\n', q_start);
fprintf('q_pick  : [%.4f  %.4f  %.4f  %.4f]\n', q_pick);
fprintf('q_place : [%.4f  %.4f  %.4f  %.4f]\n', q_place);
fprintf('q_home  : [%.4f  %.4f  %.4f  %.4f]\n', q_home);


wp_q = {q_start; q_pick; q_place; q_home};
wp_p = {[0;0;0]; pick_position; place_position; home_position};
wp_labels = {'Back to start'; 'Moving to pick'; 'Moving to place'; 'Returning home'};
N = numel(wp_q);


hFig = figure('Name','4-DOF Arm','Color','w');
hAx  = axes('Parent', hFig);
hold(hAx, 'on');
grid(hAx, 'on');
xlabel(hAx,'X'); ylabel(hAx,'Y'); zlabel(hAx,'Z');
view(hAx, 135, 25);
axis(hAx, 'vis3d');
xlim(hAx,[-1 1]);
ylim(hAx,[-1 1]);
zlim(hAx,[ 0 2]);

hLink   = plot3(hAx, nan, nan, nan, '-o', ...
                'LineWidth', 3, 'MarkerSize', 6, ...
                'Color', [0.2 0.4 0.8], ...
                'MarkerFaceColor', [0.2 0.4 0.8]);
hEE     = scatter3(hAx, nan, nan, nan, 100, 'r', 'filled');
hTarget = scatter3(hAx, nan, nan, nan, 120, 'g', 'filled', 'Marker','pentagram');


while ishandle(hFig)
    for i = 1:N
        if ~ishandle(hFig), break; end

        i_from = i;
        i_to   = mod(i, N) + 1;  

        q_from = wp_q{i_from};
        q_to   = wp_q{i_to};
        p_to   = wp_p{i_to};

        
        set(hTarget, 'XData', p_to(1), 'YData', p_to(2), 'ZData', p_to(3));
        title(hAx, wp_labels{i_to});

       
        for t = linspace(0, 1, 60)
            if ~ishandle(hFig), break; end
            s = 3*t^2 - 2*t^3;
            q = q_from + s*(q_to - q_from);
            update_robot(fk(q), hLink, hEE);
        end

        pause(0.4);   
    end
end



function q_sol = inverse_kinematics(target, fk, q_init)
    opts = optimoptions('fminunc', ...
                        'Display',             'off', ...
                        'Algorithm',           'quasi-newton', ...
                        'MaxIterations',       5000, ...
                        'OptimalityTolerance', 1e-9, ...
                        'StepTolerance',       1e-9);
    q_sol = fminunc(@(q) cost_function(q, target, fk), q_init, opts);
end

function err = cost_function(q, target, fk)
    P  = fk(q);
    ee = P(:, end);
    err = norm(ee - target)^2;
end

function P = compute_fk(q, L1, L2, L3, L4, Lg, rotz, roty)
    P0 = [0; 0; 0];
    P1 = [0; 0; L1];

    T  = rotz(q(1)) * roty(q(2));
    P2 = P1 + T * [L2; 0; 0];

    T  = T * roty(q(3));
    P3 = P2 + T * [L3; 0; 0];

    T  = T * roty(q(4));
    P4 = P3 + T * [L4; 0; 0];

    Pg = P4 + T * [Lg; 0; 0];

    P  = [P0, P1, P2, P3, P4, Pg];
end

function update_robot(P, hLink, hEE)
    set(hLink, 'XData', P(1,:), 'YData', P(2,:), 'ZData', P(3,:));
    set(hEE,   'XData', P(1,end), 'YData', P(2,end), 'ZData', P(3,end));
    drawnow limitrate;
end