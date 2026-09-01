% -------------------------------------------------------------------------
% Singularity-Free Vector Field Navigation: Full Architecture
% Features: Modulation Matrix, Convergent Wake Symmetry Breaking, 
%           1st-Order Distance Approximation, Tiered Safety Buffers
% -------------------------------------------------------------------------
clear; clc; close all;

% 1. Define Workspace and Grid
x_range = linspace(-5, 5, 40);
y_range = linspace(-4, 4, 35);
[X, Y] = meshgrid(x_range, y_range);

% 2. Initialize Fields
% Path-Following Field (Vp): Uniform flow from left to right
Vp_x = (1/sqrt(2)) .* ones(size(X));
Vp_y = (1/sqrt(2)) .* ones(size(Y));

% Preallocate Resultant Field (Vc)
Vc_x = zeros(size(X));
Vc_y = zeros(size(Y));

% 3. Obstacle & Safety Parameters
% Elliptical Obstacle Geometry (C2 Arbitrary Shape)
a = 1.2; % Semi-major axis
b = 2.0; % Semi-minor axis
xc = 0;  % X-center
yc = 0;  % Y-center

%modulation spatial effect
rho = 3;

% Safety Buffer Definition
r_m = 0.8; % Required Euclidean safety margin (e.g., drone radius + buffer)

% Symmetry Breaking Parameters
theta_max = pi/2;  % Maximum twist (90 degrees)
k_decay = 5;       % Sharpness of angular decay
spatial_decay = 10; % Sharpness of spatial decay outside safety boundary

% 4. Compute Vector Field
for i = 1:numel(X)
    
    % Current spatial coordinate
    px = X(i);
    py = Y(i);
    
    % Evaluate the implicit C2 shape function F(x) = 0
    % F < 0 (Inside), F = 0 (Boundary), F > 0 (Outside)
    F_val = ((px - xc)/a)^2 + ((py - yc)/b)^2 - 1;
    
    % Check True Physical Boundary (Crash limit)
    if F_val <= 0
        Vc_x(i) = NaN;
        Vc_y(i) = NaN;
        continue; % Stop calculating for this point
    end
    
    % Calculate Gradient of F to find normal vector
    gradF = [2*(px - xc)/a^2; 
             2*(py - yc)/b^2];
    norm_gradF = norm(gradF);
    
    % 1st-Order Euclidean Distance Approximation
    D_dist = F_val / norm_gradF; 
    
    % Map distance to the Safety Gamma Function
    % Gamma_safe = 1 exactly at the margin (r_m)
    Gamma_safe = D_dist / r_m; 
    
    % Define Local Basis Frame (E)
    n = gradF / norm_gradF;   % Normal vector
    t = [-n(2); n(1)];        % Tangent vector
    E = [n, t];
    
    % Input Vector Preparation
    Vp_current = [Vp_x(i); Vp_y(i)];
    Vp_hat = Vp_current / norm(Vp_current);
    
    % --- SYMMETRY BREAKING: CONVERGENT WAKE ---
    dot_prod = dot(n, Vp_hat); 
    cross_prod = n(1)*Vp_hat(2) - n(2)*Vp_hat(1);
    
    % Establish rotational polarity (diverge front, converge rear)
    s_dir = -sign(cross_prod);
    if s_dir == 0
        s_dir = 1; % Tie-breaker on exact centerlines
    end
    
    % Bounded twist angle with dynamic phase-shift (-sign(dot_prod))
    % Using max(Gamma_safe-1, 0) prevents decay from exploding inside the buffer
    theta = -sign(dot_prod) * s_dir * theta_max * ...
            exp(-k_decay * (1 - abs(dot_prod))^2) * ...
            exp(-spatial_decay * max(Gamma_safe - 1, 0));
        
    % Construct Rotation Matrix
    R_mat = [cos(theta), -sin(theta); 
             sin(theta),  cos(theta)];
         
    Vp_rotated = R_mat * Vp_current;
    
    % --- MODULATION MATRIX ---
    % Normal velocity (lambda1) zeroes out at Gamma_safe = 1, becomes negative if Gamma_safe < 1
    lambda1 = 1 - (1 / Gamma_safe)^rho; 
    lambda2 = 1 + (1 / Gamma_safe)^rho; 
    
    D = [lambda1, 0; 
         0, lambda2];
    
    % Transformation: Vc = E * D * E^-1 * Vp_rotated
    M = E * D * E';
    Vc = M * Vp_rotated;
    
    % Store Final Output
    Vc_x(i) = Vc(1);
    Vc_y(i) = Vc(2);
end

% 5. Visualization
figure;
hold on; grid on; axis equal;

% Plot True Physical Obstacle (Solid)
theta_plot = linspace(0, 2*pi, 100);
obs_x = xc + a*cos(theta_plot);
obs_y = yc + b*sin(theta_plot);
fill(obs_x, obs_y, [0.3 0.3 0.3], 'EdgeColor', 'none'); % Dark Grey carbon-fiber
plot(obs_x, obs_y, 'Color', '#D95319', 'LineWidth', 2);

% Plot the Safety Boundary (Dashed)
% Since it's a 1st order approximation, we'll plot the exact Minkowski expansion 
% for visual confirmation of the buffer zone.
safe_x = obs_x + r_m * (2*cos(theta_plot)/a^2) ./ sqrt((2*cos(theta_plot)/a).^2 + (2*sin(theta_plot)/b).^2);
safe_y = obs_y + r_m * (2*sin(theta_plot)/b^2) ./ sqrt((2*cos(theta_plot)/a).^2 + (2*sin(theta_plot)/b).^2);
plot(safe_x, safe_y, '--', 'Color', '#77AC30', 'LineWidth', 1.5);

% Plot Resultant Vector Field
vel_mag = sqrt(Vc_x.^2 + Vc_y.^2);
quiver(X, Y, Vc_x./vel_mag, Vc_y./vel_mag, 0.55, 'Color', '#EDB120', 'LineWidth', 1.2);

% Formatting
title('Singularity-Free Field: Convergent Wake & Safety Buffer', 'FontSize', 14, 'FontWeight', 'bold');
legend('Physical Obstacle', 'Boundary', 'Safety Margin (r_m)', 'Location', 'northeast');
xlabel('X (meters)', 'FontSize', 12);
ylabel('Y (meters)', 'FontSize', 12);
xlim([-5 5]); ylim([-4 4]);
hold off;