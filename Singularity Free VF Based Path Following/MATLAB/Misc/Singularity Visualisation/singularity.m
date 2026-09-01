clc; clear; close all;

syms x y;
% fn = x^2 + y^2 - 100; region.x = [-15, 15]; region.y = [-15, 15];
% fn = (x/100)^2 + (y/50)^2 - 1; region.x = [-150, 150]; region.y = [-100, 100];
% fn = y - 100*cos(x/100); region.x = [-150, 150]; region.y = [-150, 150];
fn = x^2 - 4 * y^2 * (1 - y^2); region.x = [-2, 2]; region.y = [-1.2, 1.2];

% k = 400;
% fn = (x./k).^2 + (y./k).^2 ...
%     + 0.6*exp(-((x./k - 0.5).^2 + (y./k + 0.3).^2)) ...
%     + 0.4*exp(-((x./k + 0.7).^2 + (y./k - 0.6).^2)) ...
%     + 0.35*exp(-((x./k + 0.2).^2 + (y./k + 0.8).^2)) ...
%     + 0.25*exp(-((x./k - 0.9).^2 + (y./k - 0.1).^2)) ...
%     + 0.15*sin(3*x./k).*cos(2*y./k) ...
%     + 0.12*sin(5*y./k + 0.7) ...
%     + 0.10*cos(4*x./k - 1.1) ...
%     + 0.08*(x./k).*(y./k) - 1.8; region.x = [-600, 600]; region.y = [-600, 600];



[X, Y] = meshgrid(linspace(region.x(1), region.x(2), 100),...
    linspace(region.y(1), region.y(2), 100));

fm = matlabFunction(fn,"Vars", [x, y]);
Z = fm(X, Y);

%Vector Field (VF) Calculation
fnx = diff(fn, x);
fny = diff(fn, y);

fmx = matlabFunction(fnx, "Vars", [x, y]);
fmy = matlabFunction(fny, "Vars", [x, y]);
vx = fmx(X, Y); vy = fmy(X, Y);

%normalize
normvx = zeros(size(vx));
normvy = zeros(size(vy));
mask = sqrt(vx.^2 + vy.^2) > 0.1;
normvx(mask) = vx(mask)./sqrt(vx(mask).^2 + vy(mask).^2);
normvy(mask) = vy(mask)./sqrt(vx(mask).^2 + vy(mask).^2);

singularPts = [
    X(~mask)';
    Y(~mask)'
];

figure;
hold on; grid on;
surf(X, Y, Z, "FaceAlpha", 1);
shading interp;
colormap turbo; colorbar;

% plot3(curve(1, :), curve(2, :), zeros(1, size(curve, 2)), "LineWidth", 1.5, "Color", "#0099ff");
fimplicit(fn, [region.x, region.y], "LineWidth", 1.5, "Color", "#ff0055");


xlabel("x-axis");
ylabel("y-axis");
zlabel("r = f(x, y)");
title("Pseudo Distance [r = f(x, y)] Plot");

view(45, 30); 
hold off;


fig = uifigure('Position',[100 100 800 600]);
ax = uiaxes(fig,...
    'Position',[50 100 700 450]);
hold(ax, "on"); grid(ax, "on"); grid(ax, "minor");


p2 = quiver(ax, X, Y, normvx, normvy, "Color", "#ffdd00");
fimplicit(ax, fm, [region.x, region.y], "LineWidth", 1.5, "Color", "#0099ff");
p1 = scatter(ax, singularPts(1, :), singularPts(2, :), "Marker", "o", "SizeData", 80, ...
    "MarkerFaceColor", "#ff0055", "MarkerEdgeColor", "none");

xlabel(ax, "x-axis");
ylabel(ax, "y-axis");
title(ax, "vector Field");

hold(ax, "off");

lbl = uilabel(fig, ...
    "Position", [100, 65, 300, 30], ...
    "Text", "Singularity Tolerance");
sld = uislider(fig,...
    'Position',[100 50 300 3],...
    'Limits',[1e-4, 1],...
    'Value', 0.1);

sld.ValueChangedFcn = @(src,event) updatePlot(src,{p1, p2}, X, Y, vx, vy);

function updatePlot(src, p, X, Y, vx, vy)
    tol = src.Value;
    
    %normalize
    normvx = zeros(size(vx));
    normvy = zeros(size(vy));
    mask = sqrt(vx.^2 + vy.^2) > tol;
    normvx(mask) = vx(mask)./sqrt(vx(mask).^2 + vy(mask).^2);
    normvy(mask) = vy(mask)./sqrt(vx(mask).^2 + vy(mask).^2);
    
    singularPts = [
        X(~mask)';
        Y(~mask)'
        ];
    p{1}.XData = singularPts(1, :);
    p{1}.YData = singularPts(2, :);
    
    p{2}.UData = normvx;
    p{2}.VData = normvy;
end