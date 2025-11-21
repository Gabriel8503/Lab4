%% [text] Step 1: Create Trapezoidal Velocity Trajectories
clear all;
close all;
clc;

% [text] (1a) Calculate arc length of your imported trajectory
load("Total_Path.mat");    
load("LED_Path.mat")

xd = Total_Path(:,1);
yd = Total_Path(:,2);

% Compute arc length numerically
d = 0;
for i = 1:length(xd)-1
    d = d + sqrt( (xd(i+1)-xd(i))^2 + (yd(i+1)-yd(i))^2 );
end


% Desired final time
tfinal = 60;

% [text] Determine the average speed, c, of end effector over tfinal
c = d / tfinal;  


% [text] (1b) Forward-Euler method for alpha(t)
% Normalized trapezoidal velocity function g(t)
g = @(t, T, ta) (T/(T-ta)) * ( ...
      (t < ta) .* (t/ta) + ...
      (t >= ta & t <= (T - ta)) .* 1 + ...
      (t > (T - ta)) .* ((T - t)/ta) ...
);

T = 2*pi;          % alpha runs from 0 to T just like Lissajous case
dt = 1/100;        % step size
t = 0:dt:tfinal;   % simulation time

alpha = zeros(size(t));

% Precompute segment lengths for path parameterization (arc-length)
seg_lengths = [0; cumsum(sqrt(diff(xd).^2 + diff(yd).^2))];
L = seg_lengths(end);

% Acceleration time (CHANGEABLE)
ta = 5;

for k = 1:length(t)-1

    % Map alpha(k) → current arc-length s(k)
    s_cur = (alpha(k)/T) * L;

    % Find segment index
    idx = find(seg_lengths <= s_cur, 1, 'last');
    if idx >= length(seg_lengths)
        idx = length(seg_lengths)-1;
    end

    % Compute unit tangent direction at the segment
    dx = xd(idx+1) - xd(idx);
    dy = yd(idx+1) - yd(idx);
    seg_d = sqrt(dx^2 + dy^2);

    if seg_d > 0
        xdot = dx / seg_d;
        ydot = dy / seg_d;
    else
        xdot = 0; ydot = 0;
    end

    % Desired ds/dt = trapezoid, limited to 0.25 m/s
    ds_dt = min(c * g(t(k), tfinal, ta), 0.25);

    % Equation (7): alpha_dot = (ds/dt) * (T / L)
    alpha(k+1) = alpha(k) + dt * ds_dt * (T / L);

end

% [text] Plot alpha(t)
figure()
plot(t, alpha, 'LineWidth', 2);
grid on;
xlabel('time (s)');
ylabel('\alpha(t)');
title('Plot of \alpha(t)');
yline(T, 'k--', 'LineWidth', 2);
legend('\alpha(t)', 'T (period)', 'Location', 'southeast');

% [text] (1c) Generate retimed trajectory x[k], y[k]

% Allocate memory
Nalpha = length(alpha);
x = zeros(size(alpha));
y = zeros(size(alpha));
LED = zeros(size(alpha)); 

for k = 1:Nalpha
    s_k = (alpha(k)/T) * L;

    idx_led = find(seg_lengths <= s_k, 1, 'last');
    if isempty(idx_led)
        idx_led = 1;
    elseif idx_led >= length(seg_lengths)
        idx_led = length(seg_lengths) - 1;
    end
    % safe clamp if LED_Path shorter/longer
    idx_led = max(1, min(idx_led, length(LED_Path)));
    LED(k) = LED_Path(idx_led);
    
    % Find segment index for position reconstruction
    idx = find(seg_lengths <= s_k, 1, 'last');
    if isempty(idx)
        idx = 1;
    elseif idx >= length(seg_lengths)
        idx = length(seg_lengths) - 1;
    end

    ds = s_k - seg_lengths(idx);

    dx = xd(idx+1) - xd(idx);
    dy = yd(idx+1) - yd(idx);
    seg_d = sqrt(dx^2 + dy^2);

    if seg_d > 0
        ux = dx / seg_d;
        uy = dy / seg_d;
    else
        ux = 0; uy = 0;
    end

    x(k) = xd(idx) + ux * ds;
    y(k) = yd(idx) + uy * ds;
end

% Ensure final LED defined
if Nalpha >= 2
    LED(end) = LED(end-1);
end

% [text] Plot the speed profile v[k]
v = sqrt( diff(x).^2 + diff(y).^2 ) / dt;

figure()
plot(t(2:end), v, 'LineWidth', 3); hold on;
yline(c, 'k--', 'LineWidth', 2);        % average required velocity
yline(0.25, 'r--', 'LineWidth', 2);    % velocity limit
grid on;
xlabel('time (s)');
ylabel('velocity (m/s)');
title('Trajectory Velocity');
legend('velocity', 'average velocity', 'velocity limit', 'Location', 'south');

%%
%[text] ## Step 2: Forward Kinematics
%[text] (2c) Calculate T0
% these values were obtained from the URDF directly
L1 = 0.2435;
L2 = 0.2132;
W1 = 0.1311;
W2 = 0.0921;
H1 = 0.1519;
H2 = 0.0854;

% home position of end effector
M = [-1 0 0 L1+L2;
    0 0 1 W1+W2;
    0 1 0 H1-H2;
    0 0 0 1];

% screw axes
S1 = [0 0 1 0 0 0]';
S2 = [0 1 0 -H1 0 0]';
S3 = [0 1 0 -H1 0 L1]';
S4 = [0 1 0 -H1 0 L1+L2]';
S5 = [0 0 -1 -W1 L1+L2 0]';
S6 = [0 1 0 H2-H1 0 L1+L2]';
S = [S1 S2 S3 S4 S5 S6];

% body screw axes
B1 = ECE569_Adjoint(M)\S1;
B2 = ECE569_Adjoint(M)\S2;
B3 = ECE569_Adjoint(M)\S3;
B4 = ECE569_Adjoint(M)\S4;
B5 = ECE569_Adjoint(M)\S5;
B6 = ECE569_Adjoint(M)\S6;
B = [B1 B2 B3 B4 B5 B6];

% joint angles
theta0 = [-1.6800   -1.4018   -1.8127   -2.9937   -0.8857   -0.0696]';

% calculate the 4x4 matrix representing the transition
% from end effector frame {b} to the base frame {s} at t=0: Tsb(0)

T0_space = ECE569_FKinSpace(M,S,theta0)
T0_body = ECE569_FKinBody(M,B,theta0)
T0_space-T0_body
T0 = T0_body;
%[text] Calculate Tsd at every time step.
% Calculate Tsd(t) for t=0 to t=tfinal
% Tsd(t) = T0 * Td(t)
N = length(x);
Tsd = zeros(4,4,N);
for i = 1:N
    R  = eye(3);
    p  = [x(i); y(i); 0];
    Td = [R,p; zeros(1,3),1];
    Tsd(:,:,i) = T0 * Td;
end
%%
%[text] (2d) Plot (x,y,z) in the s frame
xs = Tsd(1,4,:);
ys = Tsd(2,4,:);
zs = Tsd(3,4,:);
figure();
plot3(xs(:), ys(:), zs(:), 'LineWidth', 1)
title('Trajectory \{s\} frame')
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
hold on
plot3(xs(1),ys(1),zs(1),'go','MarkerSize',10,'LineWidth',2)
plot3(xs(end),ys(end),zs(end),'rx','MarkerSize',10,'LineWidth',2)
legend('Trajectory', 'Start', 'End')
grid on
hold off
%%
%[text] ## Step 3: Inverse Kinematics
initialguess = theta0;
Td = T0;

% you need to implement IKinBody
[thetaSol, success] = ECE569_IKinBody(B,M,Td,theta0,1e-6,1e-6);
if (~success)
    close(f);
    error('Error. \nCould not perform IK at index %d.',1)
end
%%
%[text] (3c) Perform IK at each time step
thetaAll = zeros(6,N);
thetaAll(:,1) = theta0;

% you can comment out the waitbar functions if they aren't working
% (sometimes they don't work with .mlx files)
% If the code gets stuck here, you will need to restart MATLAB
f = waitbar(0,['Inverse Kinematics (1/',num2str(N),') complete.']);

for i=2:N
    initialguess = thetaAll(:,i-1);

    [thetaSol, success] = ECE569_IKinBody(B,M,Tsd(:,:,i),initialguess,1e-6,1e-6);
    if (~success)
        close(f);
        error('Error. \nCould not perform IK at index %d.',i)
    end
    thetaAll(:,i) = thetaSol;
    waitbar(i/N,f,['Inverse Kinematics (',num2str(i),'/',num2str(N),') complete.']);
end
close(f);
%%
%[text] (3c) Verify that the joint angles don't change very much
dj = diff(thetaAll');
figure();
plot(t(1:end-1), dj)
title('First Order Difference in Joint Angles')
legend('J1','J2','J3','J4','J5','J6','Location','northeastoutside')
grid on
xlabel('time (s)')
ylabel('first order difference')
%%
%[text] (3d) Verify that the joints we found actually trace out our trajectory (forward kinematics)
actualTsd = zeros(4,4,N);
for i=1:N
    actualTsd(:,:,i) = T0*ECE569_FKinBody(M,B,thetaAll(:,i));
end

xs = actualTsd(1,4,:);
ys = actualTsd(2,4,:);
zs = actualTsd(3,4,:);
figure();
plot3(xs(:), ys(:), zs(:), 'LineWidth', 1)
title('Verified Trajectory \{s\} frame')
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
hold on
plot3(xs(1),ys(1),zs(1),'go','MarkerSize',10,'LineWidth',2)
plot3(xs(end),ys(end),zs(end),'rx','MarkerSize',10,'LineWidth',2)
legend('Trajectory', 'Start', 'End')
grid on
hold off
%%
%[text] (3e) Verify that the end effector does not enter a kinematic singularity, by plotting the determinant of your body jacobian
body_dets = zeros(N,1);
for i=1:N
    body_dets(i) = det(ECE569_JacobianBody(B,thetaAll(:,i)));
end
figure();
plot(t, body_dets)
title('Manipulability')
grid on
xlabel('time (s)')
ylabel('det of J_B')
%%
%[text] (3f) Save to CSV File
% you can play with turning the LEDs on and off
led = ones(N,1);

% save to the CSV file
data = [t' thetaAll' LED(:)];

% TODO: change the csv filename to your purdue ID
writematrix(data, 'gturak_bonus.csv')

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":47.9}
%---
