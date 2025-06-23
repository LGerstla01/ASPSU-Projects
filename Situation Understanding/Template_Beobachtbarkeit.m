clear all

% Symbolic variables
syms pos_x pos_y theta v omega u1 u2 a b c d e dt real
x = [pos_x; pos_y; theta; v; omega];
u = [];

% Discrete nonlinear state equations
f = [pos_x + v*cos(theta)*dt;
     pos_y + v*sin(theta)*dt;
     theta + omega*dt;
     v;
     omega];

% Qutput equations (measurement equations)
h = [pos_x;
     pos_y];

% Jacobian matrices
A = jacobian(f, x);   % System matrix
B = jacobian(f, u);   % Input matrix
C = jacobian(h, x);   % Output matrix


% Observability matrix 
% General:
% O = [C;
%      C*A;
%      C*A^2;
%      ....
%      C*A^(n-1)]; % n is equal with the number of state variables

% with four state variables:
O = [C;
     C*A;
     C*A^2;
     C*A^3
     C*A^4];


% Parameter values
params = {a, b, c, d, e, dt};
values = {1, 1, 1, 1, 1, 0.1};

% Specific state x0
x0_vals = {12, 20, 2*pi, 0.9, 0.2};

% Substitute parameters and state into observability matrix
O_subs = subs(O, [pos_x, pos_y, theta, v, omega, params{:}], [x0_vals{:}, values{:}]);

% Compute rank of the observability matrix
rang_O = rank(O_subs);

% Output
disp('Observability matrix O at x0:');
disp(O_subs);
disp(['Rank of the observability matrix at x0: ', num2str(rang_O)]);

% Check observability
if rang_O == length(x)
    disp('✅ System is observable at this state.');
else
    disp('❌ System is NOT observable at this state.');
end

