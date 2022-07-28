%%%
%   Title:      Reduced order observer sim for pitch dyanmics
%   Authour:    Orkun Ozkan (Control UROP Group)
%   Date:       20/07/2022
%   Aim:        One is given a system. System has an unobservable state.
%               Aim is to produce reliable estimator for said state.
%   Ref:        https://www.youtube.com/watch?v=L_NN4lSxyJU
%%

%%  Definitions
xi    = 0.084;
omega = 3.44;
tao   = 0.03;
mu    = 358.5;

%%  System state space representation
A11 = [0, 1; -(omega)^2, -2*xi*omega];
A12 = [0, 0; mu, 1];
A21 = [0, 0; 0, 0];
A22 = [-1/tao, 0; 0, 0];
A   = [A11, A12; A21, A22]

B1  = [0; 0];
B2  = [1/tao; 0];
B   = [B1; B2]

C1  = [1, 0; 0, 1];
C2  = [0, 0; 0, 0];
C   = [C1, C2]

%%  Determining L via pole placement
p = [-10, -9];
L = place(A22', (C1*A12)', p)'

%%  Determining reduced observers state space matrices
F = A22 - L*A12

H = F*L + A21 - L*A11

G = B2 - L*B1

eig(F)      % should be same as root of L, aka -100