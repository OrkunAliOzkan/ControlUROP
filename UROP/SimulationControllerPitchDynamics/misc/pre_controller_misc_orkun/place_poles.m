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
A12 = [0; mu];
A21 = [0, 0];
A22 = [-1/tao];
A   = [A11, A12; A21, A22]

B1  = [0; 0];
B2  = [1/tao];
B   = [B1; B2]

C1  = [1, 0; 0, 1];
C2  = [0; 0];
C   = [C1, C2]

%%  Determining L via pole placement
p = -100;
L = place(A22', (C1*A12)', p)'