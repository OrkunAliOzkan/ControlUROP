%%  Key variables
xi    = 0.084;
omega = 3.44;
tao   = 0.03;
mu    = 358.5;

%%  Defining Submatrices
    %2D
%{

A   = [0, 1; -(omega * omega), -2*xi*omega]
B   = [0;mu]
C   = [1, 0; 0, 1]
%}
    %   3D
A11 = [0, 1; -(omega * omega), -2*xi*omega]
A12 = [0; mu]
A21 = [0, 0]
A22 = [-1/tao]

B1  = [0; 0]
B2  = [mu]

C1  = [1, 0; 0, 1]
C2  = [0; 0]

A   = [A11, A12; A21, A22]
B   = [B1; B2]
C   = [C1, C2]

%%  Computing L for observer (3D+)
poles_L_3D = [-10];
%poles_L_4D = [-9, -10];
L = place(A22', A12', poles_L_3D)'

%%  Computing F, G, H for observer (3D+)
F = A22 - L*A12;
H = F*L - L*A11 + A21;
G = -L*B1 + B2;

%%  Computing K for controller
%poles_K_2D = [-200, -210];
poles_K_3D = [-200, -210, -250];
%poles_K_4D = [-200, -210, -220, -230];

%K_2D = place(A_2D, B_2D, p_K_2D)
K = place(A, B, poles_K_3D)