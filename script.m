clc
clear



%data
A_ES=[0 1; -1 -2];
A_IS=[0 1; 0 0];
A_OSC=[0 1; -1 -1];

A=A_ES;
B=[0; 1];
C=[1 -1];


%1° Open Loop Observer ----------------------------------------------------
A_OL = A;


%2° Closed Loop Observer --------------------------------------------------
Poles=[-12; -10];
L=(place(A_OL',C',Poles))'

A_CL = A_OL - L*C;

%%

%3° Unknown Input Observer ------------------------------------------------

%for static unknown input : d=step
Ad=0; Bd=0; Cd=1; %d_dot = 0

Ae=[A B*Cd; zeros(1,2) Ad];
Be=[B; 0];
Ce=[C 0];

PL=[-3; -2; -1];
Le=(place(Ae',Ce',PL))'

%visualizing params
x_gain = [1 0 0; 0 1 0];
d_gain = [0 0 1];

d0 = 1;
hat_0 = [0; 0; 0];


%%

%for dynamic unknown input : d=sin
T=1; w=2*pi/T;
Ad=[0 w; -w 0];
Bd = [0; 0];
Cd=[1 0];

Ae=[A B*Cd; zeros(2,2) Ad];
Be=[B; zeros(2,1)];
Ce=[C zeros(1,2)];

PL = [-1; -2; -3; -4];
Le=(place(Ae',Ce',PL))'

%visualizing params
x_gain = [1 0 0 0; 0 1 0 0];
d_gain = [0 0 1 0; 0 0 0 1];

d0 = [1; 1];
hat_0 = [0; 0; 0; 0]

%%

%4° PI Observer -----------------------------------------------------------
A=[0 1; -1 -2];
B=[1; 1];
C=[1 2];
E=[1; -3];


Ae=[A E; zeros(1,3)];
Be=[B; 0];
Ce=[C 0];

PLe=[-1; -2; -3];
Le=(place(Ae',Ce',PLe))'


%%

%5° State Estimation

%5.1° Filtre de KALMAN
G=[1; 1];
D=0;
H=0;
sys=ss(A,[B G],C,[D H]);

QW=0.01;
RV=0.01;
N=0;

display('>>Kalman Estimation params :')
[KEST, LK, ERR]=kalman(sys,QW,RV,N)


%5.2° Linear Quadratic Estimation (LQE)
D=0;
G2=eye(2);

QW2=0.01*eye(2);
RV2=0.01;

display('>>LQE params :')
[L_LQE P0 PL]=lqe(A,G2,C,QW2,RV2)
