%put constant values in this file%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%You NEED these constants%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
c1=8;%link 1 friction coeffecient
c2=8;%link 2 friction coeffecient
l1=15/34; %link 1 length
l2=15/34; %link 2 length

% c1=6;%link 1 friction coeffecient
% c2=6;%link 2 friction coeffecient
% l1=15/56; %link 1 length
% l2=15/56; %link 2 length

% c1=4;%link 1 friction coeffecient
% c2=4;%link 2 friction coeffecient
% l1=0.15; %link 1 length
% l2=0.15; %link 2 length

m1=0.375;%link 1 mass
m2=0.375;%link 2 mass
g=3.7;%acceleration due to gravity m/s^2 on mars

my_desX=[0.1 0.2 0.2 0.1 0.1];
my_desY=[0.2 0.2 0.1 0.1 0.2];
syms my_alpha my_beta
for my_i=1:length(my_desX)
my_len=sqrt(my_desX(my_i)^2+my_desY(my_i)^2);
my_ang=atan(my_desY(my_i)/my_desX(my_i));
my_eqns=[l1*sin(my_alpha)==l2*sin(my_beta),l1*cos(my_alpha)+l2*cos(my_beta)==my_len];
my_vars=[my_alpha my_beta];
[my_qdes1, my_qdes2]=solve(my_eqns,my_vars);
my_qdes1=double(max(my_qdes1));
my_qdes2=double(max(my_qdes2));
my_qdes2=my_qdes1+my_qdes2;
my_qdes1=my_ang-my_qdes1;
my_q1des(my_i)=my_qdes1;
my_q2des(my_i)=my_qdes2;
end

x_0=[my_q1des(1),0,my_q2des(1),0]';%x_0=[q1_0,q1dot_0,q2_0,q2dot_0] initial conditions for the robot
my_curPoint=1;
my_checkStable=0;
my_timer=0;
my_timer2=0;
my_startTimer=false;
my_energy=0;
my_complete=false;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Declare all your variables here, prefix with my_ %Feel Free to add to or remove these constants%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
my_time=0;
my_angle_vector=[0 0]';
my_state_estimate_vector=[0 0 0 0]';
my_some_variable_a=0;
my_some_variable_b=0;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%