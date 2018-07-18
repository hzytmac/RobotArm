syms my_q1 my_q2 my_q1d my_q2d my_q1dd my_q2dd my_T1 my_T2;
syms my_x1 my_x2 my_x3 my_x4 my_u1 my_u2;

my_e_prev=[0;0];
my_x_hat=x_0;
q=[x_0(1);x_0(3)];
my_q_op=q;

my_eqn1=my_T1==((m1*l1^2)/3+(m2*l2^2)/12+m2*(l1^2+l2^2/4+l1*l2*cos(my_q2)))*my_q1dd+((m2*l2^2)/3+(m2*l1*l2)/2*cos(my_q2))*my_q2dd-m2*l1*l2*sin(my_q2)*my_q1d*my_q2d-(m2*l1*l2*sin(my_q2))/2*my_q2d^2+(m1*l1/2+m2*l1)*g*cos(my_q1)+m2*l2/2*g*cos(my_q1+my_q2)+c1*my_q1d;
my_eqn2=my_T2==(m2*l2^2/3+m2*l1*l2/2*cos(my_q2))*my_q1dd+m2*l2^2/3*my_q2dd+m2*l1*l2*sin(my_q2)/2*my_q1d^2+m2*l2/2*g*cos(my_q1+my_q2)+c2*my_q2d;
my_eqn1=subs(my_eqn1,[my_q1,my_q1d,my_q2,my_q2d,my_T1,my_T2], [my_x1,my_x2,my_x3,my_x4, my_u1,my_u2]);
my_eqn2=subs(my_eqn2,[my_q1,my_q1d,my_q2,my_q2d,my_T1,my_T2], [my_x1,my_x2,my_x3,my_x4, my_u1,my_u2]);

my_f1=my_x2;
my_f3=my_x4;
my_eqns=[my_eqn1,my_eqn2];
my_vars=[my_q1dd my_q2dd];
[my_f2, my_f4]=solve(my_eqns,my_vars);

my_y=[my_x1; my_x3];

my_f2_func=matlabFunction(my_f2);
my_f4_func=matlabFunction(my_f4);
my_equil_eqns=[subs(my_f2,[my_x2,my_x4],[0,0])==0,subs(my_f4,[my_x2,my_x4],[0,0])==0];
my_vars=[my_u1 my_u2];
[my_u1_eq,my_u2_eq]=solve(my_equil_eqns,my_vars);
my_solve_u1=matlabFunction(my_u1_eq);
my_solve_u2=matlabFunction(my_u2_eq);

my_A=jacobian([my_f1, my_f2, my_f3, my_f4],[my_x1,my_x2, my_x3, my_x4]);
my_B=jacobian([my_f1, my_f2, my_f3, my_f4],[my_u1, my_u2]);
my_C=jacobian([my_y],[my_x1,my_x2, my_x3, my_x4]);
my_D=jacobian([my_y],[my_u1, my_u2]);
my_C=double(my_C);
my_D=double(my_D);

my_subsA=matlabFunction(my_A);
my_subsB=matlabFunction(my_B);

my_solve_u=(@(my_x1,my_x3)[my_solve_u1(my_x1,my_x3);my_solve_u2(my_x1,my_x3)]);

my_U_eq=my_solve_u(q(1),q(2));

my_A=my_subsA(my_U_eq(1),my_U_eq(2),q(1),0,q(2),0);
my_B=my_subsB(q(2));
my_A=double(my_A);
my_B=double(my_B);
tau_0=my_U_eq; %initial torque

my_count=0;
my_timer=0;