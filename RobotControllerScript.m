my_xPos=l1*cos(qact(1))+l2*cos(qact(1)+qact(2));
my_yPos=l1*sin(qact(1))+l2*sin(qact(1)+qact(2));

my_xPosEst=l1*cos(my_x_hat(1))+l2*cos(my_x_hat(1)+my_x_hat(3));
my_yPosEst=l1*sin(my_x_hat(1))+l2*sin(my_x_hat(1)+my_x_hat(3));

if (my_xPos-my_desX(my_curPoint))^2+(my_yPos-my_desY(my_curPoint))^2<=0.004^2
    if my_curPoint==5
        my_complete=true;
    end
    my_checkStable=my_checkStable+1;
    my_timer2=my_timer2+1;
    if my_checkStable>500
        if my_curPoint<5
            my_goodToGo=true;
%             my_curPoint=my_curPoint+1;
%             disp(my_timer2);
            my_timer2=0;
        end
    end
else
    if my_timer2>0
        my_timer2=my_timer2+1;
    end
   my_checkStable=0;
   my_goodToGo=false;
end

if (my_xPosEst-my_desX(my_curPoint))^2+(my_yPosEst-my_desY(my_curPoint))^2<=0.004^2
    my_startTimer=true;
end
if my_startTimer
   my_timer=my_timer+1;
   if my_timer>680 || (my_timer>500 && my_curPoint==1)
       if my_curPoint<5
            my_curPoint=my_curPoint+1;
            disp(my_goodToGo);
            my_timer=0;
       end
       my_startTimer=false;
    end
end
my_angStdDev=deg2rad(1/3);
my_velStdDev=0;%(sqrt(my_angStdDev)/0.001);
%my_Q=diag([0.0058, my_velStdDev, 0.0058, my_velStdDev])^2;
my_Q=diag([0.0000492, 0.0000004, 0.0002506, 0.0000007]);
my_R=diag([deg2rad(1/3) deg2rad(1/3)])^2;

[my_F, my_P, my_ev]=lqr(my_A',my_C',my_Q,my_R);
my_F=my_F';
% F=place(A',C',[-10, -10.1,-10.2,-10.3])';
my_x_hat=((my_A-my_F*my_C)*(my_x_hat-[my_q_op(1) 0 my_q_op(2) 0]')+my_B*(U-my_U_eq)+my_F*(q-my_q_op))*0.001+my_x_hat;

my_yref=[my_q1des(my_curPoint); my_q2des(my_curPoint)];
my_x_des=[my_yref(1); 0; my_yref(2); 0];

if (mod(my_count,10000)==0)
    my_U_eq=my_solve_u(q(1),q(2));
    my_A=my_subsA(my_U_eq(1),my_U_eq(2),q(1),0,q(2),0);
    my_B=my_subsB(q(2));
    my_A=double(my_A);
    my_B=double(my_B);
    my_q_op=[my_x_hat(1); my_x_hat(3)];%q;

    my_A_aug=[my_A zeros(4, 2); my_C zeros(2)];
    my_B_aug=[my_B; zeros(2)];
%     K=place(A_aug,B_aug,[-5, -5.1, -5.2, -5.3, -5.4, -5.5]);
    my_Q=diag([1 1 1 1 100000 100000]);
    my_R=diag([0.01 0.01]);
    my_K=lqr(my_A_aug,my_B_aug,my_Q,my_R);
    
    my_K1=my_K(:,1:4);
    my_K2=my_K(:,5:6);
end

my_e=my_e_prev+(q-my_yref)*0.001; 
U=-my_K1*(my_x_hat-[my_q_op(1) 0 my_q_op(2) 0]')-my_K2*my_e+my_U_eq;
if my_complete==false
    my_energy=my_energy+(U'*U)*0.001;
end
my_e_prev=my_e;

my_count=my_count+1;