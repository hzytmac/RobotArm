%this is the general format of the simulator script
close all
% clear
% clc
Constants;
constants2;

    
%initializing code

%initial conditions   
X0=x_0;
U=tau_0;

m1noise=m1-0.1*m2;
m2noise=m2+0.1*m2;

c1noise=c1+0.5;
c2noise=c2+0.5;
%U=[1,1];
[tout,qout]=ode45(@(time,x)simulatorofficial(time,x,U,l1,l2,m1noise,m2noise,g,c1noise,c2noise),[0 0.001],X0);
q=qout(end,[1,3])';

q_total=[qout;zeros(10000*50,4)];
t_total=[tout;zeros(10000*50,1)];
index=length(tout)+1;
for t=0.001:0.001:10
   %check if robot meets requirements
   qact=q;
   q=q+deg2rad(1/3)*randn(1,1); % add measurement noise
   
    RobotControllerScript; %your script is used here.
    %U=[1;1];
   
   [tout,qout]=ode45(@(time,x)simulatorofficial(time,x,U,l1,l2,m1noise,m2noise,g,c1noise,c2noise),[t t+0.001],qout(end,:));
   if (my_complete==false)
       my_finishedT=t;
   end
   q=qout(end,[1,3])';
   t_total(index:index+length(tout)-1)=tout;
   q_total(index:index+length(tout)-1,:)=qout;
   index=index+length(tout);
end

i2 = find(t_total, 1, 'last');
t_total=t_total(1:i2);
q_total=q_total(1:i2,:);
figure()
plot(t_total,q_total(:,1),t_total,q_total(:,3));
legend('q1', 'q2');

disp('energy');
disp(my_energy);
disp('time');
disp(my_finishedT);

visualize([m1 m2 l1 l2 1 1], t_total, q_total(:,1), q_total(:,3), '');

%calculate energy/time, etc...

