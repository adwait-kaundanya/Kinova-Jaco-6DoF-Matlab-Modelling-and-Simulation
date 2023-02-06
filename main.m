% Start of the project 
% First we calculate the DH parameters
tic;
clear;
AT = 1e-6;
RT = 1e-3;
RF = 1;
% This is section for variables
syms Q1 Q2 Q3 Q4 Q5 Q6 real
syms dQ1 dQ2 dQ3 dQ4 dQ5 dQ6 real
dQ = [dQ1; dQ2; dQ3; dQ4];
% masses of the arms
m1= 0.64; m2 =0.64; m3=0.64; m4= 0.64;
% gravitational constant
g=9.81;
aa=(11*pi)/72;
ca=cos(aa);
sa=sin(aa);
c2a=cos(2*aa);
s2a=sin(2*aa);
d4b=0.2073+((sa/s2a)*0.0743);
d5b=((sa/s2a)*0.0743)+((sa/s2a)*0.0743);
d6b=((sa/s2a)*0.0743)+0.1687;
alphaa = [90,180,90,55,55,180]; % this is the alpha value for all  the link
a=[0,0.41, 0, 0, 0, 0]; % Length of the Link
d=[0.2755,0,-0.0098,-d4b,-d5b,-d6b]; %Offset
Q=[Q1,Q2,Q3,Q4,Q5]; % joint angle variation
Q_column=[Q1;Q2;Q3;Q4];
m = 3;
g =9.81;
% 
% Txy is the transformation from frame x to frame y.

% Transformation Matrices
for i = 1:6
switch i
    case 1
       T01= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
    case 2
        T12= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
    case 3
        T23= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
    case 4
        T34= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
    case 5
        T45= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
%     case 6
%          T56= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];    
end
end
% 
% These are the individual tranformation matrices

T01;
T12;
T23;
T34;
T45;
% T56;
% transformation_matrix= T01*T12*T23*T34*T45*T56;
T02 = T01*T12;
T03 = T01*T12*T23;
T04 = T01*T12*T23*T34;

T05 = T01*T12*T23*T34*T45;
% T06 = T01*T12*T23*T34*T45*T56;
z0 = [0; 0; 1];
z1 = T01(1:3,3);
z2 = T02(1:3,3);
z3 = T03(1:3,3);
z4 = T04(1:3,3);
z5 = T05(1:3,3);
t1= T01(1:3, 4);
t2= T02(1:3, 4);
t3= T03(1:3, 4);
t4= T04(1:3, 4);
t5= T05(1:3, 4);
% t6= T06(1:3, 4);

% centre of mass 
com1 = t2/2;
com2 = (t3-t2)/2;
com3 = (t4-t3)/2; 
com4 = (t5-t4)/2;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Jcm1 = jacobian(com1,Q_column); Jcm1 = simplify(Jcm1); vcm1 = Jcm1*dQ; vcm1 = simplify(vcm1);
Jcm2 = jacobian(com2,Q_column); Jcm2 = simplify(Jcm2); vcm2 = Jcm2*dQ; vcm2 = simplify(vcm2);
Jcm3 = jacobian(com3,Q_column); Jcm3 = simplify(Jcm3); vcm3 = Jcm3*dQ; vcm3 = simplify(vcm3);
Jcm4 = jacobian(com4,Q_column); Jcm4 = simplify(Jcm4); vcm4 = Jcm4*dQ; vcm4 = simplify(vcm4);
Jee =  jacobian(t5,Q_column); Jee = simplify(Jee);
jeedq = Jee*dQ;
jacob_jeedq =  jacobian(jeedq,Q_column)*dQ; jacob_jeedq = simplify(jacob_jeedq);

% Jcm5 = jacobian(com5,Q); Jcm5 = simplify(Jcm5); vcm5 = Jcm5*dQ; vcm5 = simplify(vcm5);
% Jcm6 = jacobian(com6,Q); Jcm6 = simplify(Jcm6); vcm6 = Jcm6*dQ; vcm6 = simplify(vcm6);
% angular jacobian for the ee
% Jwcm6 = [z0,z1,z2,z3];
% wcm6 = Jwcm6*dQ; wcm6 = simplify(Jwcm6*dQ); 
% 
% % Inertia matrix for the ee This was an attempt to add inertial
% parameters
% I = [0.1,0.1,0.1;0.1, 0.1, 0.1; 0.1, 0.1, 0.1];
% to_be_jaco = jacobian(Jcm4*


% Kinetic energy  capital(K)
K = 0.5*m1*transpose(vcm1)*vcm1 + 0.5*m2*transpose(vcm2)*vcm2 + 0.5*m3*transpose(vcm3)*vcm3 + 0.5*m4*transpose(vcm4)*vcm4;
K = simplify(K);

% Compute the mass-inertia matrix
dK_dqdot = jacobian(K,dQ);
D        = jacobian(dK_dqdot,dQ);
D        = simplify(D);

% COmpute the Coriolis and centrifugal terms
C = 0*D;
N = length(Q_column);
for k=1:N
    for j=1:N
        sum = 0*g;
        for i=1:N
            sum = sum + 1/2*(diff(D(k,j),Q_column(i)) + diff(D(k,i),Q_column(j)) - diff(D(i,j),Q_column(k)))*dQ(i);
        end
        C(k,j) = sum;
    end
end
% Cdq = C*dQ;
% Cdq = simplify(Cdq);

% Gravity terms
V = m1*g*(com1(3)/2) +m2*g*(com2(3)/2)+ m3*g*(com3(3)/2)+ m4*g*(com4(3)/2) ;
G = jacobian(V,Q_column).';

% the new jaco combined term





% Save m.files
% matlabFunction(D, 'vars', {Q_column}, 'file', 'inertia_Matrix', 'Optimize', false);
% matlabFunction(Cdq, 'vars', {Q_column,dQ}, 'file', 'Coriolis_centrifugal_terms', 'Optimize', false);
% matlabFunction(C, 'vars', {Q_column,dQ}, 'file', 'Coriolis_centrifugal_wodq', 'Optimize', false);
% matlabFunction(G, 'vars', {Q_column}, 'file', 'gravity_terms', 'Optimize', false);
% matlabFunction(jacob_jeedq, 'vars', {Q_column,dQ}, 'file', 'endeffect_jacobian', 'Optimize', false);
% matlabFunction(T04, 'vars', {Q_column}, 'file', 'ee_pos', 'Optimize', false);
%matlabFunction(t5, 'vars', {Q_column}, 'file', 'endeffector_cartesian', 'Optimize', false);
%matlabFunction(Jee, 'vars', {Q_column}, 'file', 'newjacobian', 'Optimize', false);


% codegen inertia_Matrix -args {zeros(4,1)}  -o Cfcn_inertia_Matrix -report
% codegen Coriolis_centrifugal_wodq -args {zeros(4,1),zeros(4,1)}  -o Cfcn_Coriolis_centrifugal_wodq -report
%codegen gravity_terms  -args {zeros(4,1)}  -o Cfcn_gravity_terms  -report
%codegen ee_jacobian  -args {zeros(4,1)}  -o Cfcn_jacobian  -report
%codegen newjacobian  -args {zeros(4,1)}  -o Cfcn_jacobian_ee  -report
[Time,q,dq] = closed_loop_simulator(t1,t2,t3,t4,t5);
%%%%%%%%%%%%%%%%%%%%%%plotting figures
% a = [0.1 0.1 0.2 0.6 ; 0 0.2 0.3 0.5; 0 0.1 0.2 0.7; 0 0.3 0.1 0.6];
% a = [1 2 3 4; 5 6 7 8; 2 5 6 7; 7 11 12 18];
% change in animation also
% a = [pi/8 pi/8 pi pi; pi/4 pi/4 pi/2 pi/2; pi/6 pi/6 pi/4 pi/4; pi/8 pi/8 pi/4 pi/4];
% ret_bez = [];
% retd_bez = [];
% for i = 1: length(Time)
%     ret = bezier(a,Time(i)/20);
%     retd = bezierd(a,Time(i)/20);
%     ret_bez = [ret_bez ret];
%     retd_bez = [retd_bez retd];
% end    
% plot_err = Q_return - ret_bez;
% velocity_err = dQ_return - retd_bez;
alpha = [pi/8 pi/8 pi pi; pi/4 pi/4 pi/2 pi/2; pi/6 pi/6 pi/4 pi/4; pi/8 pi/8 pi/4 pi/4];
u_plot = [];
xtil_plot = [];
xtildot_plot = [];
ret_bez = [];
for i = 1: length(Time)
    if(Time(i)<=5)
        fimp = [10;10;10];
    else
        fimp = [0;0;0];
    end    
    [U,xtil,xtildot,xd] = u(q(:,i),dq(:,i),Time(i),fimp);
    retd = bezier(alpha,Time(i)/20);
    %xeemat = ee_pos(q(:,i));
    xee = endeffector_cartesian(q(:,1));
  
    xtilda = xee -xd;
    u_plot = [u_plot U];
    xtil_plot = [xtil_plot xtilda];
    xtildot_plot = [xtildot_plot xtildot];
    ret_bez = [ret_bez retd];
end    
% figure(1);
% title('xtil')
% plot(Time,xtil_plot)
% figure(2);
% title('xtildadot')
% plot(Time,xtildot_plot)
% figure(3);
% title('uplot')
% plot(Time,u_plot)
% m = 3;
% g = 9.81;
% f_con = [0;0; -m*g]
figure(1)
sgtitle('u vs t profiles')
% subplot(2,2,1)
plot(Time,u_plot)
ylim([-10 10]);
xlabel('Time(s)')
ylabel('Control Input')
grid on
% subplot(2,2,2)
% plot(Time,u_plot(2,:))
% xlabel('Time(s)')
% ylabel('dq2(rad/s)')
% grid on
% subplot(2,2,3)
% plot(Time,u_plot(3,:))
% xlabel('Time(s)')
% ylabel('dq3(rad/s)')
% grid on
% subplot(2,2,4)
% plot(Time,u_plot(4,:))
% xlabel('Time(s)')
% ylabel('dq4(rad/s)')
% grid on


figure(2)
sgtitle('xtildadot vs t profiles')
% subplot(2,2,1)
plot(Time,xtildot_plot)
xlabel('Time(s)')
ylabel('xtildadot(m/s)')
grid on
% subplot(2,2,2)
% plot(Time,xtildot_plot(2,:))
% xlabel('Time(s)')
% ylabel('dq2(rad/s)')
% grid on
% subplot(2,2,3)
% plot(Time,xtildot_plot(3,:))
% xlabel('Time(s)')
% ylabel('dq3(rad/s)')
% grid on

figure(3)
sgtitle('xtilda vs t profiles')
% subplot(2,2,1)
plot(Time,xtil_plot)
xlabel('Time(s)')
ylabel('xtilda(m/s)')
grid on
% subplot(2,2,2)
% plot(Time,xtil_plot(2,:))
% xlabel('Time(s)')
% ylabel('dq2(rad/s)')
% grid on
% subplot(2,2,3)
% plot(Time,xtil_plot(3,:))
% xlabel('Time(s)')
% ylabel('dq3(rad/s)')
% grid on
% subplot(2,2,4)
% plot(Time,velocity_err(4,:))
% xlabel('Time(s)')
% ylabel('dq4(rad/s)')
% grid on

% as = f_con - f_plot;
% figure(3)
% sgtitle('force')
% plot(Time,as);
% print('dq_t_plots','-djpeg')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
animator(q,Time,t1,t2,t3,t4);


% figure(1)
% sgtitle('q vs t profiles')
% subplot(2,2,1)
% plot(Time,pos_err)
% xlabel('Time(s)')
% ylabel('q1(rad)')
% grid on
% 

% a = [0 0.1 pi pi/2 ; 0 pi 0.6 pi/2; 0 0.5 0.9 pi/2; 0 0.3 0.6 pi/2];
%a = [1 3 4 5; 8 9 0 3; 2 5 6 1; 1 1 18 0];
% qd = [];
% qdotd = [];
% t = 0:1:20;
% for i = 0:1:20
%     calc = bezier(a,i);
%     calc1 = beziera(a,i);
%     qd = [qd calc];
%     qdotd = [qdotd calc1];
%     qd = qdotd;
%     
% 
% 
% end
% figure(2)
% sgtitle('dq vs t profiles')
% subplot(2,2,1)
% plot(Time,dQ_return(1,:))
% xlabel('Time(s)')
% ylabel('dq1(rad/s)')
% grid on
% subplot(2,2,2)
% plot(Time,dQ_return(2,:))
% xlabel('Time(s)')
% ylabel('dq2(rad/s)')
% grid on
% subplot(2,2,3)
% plot(Time,dQ_return(3,:))
% xlabel('Time(s)')
% ylabel('dq3(rad/s)')
% grid on
% subplot(2,2,4)
% plot(Time,dQ_return(4,:))
% xlabel('Time(s)')
% ylabel('dq4(rad/s)')
% grid on
% print('dq_t_plots','-djpeg')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% animator(q,Time,t1,t2,t3,t4);


% figure(1)
% sgtitle('q vs t profiles')
% subplot(2,2,1)
% plot(Time,pos_err)
% xlabel('Time(s)')
% ylabel('q1(rad)')
% grid on
% 

% a = [0 0.1 pi pi/2 ; 0 pi 0.6 pi/2; 0 0.5 0.9 pi/2; 0 0.3 0.6 pi/2];
%a = [1 3 4 5; 8 9 0 3; 2 5 6 1; 1 1 18 0];
% qd = [];
% qdotd = [];
% t = 0:1:20;
% for i = 0:1:20
%     calc = bezier(a,i);
%     calc1 = beziera(a,i);
%     qd = [qd calc];
%     qdotd = [qdotd calc1];
%     qd = qdotd;
%     
% 
% 
% end

toc;