function [newval,xtilda,xtildadot,xd] = u(q,dq,t,f)
%U Summary of this function goes here
%   Detailed explanation goes here
% m = 3;
% g = 9.81;
a = [pi/8 pi/8 pi pi; pi/4 pi/4 pi/2 pi/2; pi/6 pi/6 pi/4 pi/4; pi/8 pi/8 pi/4 pi/4];
Kp = 7000*eye(3);
D = Cfcn_inertia_Matrix(q);
C = Cfcn_Coriolis_centrifugal_wodq(q,dq);
G = Cfcn_gravity_terms(q);
Kd = 2500*eye(3);
% V = 4*eye(4);
qd = bezier(a,t/20);
qdcart = Cfcn_jacobian(qd);
qddot = bezierd(a,t/20);
qddotcart = Cfcn_jacobian(qddot);
acc = beziera(a,t/20);
acccart = Cfcn_jacobian(acc);
% eecart = ee_pos(q);
xee = endeffector_cartesian(q);
jt = Cfcn_jacobian_ee(q);
% jt = Cfcn_jacobian(q);
xeedot = jt*dq;

% pos_err = [];
% vel_err = [];
% qtilda = q-qd;
% qtildadot = dq-qddot;
% zeta double dot still needs to be written
% zetadot = qddot-(V*qtilda);
% sigma = qtildadot + (V*qtilda);
% zetadoubledot = acc-(V*qddot);

% val = (D*zetadoubledot) + (C*zetadot)+G-(Kd*sigma) - (jt'*f);
% pe = qtilda;
% ve = qtildadot;
% f_adapted = KI*jt*sigma;
xd = qdcart(:,4);
xddot = qddotcart(:,4);
xdoubledot = acccart(:,4);

xtilda = xee - xd;
xtildadot = xeedot - xddot;

weirdterm = Cfcn_endeffect_jacobian(q,dq);
proj = eye(3) - (jt*inv(D)*jt');

newval = (pinv(jt*inv(D)))*(xdoubledot - Kp*xtilda - Kd*xtildadot + (jt*inv(D)*((C*dq) + G)) - weirdterm + (proj*f));
% size(jt*inv(D))
% pos_err = [pos_err; pe];
% vel_err = [vel_err; ve];
end

