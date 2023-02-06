function [com] = closed_loop_dyn(t,x,f_c)
%%%%this part to next comment is older
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% m = 3;
% g = 9.81;
% f;
% f_c = [0;0;f];
Q_column = x(1:4);
dQ = x(5:8);
% f = x(9:11);  % force has been added here
% force = [0;0;-m*g];
% pos_err = [];
% vel_err = [];
D_cld = Cfcn_inertia_Matrix(Q_column);
Cdq_cld = Cfcn_Coriolis_centrifugal_terms(Q_column,dQ);
G_cld = Cfcn_gravity_terms(Q_column);
H = Cdq_cld+G_cld;
jacob = Cfcn_jacobian(Q_column);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[U] = u(Q_column,dQ,t,f_c);


st_var = [dQ; inv(D_cld)*(U-H + jacob'*f_c) ];
com = st_var;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end