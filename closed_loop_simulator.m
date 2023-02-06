function [Time,Q_plot,dQ_plot] = closed_loop_simulator(t1,t2,t3,t4,t5)
N_step = 20;
% startpos = pi;   % changed from pi/30 to pi/3 multiplicity from 5 to 10
% x0= [startpos*5*2;2.5*2*startpos;startpos;startpos;0.1*5*2;2*2.5*0.1;0.1;0.1]; % force added here
startpos = pi/3;
% x0= [startpos*5;2.5*startpos;startpos;startpos;0.1*5;2.5*0.1;0.1;0.1];
x0= [startpos*10;5*startpos;-3*startpos;5*startpos;0.2*5;3*0.2;0.1*3;8*0.1];
%--------------------------------------------------------------------------
% ODE parameters
AT = 1e-6;
RT = 1e-3;
RF = 1;
%--------------------------------------------------------------------------
gait_number = 1;
Tmax        = 1; % maximum elapsed time for the gait
Time        = [];
X           = [];
Q_plot           = [];
dQ_plot          = [];
% xtilda_plot = [];
% xtildadot_plot = [];
% u_plot = [];
% force_plot = [];                  %force added here
t_init      = 0;

options = odeset('AbsTol',AT,'RelTol',RT,'Refine',RF);

while gait_number<=N_step
    t_span = [t_init t_init+Tmax];
    %t_span = [0.02:0.02:2];
    %--------------------------------------------------------------------------
    %tic;
    % Continuous-time dynamics
    if(gait_number<=5)
        f_c = [10;10;10];
    else
        f_c = [0;0;0];
    end    
    [t,x]   = ode45(@closed_loop_dyn,t_span,x0,options,f_c); 
    %toc;
    %--------------------------------------------------------------------------
    % Save data
    t_temp  = t;
    x_temp  = x;
    t       = t_temp';
    x       = x_temp';
%     qtilda_temp = qtilda;
%     qtildadot_temp = qtildadot;
%     qtilda = qtilda_temp';
%     qtildadot = qtildadot_temp';
    Q_column       = x(1:4,:);  % Position
    dQ      = x(5:8,:); % Velocity
%     force = x(9:11,:); % force has been added here
    Time    = [Time t];
    Q_plot       = [Q_plot Q_column];
    dQ_plot      = [dQ_plot dQ];
%     force_plot = [force_plot force];
    X       = [X x];
%    trac_err = [trac_err pos_err];
%    vel_trac_err = [vel_trac_err vel_err];
    q_u = x0(1:4);
    dq_u = x0(5:8);
%     [U,xtilda,xtildadot] = u(q_u,dq_u,t,f_c);
%     u_plot = [u_plot U];
%     xtilda_plot = [xtilda_plot xtilda];
%     xtildadot_plot = [xtildadot_plot xtildadot];
    gait_number= gait_number + 1;
    t_init = Time(end);
    x0 = X(:,end);
end; % end of while




