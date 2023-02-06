clc
clear all

syms q1 q2 q3 q4 q5 q6 real
syms dq1 dq2 dq3 dq4 dq5 dq6 real

syms r l real
syms m MH MT g real

% Generalized coordinates
q  = [q1; q2; q3; q4; q5; q6];
dq = [dq1; dq2; dq3; dq5; dq5; dq6];

% Positions
pcm1 = [r/2*sin(q1); r/2*cos(q1)];         % CoM for leg 1 pos
pcmH = [r*sin(q1); r*cos(q1)];             % CoM for the hip
pcm2 = pcmH + [r/2*sin(-q2); -r/2*cos(-q2)]; % CoM for leg 2
pcmT = pcmH + [l*sin(q3); l*cos(q3)];
p2   = pcmH + [r*sin(-q2); -r*cos(-q2)];

% Velocities
Jcm1 = jacobian(pcm1,q); Jcm1 = simplify(Jcm1); vcm1 = Jcm1*dq; vcm1 = simplify(vcm1);
Jcm2 = jacobian(pcm2,q); Jcm2 = simplify(Jcm2); vcm2 = Jcm2*dq; vcm2 = simplify(vcm2);
JcmH = jacobian(pcmH,q); JcmH = simplify(JcmH); vcmH = JcmH*dq; vcmH = simplify(vcmH);
JcmT = jacobian(pcmT,q); JcmT = simplify(JcmT); vcmT = JcmT*dq; vcmT = simplify(vcmT);

% Compute the Kinetic enrgy
K = 1/2*m*transpose(vcm1)*vcm1  + 1/2*m*transpose(vcm2)*vcm2 +...
    1/2*MH*transpose(vcmH)*vcmH + 1/2*MT*transpose(vcmT)*vcmT;

K = simplify(K);

% Compute the mass-inertia matrix
dK_dqdot = jacobian(K,dq);
D        = jacobian(dK_dqdot,dq);
D        = simplify(D);

% COmpute the Coriolis and centrifugal terms
C = 0*D;
N = length(q);
for k=1:N
    for j=1:N
        sum = 0*g;
        for i=1:N
            sum = sum + 1/2*(diff(D(k,j),q(i)) + diff(D(k,i),q(j)) - diff(D(i,j),q(k)))*dq(i);
        end
        C(k,j) = sum;
    end
end
Cdq = C*dq;
Cdq = simplify(Cdq);

% Gravity terms
V = m*g*pcm1(2) + m*g*pcm2(2) + MH*g*pcmH(2) + MT*g*pcmT(2); V = simplify(V);
G = jacobian(V,q).';

% Input matrix
qa1 = pi - q1 + q3;
qa2 = pi - q2 + q3;
qa  = [qa1; qa2];
B   = jacobian(qa,q).';

% Impact dynamics
syms x y real
syms dx dy real

qe  = [q; x; y];
dqe = [dq; dx; dy];

% Positions
pcm1e = [x; y] + [r/2*sin(q1); r/2*cos(q1)];         % CoM for leg 1 pos
pcmHe = [x; y] + [r*sin(q1); r*cos(q1)];             % CoM for the hip
pcm2e = pcmHe + [r/2*sin(-q2); -r/2*cos(-q2)]; % CoM for leg 2
pcmTe = pcmHe + [l*sin(q3); l*cos(q3)];

% Velocities
Jcm1e = jacobian(pcm1e,qe); Jcm1e = simplify(Jcm1e); vcm1e = Jcm1e*dqe; vcm1e = simplify(vcm1e);
Jcm2e = jacobian(pcm2e,qe); Jcm2e = simplify(Jcm2e); vcm2e = Jcm2e*dqe; vcm2e = simplify(vcm2e);
JcmHe = jacobian(pcmHe,qe); JcmHe = simplify(JcmHe); vcmHe = JcmHe*dqe; vcmHe = simplify(vcmHe);
JcmTe = jacobian(pcmTe,qe); JcmTe = simplify(JcmTe); vcmTe = JcmTe*dqe; vcmTe = simplify(vcmTe);

% Compute the Kinetic enrgy
Ke = 1/2*m*transpose(vcm1e)*vcm1e  + 1/2*m*transpose(vcm2e)*vcm2e +...
     1/2*MH*transpose(vcmHe)*vcmHe + 1/2*MT*transpose(vcmTe)*vcmTe;

Ke = simplify(Ke);

% Compute the mass-inertia matrix
dK_dqdot = jacobian(Ke,dqe);
De        = jacobian(dK_dqdot,dqe);
De        = simplify(De);

% Compute Je (the Jacobian at the swing leg end)
p2e = pcmHe + [r*sin(-q2); -r*cos(-q2)]; % swing leg end pos.
Je = jacobian(p2e,qe); Je = simplify(Je);

%%
robot_params = [r; l; m; MT; MH; g];
% Save m.files
matlabFunction(D, 'vars', {q,robot_params}, 'file', 'inertia_Matrix', 'Optimize', false);
matlabFunction(Cdq, 'vars', {q,dq,robot_params}, 'file', 'Coriolis_centrifugal_terms', 'Optimize', false);
matlabFunction(G, 'vars', {q,robot_params}, 'file', 'gravity_terms', 'Optimize', false);
matlabFunction(B, 'vars', {q,robot_params}, 'file', 'input_matrix', 'Optimize', false);
matlabFunction(De, 'vars', {qe,robot_params}, 'file', 'extended_inertia_Matrix', 'Optimize', false);
matlabFunction(Je, 'vars', {qe,robot_params}, 'file', 'extended_swing_Jacobian', 'Optimize', false);
matlabFunction([p2,pcmH,pcmT], 'vars', {q,robot_params}, 'file', 'forward_kinematics', 'Optimize', false);
