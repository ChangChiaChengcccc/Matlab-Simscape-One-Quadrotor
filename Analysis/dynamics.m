close all
% Declare
q = quadcopter_data;

q.R_w2b = sim_R_w2b.signals.values;
q.f1 = sim_f1.signals.values;
q.f2 = sim_f2.signals.values;
q.f3 = sim_f3.signals.values;
q.f4 = sim_f4.signals.values;
q.f = q.f1 + q.f2 + q.f3 + q.f4;
q.pos_meas = transpose(sim_pos_meas.signals.values);
q.vel_meas = transpose(sim_vel_meas.signals.values);
q.acc_meas = transpose(sim_acc_meas.signals.values);
q.W_meas = transpose(sim_W_meas.signals.values);
q.dW_meas = transpose(sim_dW_meas.signals.values); 
q.steps = linspace(1,length(q.f1),length(q.f1));
q.t_vec = sim_f1.time;

% dynamics
q.acc_dyn = zeros(3,length(q.f1));
q.dW_dyn = zeros(3,length(q.f1));
q.M = zeros(3,length(q.f1));
q.f_vec = zeros(4,length(q.f1));
q.ConIn = zeros(4,length(q.f1));

for i = 1:length(q.t_vec)
    % efficiency [1 0.7 0.7 1]
    % translational
    q.acc_dyn(:,i) = q.f(i)*inv(q.R_w2b(:,:,i))*q.e3/q.m - q.g*q.e3;

    % rotational
    q.f_vec(:,i) = [q.f1(i); q.f2(i); q.f3(i); q.f4(i)];
    q.ConIn(:,i) = q.allo*q.f_vec(:,i);
    q.M(:,i) = q.ConIn(2:4,i);
    q.dW_dyn(:,i) = inv(q.J)*(q.M(:,i)-cross(q.W_meas(:,i),q.J*q.W_meas(:,i)));
end

%ukf
 q.states = ukf_state_estimation(q);

%plot
%plotq(q)
