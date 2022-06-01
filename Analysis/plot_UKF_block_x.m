UKF_x = UKF_block_x.signals.values;
t_vec = UKF_block_x.time;
size_UKF_x = size(UKF_x);
UKF_state = zeros(size_UKF_x(1),size_UKF_x(3));
for i=1:size_UKF_x(3)
    UKF_state(:,i) =  UKF_x(:,:,i);
end
UKF = UKF_data;
UKF.x = UKF_state(1:3,:);
UKF.v = UKF_state(4:6,:);
UKF.a = UKF_state(7:9,:);
UKF.W = UKF_state(10:12,:);
UKF.dW = UKF_state(13:15,:);
UKF.E = UKF_state(16:19,:);

quadrotor = quadcopter_data;
quadrotor.R_w2b = sim_R_w2b.signals.values;
quadrotor.f1 = sim_f1.signals.values;
quadrotor.f2 = sim_f2.signals.values;
quadrotor.f3 = sim_f3.signals.values;
quadrotor.f4 = sim_f4.signals.values;
quadrotor.f = quadrotor.f1 + quadrotor.f2 + quadrotor.f3 + quadrotor.f4;
quadrotor.pos_meas = transpose(sim_pos_meas.signals.values);
quadrotor.vel_meas = transpose(sim_vel_meas.signals.values);
quadrotor.acc_meas = transpose(sim_acc_meas.signals.values);
quadrotor.W_meas = transpose(sim_W_meas.signals.values);
quadrotor.dW_meas = transpose(sim_dW_meas.signals.values); 
quadrotor.steps = linspace(1,length(quadrotor.f1),length(quadrotor.f1));
quadrotor.t_vec = sim_f1.time;
quadrotor.ex_enu = zeros(3,length(t_vec));
quadrotor.eR = zeros(3,length(t_vec));
for i=1:length(t_vec)
    ex_ned =  sim_ex_ned.signals.values(:,1,i);
    quadrotor.ex_enu(:,i) = [ex_ned(2);ex_ned(1);-ex_ned(3)];
    quadrotor.eR(:,i)= sim_eR.signals.values(:,1,i);
end

true_state_vec = [ quadrotor.pos_meas; quadrotor.vel_meas; ...
                   quadrotor.acc_meas; quadrotor.W_meas; quadrotor.dW_meas ];

%% plot
% x
figure(1)
subplot(3,1,1)
plot(t_vec, true_state_vec(1,:),t_vec, UKF_state(1,:),'b');
title('2-D x Plot','FontSize',20);
x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
y = ylabel('$x(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
legend('$x\ ideal$','$x\ estimation$', 'Interpreter', 'latex','FontSize',20)
grid on
subplot(3,1,2)
plot(t_vec, true_state_vec(2,:),'k',t_vec, UKF_state(2,:),'b');
title('2-D y Plot','FontSize',20);
x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
y = ylabel('$y(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
legend('$y\ ideal$','$y\ estimation$', 'Interpreter', 'latex','FontSize',20)
grid on
subplot(3,1,3)
plot(t_vec, true_state_vec(3,:),'k',t_vec, UKF_state(3,:),'b');
title('2-D z Plot','FontSize',20);
x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
y = ylabel('$z(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
legend('$z\ ideal$','$z\ estimation$', 'Interpreter', 'latex','FontSize',20)
grid on

% v
figure(2)
subplot(3,1,1)
plot(t_vec, true_state_vec(4,:),'k',t_vec, UKF_state(4,:),'b');
title('2-D vx Plot','FontSize',20);
x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
y = ylabel('$vx(m/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
legend('$vx\ ideal$','$vx\ estimation$', 'Interpreter', 'latex','FontSize',20)
grid on
subplot(3,1,2)
plot(t_vec, true_state_vec(5,:),'k',t_vec, UKF_state(5,:),'b');
title('2-D vy Plot','FontSize',20);
x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
y = ylabel('$vy(m/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
legend('$vy\ ideal$','$vy\ estimation$', 'Interpreter', 'latex','FontSize',20)
grid on
subplot(3,1,3)
plot(t_vec, true_state_vec(6,:),'k',t_vec, UKF_state(6,:),'b');
title('2-D vz Plot','FontSize',20);
x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
y = ylabel('$vz(m/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
legend('$vz\ ideal$','$vz\ estimation$', 'Interpreter', 'latex','FontSize',20)
grid on
% a
figure(3)
subplot(3,1,1)
plot(t_vec, true_state_vec(7,:),'k',t_vec, UKF_state(7,:),'b');
title('2-D ax Plot','FontSize',20);
x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
y = ylabel('$ax(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
legend('$ax\ ideal$','$ax\ estimation$', 'Interpreter', 'latex','FontSize',20)
grid on
subplot(3,1,2)
plot(t_vec, true_state_vec(8,:),'k',t_vec, UKF_state(8,:),'b');
title('2-D ay Plot','FontSize',20);
x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
y = ylabel('$ay(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
legend('$ay\ ideal$','$ay\ estimation$', 'Interpreter', 'latex','FontSize',20)
grid on
subplot(3,1,3)
plot(t_vec, true_state_vec(9,:),'k',t_vec, UKF_state(9,:),'b');
title('2-D az Plot','FontSize',20);
x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
y = ylabel('$az(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
legend('$az\ ideal$','$az\ estimation$', 'Interpreter', 'latex','FontSize',20)
grid on
% W 
figure(5)
subplot(3,1,1)
plot(t_vec, true_state_vec(10,:),'k',t_vec, UKF_state(10,:),'b');
title('2-D wx Plot','FontSize',20);
x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
y = ylabel('$wx(rad/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
legend('$wx\ ideal$','$wx\ estimation$', 'Interpreter', 'latex','FontSize',20)
grid on
subplot(3,1,2)
plot(t_vec, true_state_vec(11,:),'k',t_vec, UKF_state(11,:),'b');
title('2-D wy Plot','FontSize',20);
x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
y = ylabel('$wy(rad/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
legend('$wy\ ideal$','$wy\ estimation$', 'Interpreter', 'latex','FontSize',20)
grid on
subplot(3,1,3)
plot(t_vec, true_state_vec(12,:),'k',t_vec, UKF_state(12,:),'b');
title('2-D wz Plot','FontSize',20);
x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
y = ylabel('$wz(rad/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
legend('$wz\ ideal$','$wz\ estimation$', 'Interpreter', 'latex','FontSize',20)
grid on
% dW
figure(6)
subplot(3,1,1)
plot(t_vec, true_state_vec(13,:),'k',t_vec, UKF_state(13,:),'b');
title('2-D dWx Plot','FontSize',20);
x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
y = ylabel('$dWx(rad/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
legend('$dWx\ ideal$','$dWx\ estimation$', 'Interpreter', 'latex','FontSize',20)
grid on
subplot(3,1,2)
plot(t_vec, true_state_vec(14,:),'k',t_vec, UKF_state(14,:),'b');
title('2-D dWy Plot','FontSize',20);
x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
y = ylabel('$dWy(rad/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
legend('$dWy\ ideal$','$dWy\ estimation$', 'Interpreter', 'latex','FontSize',20)
grid on
subplot(3,1,3)
plot(t_vec, true_state_vec(15,:),'k',t_vec, UKF_state(15,:),'b');
title('2-D dWz Plot','FontSize',20);
x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
y = ylabel('$dWz(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
legend('$dWz\ ideal$','$dWz\ estimation$', 'Interpreter', 'latex','FontSize',20)
grid on

% E
figure(7)
plot(t_vec, UKF_state(16,:),'b',t_vec, UKF_state(17,:),'c',t_vec, UKF_state(18,:),'m',t_vec, UKF_state(19,:),'y');
ylim([0 1.1]);
title('2-D E Plot','FontSize',20);
x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
y = ylabel('$Efficiency$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
legend('$e1\ estimation$','$e2\ estimation$','$e3\ estimation$','$e4\ estimation$', 'Interpreter', 'latex','FontSize',15)
grid on

% ex_enu
figure(8)
subplot(3,1,1)
plot(t_vec, quadrotor.ex_enu(1,:),'b');
title('2-D error_x Plot','FontSize',20);
x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
y = ylabel('$x(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
legend('$error_x$', 'Interpreter', 'latex','FontSize',20)
grid on
subplot(3,1,2)
plot(t_vec, quadrotor.ex_enu(2,:),'b');
title('2-D error_y Plot','FontSize',20);
x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
y = ylabel('$y(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
legend('$error_y$', 'Interpreter', 'latex','FontSize',20)
grid on
subplot(3,1,3)
plot(t_vec, quadrotor.ex_enu(3,:),'b');
title('2-D error_z Plot','FontSize',20);
x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
y = ylabel('$z(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
legend('$error_z$', 'Interpreter', 'latex','FontSize',20)
grid on

% eR
figure(9)
subplot(3,1,1)
plot(t_vec, quadrotor.eR(1,:),'b');
title('2-D error_R(1) Plot','FontSize',20);
x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
legend('$error_R(1)$', 'Interpreter', 'latex','FontSize',20)
grid on
subplot(3,1,2)
plot(t_vec, quadrotor.eR(2,:),'b');
title('2-D error_R(2) Plot','FontSize',20);
x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
legend('$error_R(2)$', 'Interpreter', 'latex','FontSize',20)
grid on
subplot(3,1,3)
plot(t_vec, quadrotor.eR(3,:),'b');
title('2-D error_R(3) Plot','FontSize',20);
x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
legend('$error_R(3)$', 'Interpreter', 'latex','FontSize',20)
grid on