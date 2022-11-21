close all
UKF_x = UKF_block_x.signals.values;
t_vec = UKF_block_x.time;
t_vec_select = UKF_block_x.time(1:14203);
size_UKF_x = size(UKF_x);
UKF_state = zeros(size_UKF_x(1),size_UKF_x(3));
for i=1:size_UKF_x(3)
    UKF_state(:,i) =  UKF_x(:,:,i);
end
UKF = UKF_data;
UKF.x = UKF_state(1:3,1:length(t_vec));
UKF.v = UKF_state(4:6,1:length(t_vec));
UKF.a = UKF_state(7:9,1:length(t_vec));
UKF.W = UKF_state(10:12,1:length(t_vec));
UKF.dW = UKF_state(13:15,1:length(t_vec));
UKF.E = UKF_state(16:19,1:length(t_vec));

quadrotor = quadcopter_data;
quadrotor.R_w2b = sim_R_w2b.signals.values;
quadrotor.f1 = sim_f1.signals.values;
quadrotor.f2 = sim_f2.signals.values;
quadrotor.f3 = sim_f3.signals.values;
quadrotor.f4 = sim_f4.signals.values;
ord_f1 = sim_ord_f1.signals.values;
ord_f2 = sim_ordi_f2.signals.values;
ord_f3 = sim_ordi_f3.signals.values;
ord_f4 = sim_ordi_f4.signals.values;
quadrotor.f = quadrotor.f1 + quadrotor.f2 + quadrotor.f3 + quadrotor.f4;
quadrotor.pos_meas = transpose(sim_pos_meas.signals.values);
quadrotor.vel_meas = transpose(sim_vel_meas.signals.values);
quadrotor.acc_meas = transpose(sim_acc_meas.signals.values);
quadrotor.W_meas = transpose(sim_W_meas.signals.values);
quadrotor.dW_meas = transpose(sim_dW_meas.signals.values); 
quadrotor.steps = linspace(1,length(quadrotor.f1),length(quadrotor.f1));
quadrotor.t_vec = sim_f1.time;
quadrotor.ex_enu = zeros(3,length(t_vec));
quadrotor.ev_enu = zeros(3,length(t_vec));
quadrotor.eR = zeros(3,length(t_vec));
quadrotor.eW = zeros(3,length(t_vec));
quadrotor.traj = transpose(sim_Traj.signals.values(:,1:3));
ones_vec = ones(1,length(t_vec_select));
for i=1:length(t_vec)
    ex_ned =  sim_ex_ned.signals.values(:,1,i);
    quadrotor.ex_enu(:,i) = [ex_ned(2);ex_ned(1);-ex_ned(3)];
    ev_ned =  sim_ev_ned.signals.values(:,1,i);
    quadrotor.ev_enu(:,i) = [ev_ned(2);ev_ned(1);-ev_ned(3)];
    quadrotor.eR(:,i)= sim_eR.signals.values(:,1,i);
    quadrotor.eW(:,i)= sim_eW.signals.values(:,1,i);
end

true_state_vec = [ quadrotor.pos_meas(:,1:length(t_vec)); quadrotor.vel_meas(:,1:length(t_vec)); ...
                   quadrotor.acc_meas(:,1:length(t_vec)); quadrotor.W_meas(:,1:length(t_vec));...
                   quadrotor.dW_meas(:,1:length(t_vec))];

% %% plot
% % x
% figure(1)
% subplot(3,1,1)
% plot(t_vec, true_state_vec(1,1:length(t_vec)),t_vec, UKF_state(1,1:length(t_vec)),'b');
% title('2-D x Plot','FontSize',20);
% x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% y = ylabel('$x(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
% legend('$x\ ideal$','$x\ estimation$', 'Interpreter', 'latex','FontSize',20)
% grid on
% subplot(3,1,2)
% plot(t_vec, true_state_vec(2,1:length(t_vec)),'k',t_vec, UKF_state(2,1:length(t_vec)),'b');
% title('2-D y Plot','FontSize',20);
% x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% y = ylabel('$y(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
% legend('$y\ ideal$','$y\ estimation$', 'Interpreter', 'latex','FontSize',20)
% grid on
% subplot(3,1,3)
% plot(t_vec, true_state_vec(3,1:length(t_vec)),'k',t_vec, UKF_state(3,1:length(t_vec)),'b');
% title('2-D z Plot','FontSize',20);
% x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% y = ylabel('$z(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
% legend('$z\ ideal$','$z\ estimation$', 'Interpreter', 'latex','FontSize',20)
% grid on
% 
% % v
% figure(2)
% subplot(3,1,1)
% plot(t_vec, true_state_vec(4,1:length(t_vec)),'k',t_vec, UKF_state(4,1:length(t_vec)),'b');
% title('2-D vx Plot','FontSize',20);
% x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% y = ylabel('$vx(m/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
% legend('$vx\ ideal$','$vx\ estimation$', 'Interpreter', 'latex','FontSize',20)
% grid on
% subplot(3,1,2)
% plot(t_vec, true_state_vec(5,1:length(t_vec)),'k',t_vec, UKF_state(5,1:length(t_vec)),'b');
% title('2-D vy Plot','FontSize',20);
% x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% y = ylabel('$vy(m/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
% legend('$vy\ ideal$','$vy\ estimation$', 'Interpreter', 'latex','FontSize',20)
% grid on
% subplot(3,1,3)
% plot(t_vec, true_state_vec(6,1:length(t_vec)),'k',t_vec, UKF_state(6,1:length(t_vec)),'b');
% title('2-D vz Plot','FontSize',20);
% x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% y = ylabel('$vz(m/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
% legend('$vz\ ideal$','$vz\ estimation$', 'Interpreter', 'latex','FontSize',20)
% grid on
% % a
% figure(3)
% subplot(3,1,1)
% plot(t_vec, true_state_vec(7,1:length(t_vec)),'k',t_vec, UKF_state(7,1:length(t_vec)),'b');
% title('2-D ax Plot','FontSize',20);
% x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% y = ylabel('$ax(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
% legend('$ax\ ideal$','$ax\ estimation$', 'Interpreter', 'latex','FontSize',20)
% grid on
% subplot(3,1,2)
% plot(t_vec, true_state_vec(8,1:length(t_vec)),'k',t_vec, UKF_state(8,1:length(t_vec)),'b');
% title('2-D ay Plot','FontSize',20);
% x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% y = ylabel('$ay(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
% legend('$ay\ ideal$','$ay\ estimation$', 'Interpreter', 'latex','FontSize',20)
% grid on
% subplot(3,1,3)
% plot(t_vec, true_state_vec(9,1:length(t_vec)),'k',t_vec, UKF_state(9,1:length(t_vec)),'b');
% title('2-D az Plot','FontSize',20);
% x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% y = ylabel('$az(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
% legend('$az\ ideal$','$az\ estimation$', 'Interpreter', 'latex','FontSize',20)
% grid on
% % W 
% figure(5)
% subplot(3,1,1)
% plot(t_vec, true_state_vec(10,1:length(t_vec)),'k',t_vec, UKF_state(10,1:length(t_vec)),'b');
% title('2-D wx Plot','FontSize',20);
% x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% y = ylabel('$wx(rad/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
% legend('$wx\ ideal$','$wx\ estimation$', 'Interpreter', 'latex','FontSize',20)
% grid on
% subplot(3,1,2)
% plot(t_vec, true_state_vec(11,1:length(t_vec)),'k',t_vec, UKF_state(11,1:length(t_vec)),'b');
% title('2-D wy Plot','FontSize',20);
% x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% y = ylabel('$wy(rad/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
% legend('$wy\ ideal$','$wy\ estimation$', 'Interpreter', 'latex','FontSize',20)
% grid on
% subplot(3,1,3)
% plot(t_vec, true_state_vec(12,1:length(t_vec)),'k',t_vec, UKF_state(12,1:length(t_vec)),'b');
% title('2-D wz Plot','FontSize',20);
% x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% y = ylabel('$wz(rad/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
% legend('$wz\ ideal$','$wz\ estimation$', 'Interpreter', 'latex','FontSize',20)
% grid on
% % dW
% figure(6)
% subplot(3,1,1)
% plot(t_vec, true_state_vec(13,1:length(t_vec)),'k',t_vec, UKF_state(13,1:length(t_vec)),'b');
% title('2-D dWx Plot','FontSize',20);
% x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% y = ylabel('$dWx(rad/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
% legend('$dWx\ ideal$','$dWx\ estimation$', 'Interpreter', 'latex','FontSize',20)
% grid on
% subplot(3,1,2)
% plot(t_vec, true_state_vec(14,1:length(t_vec)),'k',t_vec, UKF_state(14,1:length(t_vec)),'b');
% title('2-D dWy Plot','FontSize',20);
% x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% y = ylabel('$dWy(rad/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
% legend('$dWy\ ideal$','$dWy\ estimation$', 'Interpreter', 'latex','FontSize',20)
% grid on
% subplot(3,1,3)
% plot(t_vec, true_state_vec(15,1:length(t_vec)),'k',t_vec, UKF_state(15,1:length(t_vec)),'b');
% title('2-D dWz Plot','FontSize',20);
% x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% y = ylabel('$dWz(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
% legend('$dWz\ ideal$','$dWz\ estimation$', 'Interpreter', 'latex','FontSize',20)
% grid on
% 
% E
figure(7)
plot(t_vec_select, UKF_state(16,1:length(t_vec_select)),'b',...
     t_vec_select, UKF_state(17,1:length(t_vec_select)),'c',...
     t_vec_select, UKF_state(18,1:length(t_vec_select)),'m',...
     t_vec_select, UKF_state(19,1:length(t_vec_select)),'y','LineWidth',2);
hold on;
plot(t_vec_select, 0.9*ones_vec,'--k',...
     t_vec_select, 0.7*ones_vec,'--k',...
     t_vec_select, 0.5*ones_vec,'--k',...
     t_vec_select, 0.3*ones_vec,'--k','LineWidth',2);
ylim([0 1.1]);
title('Estimates of the Motors Efficiency','FontSize',20);
x = xlabel('Time (seconds)', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
legend('$e^1$ estimation','$e^2$ estimation','$e^3$ estimation','$e^4$ estimation','ground truth', 'Interpreter', 'latex','FontSize',15)
grid on

% % ex_enu
% figure(8)
% subplot(3,1,1)
% plot(t_vec_select, quadrotor.ex_enu(1,1:length(t_vec_select)),'b','LineWidth',2);
% title('Position Errors','FontSize',20);
% y = ylabel('$e_{x_1}(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.11, 0.41]);
% grid on
% subplot(3,1,2)
% plot(t_vec_select, quadrotor.ex_enu(2,1:length(t_vec_select)),'b','LineWidth',2);
% y = ylabel('$e_{x_2}(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.11, 0.41]);
% grid on
% subplot(3,1,3)
% plot(t_vec_select, quadrotor.ex_enu(3,1:length(t_vec_select)),'b','LineWidth',2);
% x = xlabel('$Time(sec)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% y = ylabel('$e_{x_3}(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.11, 0.41]);
% grid on

% ex_norm 
figure
ex_norm = zeros(1,length(t_vec_select));
for i=1:length(t_vec_select)
    ex_norm(i) = norm(quadrotor.ex_enu(1:3,i));
end
plot(t_vec_select, ex_norm(1:length(t_vec_select)),'b','LineWidth',2);
title('Norm of Position Errors ','FontSize',20);
y = ylabel('(m)', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
x = xlabel('Time (seconds)', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
grid on

% % ev_enu
% figure(9)
% subplot(3,1,1)
% plot(t_vec, quadrotor.ev_enu(1,1:length(t_vec)),'b','LineWidth',2);
% title('Velocity Errors','FontSize',20);
% y = ylabel('$e_{v_1}(m/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
% grid on
% subplot(3,1,2)
% plot(t_vec, quadrotor.ev_enu(2,1:length(t_vec)),'b','LineWidth',2);
% y = ylabel('$e_{v_2}(m/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
% grid on
% subplot(3,1,3)
% plot(t_vec, quadrotor.ev_enu(3,1:length(t_vec)),'b','LineWidth',2);
% x = xlabel('$Time(sec)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% y = ylabel('$e_{v_3}(m/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
% grid on

% ev_norm 
figure
ev_norm = zeros(1,length(t_vec_select));
for i=1:length(t_vec_select)
    ev_norm(i) = norm(quadrotor.ev_enu(1:3,i));
end
plot(t_vec_select, ev_norm(1:length(t_vec_select)),'b','LineWidth',2);
title('Norm of Velocity Errors ','FontSize',20);
y = ylabel('(m/s)', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
x = xlabel('Time (seconds)', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
grid on

% % eR
% figure(10)
% subplot(3,1,1)
% plot(t_vec, quadrotor.eR(1,1:length(t_vec)),'b','LineWidth',2);
% title('Attitude Errors','FontSize',20);
% % x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% y = ylabel('$e_{R_1}(rad)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
% % legend('$error_R(1)$', 'Interpreter', 'latex','FontSize',15)
% grid on
% subplot(3,1,2)
% plot(t_vec, quadrotor.eR(2,1:length(t_vec)),'b','LineWidth',2);
% % title('2-D error_R(2) Plot','FontSize',20);
% % x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',15);
% y = ylabel('$e_{R_2}(rad)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
% % legend('$error_R(2)$', 'Interpreter', 'latex','FontSize',20)
% grid on
% subplot(3,1,3)
% plot(t_vec, quadrotor.eR(3,1:length(t_vec)),'b','LineWidth',2);
% % title('2-D error_R(3) Plot','FontSize',20);
% x = xlabel('$Time(sec)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% y = ylabel('$e_{R_3}(rad)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
% % legend('$error_R(3)$', 'Interpreter', 'latex','FontSize',20)
% grid on

% eR_norm 
figure
eR_norm = zeros(1,length(t_vec_select));
for i=1:length(t_vec_select)
    eR_norm(i) = norm(quadrotor.eR(1:3,i));
end
plot(t_vec_select, eR_norm(1:length(t_vec_select)),'b','LineWidth',2);
title('Norm of Attitude Errors ','FontSize',20);
y = ylabel('(rad)', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
x = xlabel('Time (seconds)', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
grid on

% % eW
% figure(11)
% subplot(3,1,1)
% plot(t_vec, quadrotor.eW(1,1:length(t_vec)),'b','LineWidth',2);
% title('Angular Velocity Errors','FontSize',20);
% y = ylabel('$e_{\Omega_1}(rad/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.10, 0.41]);
% grid on
% subplot(3,1,2)
% plot(t_vec, quadrotor.eW(2,1:length(t_vec)),'b','LineWidth',2);
% y = ylabel('$e_{\Omega_2}(rad/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.10, 0.41]);
% grid on
% subplot(3,1,3)
% plot(t_vec, quadrotor.eW(3,1:length(t_vec)),'b','LineWidth',2);
% x = xlabel('$Time(sec)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% y = ylabel('$e_{\Omega_3}(rad/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
% set(y, 'Units', 'Normalized', 'Position', [-0.10, 0.41]);
% grid on

% eW_norm 
figure
eW_norm = zeros(1,length(t_vec_select));
for i=1:length(t_vec_select)
    eW_norm(i) = norm(quadrotor.eW(1:3,i));
end
plot(t_vec_select, eW_norm(1:length(t_vec_select)),'b','LineWidth',2);
title('Norm of Angular Velocity Errors ','FontSize',20);
y = ylabel('(rad/s)', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
x = xlabel('Time (seconds)', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
grid on

% Thrust
figure
plot(t_vec(1000:length(t_vec_select)), ord_f1(1000:length(t_vec_select)),'b',...
     t_vec(1000:length(t_vec_select)), ord_f2(1000:length(t_vec_select)),'c',...
     t_vec(1000:length(t_vec_select)), ord_f3(1000:length(t_vec_select)),'m',...
     t_vec(1000:length(t_vec_select)), ord_f4(1000:length(t_vec_select)),'y','LineWidth',2);
y = ylabel('Thrusts(N)', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
ylim([0 12]);
hold on;
% plot(t_vec, 0.9*ones_vec,'--k',...
%      t_vec, 0.7*ones_vec,'--k',...
%      t_vec, 0.5*ones_vec,'--k',...
%      t_vec, 0.3*ones_vec,'--k','LineWidth',1);
% ylim([0 1.1]);
title('Thrust Command for the motors','FontSize',20);
x = xlabel('Time (Seconds)', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
legend('$f1$','$f2$','$f3$','$f4$', 'Interpreter', 'latex','FontSize',20)
grid on

%position comparison(traj,feedback,fault)
% x
figure
subplot(3,1,1)
plot(t_vec_select_geo, quadrotor_geo.pos_meas(1,1:length(t_vec_select_geo)),'-.r',...
     t_vec_select, quadrotor.pos_meas(1,1:length(t_vec_select)),'c',...
     t_vec_select, quadrotor.traj(1,1:length(t_vec_select)),'--k','LineWidth',2);
title('Position with and without Reconfiguration (Rec.)','FontSize',20);
y = ylabel('$x$(m)', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
legend('$x$ without Rec.','$x$ with Rec.','desire $x$', 'Interpreter', 'latex','FontSize',15,'Location','southeast')
grid on
subplot(3,1,2)
plot(t_vec_select_geo, quadrotor_geo.pos_meas(2,1:length(t_vec_select_geo)),'-.r',...
     t_vec_select, quadrotor.pos_meas(2,1:length(t_vec_select)),'c',...
     t_vec_select, quadrotor.traj(2,1:length(t_vec_select)),'--k','LineWidth',2);
y = ylabel('$y$(m)', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
legend('$y$ without Rec.','$y$ with Rec.','desire $y$', 'Interpreter', 'latex','FontSize',15,'Location','southeast')
grid on
subplot(3,1,3)
plot(t_vec_select_geo, quadrotor_geo.pos_meas(3,1:length(t_vec_select_geo)),'-.r',...
     t_vec_select, quadrotor.pos_meas(3,1:length(t_vec_select)),'c',...
     t_vec_select, quadrotor.traj(3,1:length(t_vec_select)),'--k','LineWidth',2);
x = xlabel('Time (Seconds)', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
y = ylabel('$z$(m)', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
legend('$z$ without Rec.','$z$ with Rec.','desire $z$', 'Interpreter', 'latex','FontSize',15,'Location','southeast')
grid on

