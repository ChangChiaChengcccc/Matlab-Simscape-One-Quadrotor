function plotq(q)
% acceleration
    figure(1)
subplot(3,1,1)
    plot(q.t_vec, q.acc_dyn(1, :),'b',q.t_vec, q.acc_meas(1,:),'k');
    title('2-D Compare ax Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$ax(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$ax\ dynamics$','$ax\ meas$', 'Interpreter', 'latex','FontSize',20)
    ylim([-1 1])
    grid on
subplot(3,1,2)
    plot(q.t_vec, q.acc_dyn(2, :),'b',q.t_vec, q.acc_meas(2,:),'k');
    title('2-D Compare ay Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$ay(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$ay\ dynamics$','$ay\ meas$', 'Interpreter', 'latex','FontSize',20)
    ylim([-1 1])
    grid on
subplot(3,1,3)
    plot(q.t_vec, q.acc_dyn(3, :),'b',q.t_vec, q.acc_meas(3,:),'k');
    title('2-D Compare az Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$az(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$az\ dynamics$','$az\ meas$', 'Interpreter', 'latex','FontSize',20)
    xlim([0 70])
    ylim([-1 5])
    grid on
% angular acceleration
    figure(2)
subplot(3,1,1)
    plot(q.t_vec, q.dW_dyn(1,:),'b',q.t_vec, q.dW_meas(1,:),'k');
    title('2-D Compare dWx Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$dWx(rad/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$dWx\ dynamics$','$dWx\ meas$', 'Interpreter', 'latex','FontSize',20)
    ylim([-0.5 0.5])
    grid on
subplot(3,1,2)
    plot(q.t_vec, q.dW_dyn(2,:),'b',q.t_vec, q.dW_meas(2,:),'k');
    title('2-D Compare dWy Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$dWy(rad/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$dWy\ dynamics$','$dWy\ meas$', 'Interpreter', 'latex','FontSize',20)
    ylim([-0.5 0.5])
    grid on
subplot(3,1,3)
    plot(q.t_vec, q.dW_dyn(3,:),'b',q.t_vec, q.dW_meas(3,:),'k');
    title('2-D Compare dWz Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$dWz(rad/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$dWz\ dynamics$','$dWz\ meas$', 'Interpreter', 'latex','FontSize',20)
    ylim([-0.5 0.5])
    grid on
% angular velocity
    figure(3)
subplot(3,1,1)
    plot(q.t_vec, q.W_meas(1,:),'k');
    title('2-D Wx Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$Wx(rad/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$Wx\ meas$', 'Interpreter', 'latex','FontSize',20)
    ylim([-0.5 0.5])
    grid on
subplot(3,1,2)
    plot(q.t_vec, q.W_meas(2,:),'k');
    title('2-D Wy Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$Wy(rad/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$Wy\ meas$', 'Interpreter', 'latex','FontSize',20)
    ylim([-0.5 0.5])
    grid on
subplot(3,1,3)
    plot(q.t_vec, q.W_meas(3,:),'k');
    title('2-D Wz Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$Wz(rad/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$Wz\ meas$', 'Interpreter', 'latex','FontSize',20)
    xlim([0 70])
    ylim([-0.5 0.5])
    grid on
end