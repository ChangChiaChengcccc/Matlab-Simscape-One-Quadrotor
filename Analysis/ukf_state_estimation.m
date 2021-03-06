function xV = ukf_state_estimation(q)
    %% data 
    
    t_vec = q.t_vec;
    % initial state
    initial_state = [
                       q.pos_meas(:,1);q.vel_meas(:,1);q.acc_meas(:,1); q.W_meas(:,1); q.dW_meas(:,1); 0; 0; 0; 0
                    ];

    true_state_vec = [
                        q.pos_meas; q.vel_meas; q.acc_meas; q.W_meas; q.dW_meas; ...
                        q.E(1)*ones(1,length(t_vec)); q.E(2)*ones(1,length(t_vec)); ...
                        q.E(3)*ones(1,length(t_vec)); q.E(4)*ones(1,length(t_vec))
                     ];

    measurement_vec = [
                        q.pos_meas; q.W_meas
                      ]; %+ 0.1*randn(length(initial_measurement),length(t_vec)); 
    
    %% ukf
    n=length(initial_state);      %number of state

    %std of process 
    q_std=[
          1e-3 1e-3 1e-3 ... %x  
          1e-3 1e-3 1e-3 ... %v 
          1e-3 1e-3 1e-3 ... %a
          1e-3 1e-3 1e-3 ... %W
          1e-3 1e-3 1e-3 ... %dW
          1e-3 1e-3 1e-3 1e-3 ...%E
      ];    
    %std of measurement
    r=[
         1e-3 1e-3 1e-3 ... %x 
         1e-2 1e-2 1e-2 ... %W
      ];    

    Q=diag(q_std)*diag(q_std);       % covariance of process
    R=diag(r)*diag(r);       % covariance of measurement  

    f=@(x,q,i,dt) [                       % x
                                            x(1)+x(4)*dt+0.5*x(7)*dt^2;
                                            x(2)+x(5)*dt+0.5*x(8)*dt^2;
                                            x(3)+x(6)*dt+0.5*x(9)*dt^2;   
                                            % v
                                            x(4)+x(7)*dt;
                                            x(5)+x(8)*dt;
                                            x(6)+x(9)*dt;
                                            % a = fRe3/m - ge3
                                            cal_acc_state(q,i,x);
                                            % W
                                            x(10)+x(13)*dt;
                                            x(11)+x(14)*dt;
                                            x(12)+x(15)*dt;
                                            % dW = inv(J)*(M - WxJW) 
                                            cal_dW_state(q,i,x);
                                            % E
                                            x(16);
                                            x(17);
                                            x(18);
                                            x(19)
                                           ];
                                       
    % measurement equation
    h=@(x,q,i,dt) [
                                            %x
                                            x(1);
                                            x(2);
                                            x(3);
                                            %W
                                            x(10);
                                            x(11);
                                            x(12);
                                            ];              
    x=initial_state;           
    P = eye(n);                                        % initial state covraiance
    N= length(t_vec);                                  % total dynamic steps
    xV = zeros(n,N);                                   % allocate memory
    for k=1:(N-1)
      z = measurement_vec(:,k);                % measurments
      [x, P] = ukf(f,x,P,h,z,Q,R,q,k);            % ukf 
      xV(:,k) = x;                            % save estimate
    end
    %% plot
    % x
    figure(1)
    subplot(3,1,1)
    plot(t_vec, true_state_vec(1,:),t_vec, xV(1,:),'b');
    title('2-D x Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$x(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$x\ ideal$','$x\ estimation$', 'Interpreter', 'latex','FontSize',20)
    grid on
    subplot(3,1,2)
    plot(t_vec, true_state_vec(2,:),'k',t_vec, xV(2,:),'b');
    title('2-D y Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$y(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$y\ ideal$','$y\ estimation$', 'Interpreter', 'latex','FontSize',20)
    grid on
    subplot(3,1,3)
    plot(t_vec, true_state_vec(3,:),'k',t_vec, xV(3,:),'b');
    title('2-D z Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$z(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$z\ ideal$','$z\ estimation$', 'Interpreter', 'latex','FontSize',20)
    grid on
    % v
    figure(2)
    subplot(3,1,1)
    plot(t_vec, true_state_vec(4,:),'k',t_vec, xV(4,:),'b');
    title('2-D vx Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$vx(m/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$vx\ ideal$','$vx\ estimation$', 'Interpreter', 'latex','FontSize',20)
    grid on
    subplot(3,1,2)
    plot(t_vec, true_state_vec(5,:),'k',t_vec, xV(5,:),'b');
    title('2-D vy Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$vy(m/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$vy\ ideal$','$vy\ estimation$', 'Interpreter', 'latex','FontSize',20)
    grid on
    subplot(3,1,3)
    plot(t_vec, true_state_vec(6,:),'k',t_vec, xV(6,:),'b');
    title('2-D vz Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$vz(m/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$vz\ ideal$','$vz\ estimation$', 'Interpreter', 'latex','FontSize',20)
    grid on
    % a
    figure(3)
    subplot(3,1,1)
    plot(t_vec, true_state_vec(7,:),'k',t_vec, xV(7,:),'b');
    title('2-D ax Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$ax(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$ax\ ideal$','$ax\ estimation$', 'Interpreter', 'latex','FontSize',20)
    grid on
    subplot(3,1,2)
    plot(t_vec, true_state_vec(8,:),'k',t_vec, xV(8,:),'b');
    title('2-D ay Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$ay(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$ay\ ideal$','$ay\ estimation$', 'Interpreter', 'latex','FontSize',20)
    grid on
    subplot(3,1,3)
    plot(t_vec, true_state_vec(9,:),'k',t_vec, xV(9,:),'b');
    title('2-D az Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$az(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$az\ ideal$','$az\ estimation$', 'Interpreter', 'latex','FontSize',20)
    grid on
    % W 
    figure(5)
    subplot(3,1,1)
    plot(t_vec, true_state_vec(10,:),'k',t_vec, xV(10,:),'b');
    title('2-D wx Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$wx(rad/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$wx\ ideal$','$wx\ estimation$', 'Interpreter', 'latex','FontSize',20)
    grid on
    subplot(3,1,2)
    plot(t_vec, true_state_vec(11,:),'k',t_vec, xV(11,:),'b');
    title('2-D wy Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$wy(rad/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$wy\ ideal$','$wy\ estimation$', 'Interpreter', 'latex','FontSize',20)
    grid on
    subplot(3,1,3)
    plot(t_vec, true_state_vec(12,:),'k',t_vec, xV(12,:),'b');
    title('2-D wz Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$wz(rad/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$wz\ ideal$','$wz\ estimation$', 'Interpreter', 'latex','FontSize',20)
    grid on
    % dW
    figure(6)
    subplot(3,1,1)
    plot(t_vec, true_state_vec(13,:),'k',t_vec, xV(13,:),'b');
    title('2-D dWx Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$dWx(rad/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$dWx\ ideal$','$dWx\ estimation$', 'Interpreter', 'latex','FontSize',20)
    grid on
    subplot(3,1,2)
    plot(t_vec, true_state_vec(14,:),'k',t_vec, xV(14,:),'b');
    title('2-D dWy Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$dWy(rad/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$dWy\ ideal$','$dWy\ estimation$', 'Interpreter', 'latex','FontSize',20)
    grid on
    subplot(3,1,3)
    plot(t_vec, true_state_vec(15,:),'k',t_vec, xV(15,:),'b');
    title('2-D dWz Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$dWz(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$dWz\ ideal$','$dWz\ estimation$', 'Interpreter', 'latex','FontSize',20)
    grid on
    % E
    figure(7)
    plot(t_vec, xV(16,:),'b',t_vec, xV(17,:),'c',t_vec, xV(18,:),'m',t_vec, xV(19,:),'y', ...
           t_vec, q.E(1)*ones(1,length(t_vec)),'k', t_vec, q.E(2)*ones(1,length(t_vec)),'k', ...
           t_vec, q.E(3)*ones(1,length(t_vec)),'k', t_vec, q.E(4)*ones(1,length(t_vec)),'k');
    ylim([0 2]);
    title('2-D E Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$Efficiency$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$e1\ estimation$','$e2\ estimation$','$e3\ estimation$','$e4\ estimation$', 'Interpreter', 'latex','FontSize',15)
    grid on
end
