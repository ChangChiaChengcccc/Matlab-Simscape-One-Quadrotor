function  dW_state = cal_dW_state(q,i,x)
    ConIn_E =  q.allo*diag(x(16:19))*q.f_vec(:,i);
    M_E = ConIn_E(2:4);
    % dW = inv(J)*(M - WxJW) 
    dW_state = inv(q.J)*(M_E -cross(q.W_meas(:,i),q.J*q.W_meas(:,i)));
end