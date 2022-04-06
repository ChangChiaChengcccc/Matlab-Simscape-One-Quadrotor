function  acc_state = cal_acc_state(q,i,x)
    ConIn_E =  q.allo*diag(x(16:19))*q.f_vec(:,i);
    f_E = ConIn_E(1);
    % a = fRe3/m - ge3
    acc_state = f_E*inv(q.R_w2b(:,:,i))*q.e3/q.m - q.g*q.e3;
end