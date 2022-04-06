classdef quadcopter_data
    properties
        m = 0.952;
        J = [3.29025*1e-3 0 0;
             0 3.29025*1e-3 0;
             0 0 5.815*1e-3];
        allo = [
                1 1 1 1;
                0.1592 -0.1592 0.1592 -0.1592;
                -0.1592 -0.1592 0.1592 0.1592;
                -2e-2 2e-2 2e-2 -2e-2
                ];
        R_w2b
        ConIn
        f1
        f2
        f3
        f4
        f
        M
        f_vec
        pos_meas
        vel_meas
        acc_dyn
        acc_meas
        W_meas
        dW_meas
        dW_dyn
        E = [1;1;1;1];
        e3 = [0;0;1];
        g = 9.80665;
        steps
        t_vec
        states
    end
end