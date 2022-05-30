function [x,P,dt]=ukf(x,P,z,f_vec,R_w2b,q,i)

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

L=numel(x);                                 %numer of states
m=numel(z);                                 %numer of measurements
alpha=1e-3;                                 %default, tunable
ki=0;                                       %default, tunable
beta=2;                                     %default, tunable
lambda=alpha^2*(L+ki)-L;                    %scaling factor
c=L+lambda;                                 %scaling factor
Wm=[lambda/c 0.5/c+zeros(1,2*L)];           %weights for means
Wc=Wm;
Wc(1)=Wc(1)+(1-alpha^2+beta);               %weights for covariance
c=sqrt(c);
X=sigmas(x,P,c);                            %sigma points around x
[x1,X1,P1,X2,dt]=ut_f(X,Wm,Wc,L,Q,f_vec,R_w2b,q,i);          %unscented transformation of process
% X1=sigmas(x1,P1,c);                         %sigma points around x1
% X2=X1-x1(:,ones(1,size(X1,2)));             %deviation of X1
[z1,Z1,P2,Z2]=ut_h(X1,Wm,Wc,m,R,f_vec,q,i);       %unscented transformation of measurments
P12=X2*diag(Wc)*Z2';                        %transformed cross-covariance
K=P12*inv(P2);
x=x1+K*(z-z1);                              %state update
P=P1-K*P12';                                %covariance update

function [y,Y,P,Y1,dt]=ut_f(X,Wm,Wc,n,R,f_vec,R_w2b,q,i)
L=size(X,2);
y=zeros(n,1);
Y=zeros(n,L);
dt = q.t_vec(i)- q.t_vec(i-1);
for k=1:L                   
    Y(:,k)=f(X(:,k),f_vec,R_w2b,i,dt);       
    y=y+Wm(k)*Y(:,k);       
end
Y1=Y-y(:,ones(1,L));
P=Y1*diag(Wc)*Y1'+R;     

function [y,Y,P,Y1]=ut_h(X,Wm,Wc,n,R,f_vec,q,i)
L=size(X,2);
y=zeros(n,1);
Y=zeros(n,L);
dt = q.t_vec(i)- q.t_vec(i-1);
for k=1:L                   
    Y(:,k)=h(X(:,k),f_vec,i,dt);       
    y=y+Wm(k)*Y(:,k);       
end
Y1=Y-y(:,ones(1,L));
P=Y1*diag(Wc)*Y1'+R;

function X=sigmas(x,P,c)
%Sigma points around reference point
%Inputs:
%       x: reference point
%       P: covariance
%       c: coefficient
%Output:
%       X: Sigma points

A = c*chol(P)';
Y = x(:,ones(1,numel(x)));
X = [x Y+A Y-A]; 

function predict_state=f(x,f_vec,R_w2b,i,dt)
% x
predict_state = zeros(19,1);
predict_state(1) = x(1)+x(4)*dt+0.5*x(7)*dt^2;
predict_state(2) = x(2)+x(5)*dt+0.5*x(8)*dt^2;
predict_state(3) = x(3)+x(6)*dt+0.5*x(9)*dt^2;   
% v
predict_state(4) = x(4)+x(7)*dt;
predict_state(5) = x(5)+x(8)*dt;
predict_state(6) = x(6)+x(9)*dt;
% a = fRe3/m - ge3
predict_state(7:9) = cal_acc_state(f_vec,R_w2b,i,x);
% W
predict_state(10) = x(10)+x(13)*dt;
predict_state(11) = x(11)+x(14)*dt;
predict_state(12) = x(12)+x(15)*dt;
% dW = inv(J)*(M - WxJW) 
predict_state(13:15) = cal_dW_state(f_vec,i,x);
% E
predict_state(16) = x(16);
predict_state(17) = x(17);
predict_state(18) = x(18);
predict_state(19) = x(19);

function  acc_state = cal_acc_state(f_vec,R_w2b,i,x)
allo_mat = [ 1 1 1 1;
            0.1592 -0.1592 0.1592 -0.1592;
            -0.1592 -0.1592 0.1592 0.1592;
            -2e-2 2e-2 2e-2 -2e-2];
m = 0.952;
e3 = [0;0;1];
g = 9.80665;
ConIn_E =  allo_mat*diag(x(16:19))*f_vec;
f_E = ConIn_E(1);
% a = fRe3/m - ge3
acc_state = f_E*inv(R_w2b)*e3/m - g*e3;

function  dW_state = cal_dW_state(f_vec,i,x)
allo_mat = [ 1 1 1 1;
            0.1592 -0.1592 0.1592 -0.1592;
            -0.1592 -0.1592 0.1592 0.1592;
            -2e-2 2e-2 2e-2 -2e-2];
J = [3.29025*1e-3 0 0;
     0 3.29025*1e-3 0;
     0 0 5.815*1e-3];
ConIn_E =  allo_mat*diag(x(16:19))*f_vec;
M_E = ConIn_E(2:4);
% dW = inv(J)*(M - WxJW) 
dW_state = inv(J)*(M_E -cross(x(10:12),J*x(10:12)));
                                       
% measurement equation
function correct_state=h(x,q,i,dt) 
%x
correct_state = zeros(6,1);
correct_state(1) = x(1);
correct_state(2) = x(2);
correct_state(3) = x(3);
%W
correct_state(4) = x(10);
correct_state(5) = x(11);
correct_state(6) = x(12);
                                            