function [K]=mydlqr(A,B,Q,R)
% This function computes de control gain K, such that
% u=-K*x stabilizes the system x(k+1)=A*x+B*u
% The matrices Q and R have to be diagonal and positive definite.
% John J. Martinez, Grenoble-INP, July 2016

P=Q; %
for i=1:2500
    P=A'*P*A+Q-A'*P*B*inv(B'*P*B+R)*B'*P*A;
end
K=inv(B'*P*B+R)*B'*P*A;
