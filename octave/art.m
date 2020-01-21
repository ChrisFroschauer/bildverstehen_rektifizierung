
% ART: factorize a PPM as  P=A*[R;t]
function [A,R,t] = art(P)
Q = inv(P(1:3, 1:3));
[U,B] = qr(Q);

R = inv(U);
t = B*P(1:3,4);
A = inv(B);
A = A ./A(3,3);
end
