

% RECTIFY: compute rectification matrices
function [T1,T2,Pn1,Pn2] = rectify(Po1,Po2)

% factorize old PPMs
[A1,R1,t1] = art(Po1);
[A2,R2,t2] = art(Po2);

% optical centers (unchanged)
c1 = - inv(Po1(:,1:3))*Po1(:,4);
c2 = - inv(Po2(:,1:3))*Po2(:,4);
%disp("c1, c2:")
%disp(c1)
%disp(c2)
% new x axis (= direction of the baseline)
v1 = (c1-c2);
% new y axes (orthogonal to new x and old z)
v2 = cross(R1(3,:)',v1);
% new z axes (orthogonal to baseline and y)
v3 = cross(v1,v2);
%disp("v1,v2,v3:")
%disp(v1)
%disp(v2)
%disp(v3)

% new extrinsic parameters

R = [v1'/norm(v1)
v2'/norm(v2)
v3'/norm(v3)];
disp("R:")
disp(R)
% translation is left unchanged
% new intrinsic parameters (arbitrary)
A = (A1 + A2)./2;A(1,2)=0; 
% no skew
disp("A:")
disp(A)

% new projection matrices
Pn1=A*[R -R*c1 ];
Pn2=A*[R -R*c2 ];

% rectifying image transformation
T1 = Pn1(1:3,1:3)* inv(Po1(1:3,1:3));
T2 = Pn2(1:3,1:3)* inv(Po2(1:3,1:3));
end
% ------------------------
