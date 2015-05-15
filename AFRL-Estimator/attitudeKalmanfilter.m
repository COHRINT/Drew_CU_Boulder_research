function [ eulerAngles,Rot_matrix,x_aposteriori,P_aposteriori ] = attitudeKalmanfilter(dt,z,x_aposteriori_k,P_aposteriori_k)
q = ones(4,1) * .1^2;
r = ones(3,1) * .1^2;

Q = diag(q.^2*dt);
wx= x_aposteriori_k(1);
wy= x_aposteriori_k(2);
wz= x_aposteriori_k(3);
wax= x_aposteriori_k(4);
way= x_aposteriori_k(5);
waz= x_aposteriori_k(6);
zex= x_aposteriori_k(7);
zey= x_aposteriori_k(8);
zez= x_aposteriori_k(9);
mux= x_aposteriori_k(10);
muy= x_aposteriori_k(11);
muz= x_aposteriori_k(12);

%%PROPAGATION
%Body angular accelerations
wak = [wax;way;waz];
wk = [wx;wy;wz] + dt*wak;
%derivative of prediction rotation matrix
O = [0,-wz,wy;wz,0,-wx;-wy,wx,0]';
%prediction of the earth z vector
zek = (eye(3) + O*dt)*[zex;zey;zez];
%prediction of the magnetic vector
muk = (eye(3)+O*dt)*[mux;muy;muz];

EZ = [0,zez,-zey;-zez,0,zex;zey,-zex,0]';
MA = [0,muz,-muy;-muz,0,mux;zey,-mux,0]';
E  = eye(3);
Z  = zeros(3);
x_apriori = [wk;wak;zek;muk];

A_lin = [Z,E,Z,Z;Z,Z,Z,Z;EZ,Z,O,Z;MA,Z,Z,O];
A_lin = eye(12)+A_lin*dt;

Qtemp = diag([q(1),q(1),q(1),q(2),q(2),q(2),q(3),q(3),q(3),q(4),q(4),q(4)]);
Q = A_lin*Qtemp*A_lin';
P_apriori = A_lin*P_aposteriori_k*A_lin'+Q;

%%UPDATE

if z(6)<4 || z(5)>15
    r(2)=10000;
end

R = diag([r(1),r(1),r(1),r(2),r(2),r(2),r(3),r(3),r(3)]);
H_k =[E,Z,Z,Z;Z,Z,E,Z;Z,Z,Z,E];
y_k=z'-H_k*x_apriori;
S_k=H_k*P_apriori*H_k'+R;
K_k=P_apriori*H_k'/S_k;

x_aposteriori = x_apriori+K_k*y_k;
P_aposteriori = (eye(12)-K_k*H_k)*P_apriori;

%Euler angle extraction
z_n_b = -x_aposteriori(7:9)./norm(x_aposteriori(7:9)); %Normalized north vector
m_n_b = x_aposteriori(10:12)./norm(x_aposteriori(10:12)); %Normalized down vector
y_n_b = cross(z_n_b,m_n_b); 
y_n_b = y_n_b./norm(y_n_b); %Calculated east vector
x_n_b = cross(y_n_b,z_n_b);
x_n_b = x_n_b./norm(x_n_b);

Rot_matrix = [x_n_b,y_n_b,z_n_b];
phi = atan2(Rot_matrix(2,3),Rot_matrix(3,3));
theta = -asin(Rot_matrix(1,3));
psi = atan2(Rot_matrix(1,2),Rot_matrix(1,1));
eulerAngles = [phi,theta,psi];
end

