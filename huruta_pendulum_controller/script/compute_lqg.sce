
m1=0.154
m2=0.076//m1
J1zz=0.000194//I0
J2zz=0.000161
J2xx=0.000265//J1
J2yy=0.000105
l1=0.00111//
l2=0.03423//i1
L1=0.05588//L0
g = 9.8
b1 = 0.001
b2 = 0.001


J1_hat = J1zz + m1*l1**2
J2_hat = 0.5*(J2zz+J2yy) + m2*l2**2
J0_hat = J1_hat+m2*L1**2
A31 = 0
A32 = g*(m2**2)*(l2**2)*L1/(J0_hat*J2_hat-(m2**2)*(L1**2)*(l2**2))
A33 = -b1*J2_hat/(J0_hat*J2_hat-(m2**2)*(L1**2)*(l2**2))
A34 = -b2*m2*l2*L1/(J0_hat*J2_hat-(m2**2)*(L1**2)*(l2**2))
A41 = 0
A42 = g*m2*l2*J0_hat/(J0_hat*J2_hat-(m2**2)*(L1**2)*(l2**2))
A43 = -b1*m2*l2*L1/(J0_hat*J2_hat-(m2**2)*(L1**2)*(l2**2))
A44 = -b2*J0_hat/(J0_hat*J2_hat-(m2**2)*(L1**2)*(l2**2))
B31 = J2_hat/(J0_hat*J2_hat-(m2**2)*(L1**2)*(l2**2))
B41 = m2*L1*l2/(J0_hat*J2_hat-(m2**2)*(L1**2)*(l2**2))
B32 = m2*L1*l2/(J0_hat*J2_hat-(m2**2)*(L1**2)*(l2**2))
B41 = J0_hat/(J0_hat*J2_hat-(m2**2)*(L1**2)*(l2**2))

A = [0 0 1 0
     0 0 0 1
     A31 A32 A33 A34
     A41 A42 A43 A44]
B = [0
     0
     B31
     B41]
C = [1 0 0 0
     0 1 0 0]
D = [0
     0]
x0 = [0
      3.14
      0
      0]
sl = syslin('c', A, B, C, D,x0)
sl2 = dscr(sl, 0.001)

V = [0.01 0
     0 0.01]
W = [0.01 0 0 0
     0 0.01 0 0
     0 0 0.01 0
     0 0 0 0.01]
R = [1]
Q = [1 0 0 0
     0 1 0 0
     0 0 1 0
     0 0 0 1]
bigQ = blockdiag(Q,R)
bigR = blockdiag(W,V)
[P,r] = lqg2stan(sl2,bigQ,bigR)
[K]=lqg(P,r)
disp(K)

