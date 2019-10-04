function [u,v] = solvefisheyedistortion(Xc,Yc,Zc,a0,a2,a3,a4)

Xc=XcYcZc(1,2);
Yc=XcYcZc(2,2);
Zc=XcYcZc(3,2);
a0=1000.08; a2=-1.02e-3; a3=-3.82e-06; a4=1.71e-10;

syms u v
p = sqrt(u^2+v^2);
F = a0+a2*p^2+a3*p^3+a4*p^4;
assume(Xc/u,'positive')
assume(Yc/v,'positive')
assume(Zc/F,'positive')
eqn = [Xc*F-Zc*u == 0, Yc*F-Zc*v == 0];
vars = [u v];
sol = solve(eqn,vars,'MaxDegree',4,'Real',true);

end