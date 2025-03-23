function [A0,B0] = discreet(A,B,T)
[m1,m2]=size([A,B]);
m0=zeros(max(m1,m2));
m0(1:m1,1:m2)=[A,B];
m0=expm(m0*T);
[a1,a2]=size(A);
[b11,b12]=size(B);
A0=m0(1:a1,1:a2);
B0=m0(1:a1,a2+1:a2+b12);

end