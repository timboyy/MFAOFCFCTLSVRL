function [x_01,wa_0] = diedai(A,B,x_01,wa_0,wc_0,R,w)
    x_01=A*x_01+B*(wa_0'*x_01)+B*w;
    ea_01=R^-1*wc_0'*x_01;
    wa_detla=-2*x_01*ea_01';
    wa_0=wa_0+wa_detla;
end