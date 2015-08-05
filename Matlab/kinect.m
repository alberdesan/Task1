function [Oa,Ea]=kinect(x,y,xant,yant,Spred,sig)

Oa=[0 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0];
Ea=[0 0 0 0]';

% Measurement covariance matrix
R = [  1  0  0  0  
       0  1  0  0  
       0  0  1  0  
       0  0  0  1];

R=R*sig*sig;

% Jacobian

H = [1 0 0 0
     0 1 0 0
     0 0 1 0
     0 0 0 1];
 
% Measurements
P(1)  = x;
P(2)  = y;
P(3)  = xant;
P(4)  = yant;
  
Salida_predicha(1) = Spred(1);
Salida_predicha(2) = Spred(2);
Salida_predicha(3) = Spred(3);
Salida_predicha(4) = Spred(4);
Oa=H'*inv(R)*H;
Ea=H'*inv(R)*(P' - Salida_predicha' + H*Spred);   

end
