clc; 
M = 2.7582989593438584919;  %% Mass of cart (kg)
 m = 0.67500000000000004441;  %% Mass of weight (kg)
 l = 0.5;  %% Length of pendulum (m)
 g = 9.81; %% Gravitational acceleration (m/s^2)


 A = [ 0  1  0  0;
       0  0  -((m*g)/(M))  0;
       0  0  0  1;
       0  0  -(((M+m)*g)/(l*M))  0;
      ];

 B = [ 0;
       (1/M);
       0;
       ((1)/(l*M));
      ];



Q = [ 20   0     0     0;
      0       200   0     0;
      0       0     20    0;
      0       0      0    2;
      ];

R = [1];

[K,P,E] = lqr(A,B,Q,R)