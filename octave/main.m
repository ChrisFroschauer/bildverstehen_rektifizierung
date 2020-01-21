
% a = [1 2 3; 4 5 6; 7 8 10]

%Projection Matrix 1 PPM1: 
PPM1 =  [-820.9809842647142, 184.2182273632835, 541.2270896040269, 297.8836542999622;
  -5.861447464327993, -626.5469693685785, 648.2870825940317, 237.8146008448999;
  -0.02655195525547172, 0.330191152363302, 0.9435405643495703, 0.4219857032493103]%Projection Matrix 2 PPM2: 
PPM2 = [-784.139169323909, 262.6948308986451, 563.0187237468371, 438.9005201336228;
  25.15067928006231, -546.409852620379, 716.7114615664634, 296.3523474088649;
  0.03508731357570102, 0.4384565889552385, 0.8980672024006767, 0.4438649486484741];
 
[T1, T2, Pn1, Pn2] = rectify(PPM1, PPM2);
T1
T2
Pn1
Pn2