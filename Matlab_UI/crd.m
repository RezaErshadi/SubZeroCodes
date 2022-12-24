clear
close all
clc
%%
E = referenceEllipsoid("wgs84","meter");
A23 = [-71.64611,-8.46492];
A22 = [-71.60575,-8.40263];
WE4E = [-71.64119,-8.39867];
ACRS24W = [-71.61888,-8.53111];


CrossConj = [-71.633630,-8.444874];

[DST_A23_A22,AZ_A23_A22] = distance(A23,A22,E);
[DST_4E_24W,AZ_4E_24W] = distance(WE4E,ACRS24W,E);
[DST_24W_A23,AZ_24W_A23] = distance(ACRS24W,A23,E);
[DST_A23_CrossConj,AZ_A23_CrossConj] = distance(A23,CrossConj,E);

depo = [-71.62231,-8.46259];
jhdest = [-71.60965,-8.41040];
now = [-71.6167647, -8.4211873];
[a,b] = distance(now,jhdest,E);

[latout,lonout] = reckon(A23(1),A23(2),2500,AZ_A23_A22,E,"degree");
RoverDepo = [latout,lonout];


% fprintf('from 24W to A23: dist:%.1f Az:%.3f \n',DST_24W_A23,AZ_24W_A23)
% fprintf('from A23 to Depo: dist:%.1f Az:%.3f \n',2500,AZ_A23_A22)
fprintf('JH: dist:%.1f Az:%.3f \n',a,b)
