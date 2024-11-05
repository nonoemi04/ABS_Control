clear all
%% sys id
 load('slipdata.mat');
x = ans(1, :);
y = ans(2, :);
st = ones(1, length(x));%step input
% Hfix = tf1;
Hf = tf(0.0154, [1, 0.1726, 0.0154]); % first id
subplot(211);plot(x, y);
subplot(212); step(Hf)

%% tuning noemia controller nr.1 distanta de franare = 26m
ovs = 0.03;
zeta = abs(log(ovs)) / sqrt(log(ovs) * log(ovs) + pi*pi); 
ts = 10;
wn = 4/ts/zeta;
Ho = tf(wn^2, [1 2*zeta*wn wn^2]); %cu tot cu controlleru
Hr = minreal((1 / Hf) * (Ho / (1 - Ho)));

coefs = pid(Hr);
subplot(311);plot(x, y);
subplot(312); step(Hf);subplot(313); step(Ho);
%P = 3.59; I = 0.361; D = 18.9; N = 1/1.25

%% tuning noemia controller nr.2 distanta de franare = 26m
ovs = 0.03;
zeta = abs(log(ovs)) / sqrt(log(ovs) * log(ovs) + pi*pi); 
ts = 20;
wn = 4/ts/zeta;
Ho = tf(wn^2, [1 2*zeta*wn wn^2]); %cu tot cu controlleru
Hr = minreal((1 / Hf) * (Ho / (1 - Ho)));
% P = 1.57, Ki = 0.18, Kd = 7.78; N=1/2.5
coefs = pid(Hr);
subplot(311);plot(x, y);
subplot(312); step(Hf);subplot(313); step(Ho);

%% crazy controller din nou

bode(Hf)
ovs = 0.03;
zeta = abs(log(ovs)) / sqrt(log(ovs) * log(ovs) + pi*pi); 
A = 1/4/sqrt(2)/zeta/zeta;
F = -2.79; % amplitudinea la wt=0.1241=[sqrt(0.0154)-partea de sus de la Hf]
N = mag2db(A);
Kp = db2mag(F-N); %Hrp(s)

Hp = Kp;
Hd = Kp*Hf;
Ho = feedback(Hd, 1)
hold; bode(Hd);


% pana aici am facut un P, de aici incolo facem PD cu d u din p alea ale noastre

wt = 0.179; % din bode Hd
ts = 1/wt/zeta^2;
%%
Kp = 4.18;
Td = 5.26;
Tn = 1/1.25;
%step(Hf); 
Hc = tf([Kp*Tn + Tn, Kp], [Tn, 1])
Ho = feedback(Hc*Hf, 1)
step(feedback(Hf,1)); hold;
step(Ho);

%% PHASE LEAD 1
T1 = 10;
T2 = 0.2;
kp = 2;
HrPD = tf([kp*T1, Kp],[T2,1])
step(feedback(Hf,1)); hold;
step(feedback(HrPD*Hf,1)); 
coef = pid(HrPD)
% P = 4.18; D = 19.2; N = 1/0.2

%% PHASE LEAD 2
T1 = 10;
T2 = 0.1;
kp = 2.5;
HrPD = tf([kp*T1, Kp],[T2,1])
step(feedback(Hf,1)); hold;
step(feedback(HrPD*Hf,1)); 
coef = pid(HrPD)
% P = 4.18; D = 24.6; N = 1/0.1


%% LQR

[A,B,C,D]=tf2ss(Hf.Numerator{1}, Hf.Denominator{1})

dim = size(A)
linii = dim(1)
coloane = dim(2)

% Q = [10 0; 0, 87];
Q = [10 0; 0, 300];
R = 0.01
k = lqr(A,B,Q,R);
kx = k(1);
ki  =k(2);
AA = A - B*k; %closed loop
BB = B * kx;
%% 
t = 0 : 80
u = ones(1,81)
[y,x,t] = step(AA,BB,C,D)
x2 = x(:,2);
sys=ss(AA,BB,C,D);
figure
step(Hf); hold on
step(sys)
