%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  IWM車のセミアクティブサスペンション  2自由度 非線形H∞制御
%  清水 第5章 Type1（オブザーバ付き出力フィードバック）式(5.40),(5.41)
%  Simulink: non_linear_H_inf_and_compare.slx 対応版
%  2022 original by Ravi / 2026 修正版
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all

%% ===== 物理パラメータ =====
m1 = 50;        % [kg]    ばね下質量（インホイールモータ込み）
k1 = 160000;    % [N/m]   タイヤ剛性
m2 = 300;       % [kg]    ばね上質量
k2 = 16000;     % [N/m]   サスペンションばね
c2 = 1500;      % [Ns/m]  パッシブダンパ


%% ===== 設計パラメータ =====
kW   = 10;      % 評価出力の重みの倍率（10, 30, 100 などで比較）
gam1 = 1.5;     % オブザーバの γ1
mw1  = 0;       % 非線形重み m1(x̂)（式(5.39),(5.41)）。0 で線形 H∞ と同じ、>0 で状態が大きいときハイゲイン
cu   = 1;       % 入力の正規化スケール u = cu*u_n [Ns/m]（下の自動調整ループで決める）
c_target = 4000;% 目標とする正の c 指令の中央値 [Ns/m]
cmax = 8000;    % 可変ダンパーの最大減衰 [Ns/m]

%% ===== ランダム路面（ISO Class 3, 60 km/h）=====
T  = 100;  dt = 0.001;  v = 60/3.6;  Nt = T/dt;
rng(0,'twister');
th = 2*pi*rand(1,Nt/2+1);
df = 1/(Nt*dt);
f  = 0:df:1/(2*dt);              % 時間周波数 [Hz]
Fs = f/v;                        % 空間周波数 [1/m]
Sn0 = [16 64 256 1024 4096 16384 65536 262144];
RoadClass = 3;

P1 = zeros(size(f));
ok = (Fs >= 0.001) & (Fs <= 100);
P1(ok) = Sn0(RoadClass)*1e-6*(0.1./Fs(ok)).^2;
p  = P1/v;

ck = zeros(1,Nt);
for ii = 1:Nt
    if ii <= Nt/2+1
        ck(ii) = sqrt(p(ii)/(2*Nt*dt))*exp(1j*th(ii));
    else
        ck(ii) = conj(ck(Nt-ii+2));
    end
    if ii == 1 || ii == Nt/2+1
        ck(ii) = real(sqrt(p(ii)/(Nt*dt))*exp(1j*th(ii)));
    end
end
zw  = real(ifft(ck))*Nt;         % 路面変位 [m]
X0d = diff([0 zw])/dt;           % 路面速度 [m/s]
dx  = X0d(10001:90001);          % 15 秒分

%% ===== プラント  xp = [z1-z0, z2-z1, z1dot, z2dot] =====
cmin = 300;                                  % 可変ダンパーの最小減衰（H∞ 側の固定分）
mk   = @(cd) [0 0 1 0; 0 0 -1 1; -k1/m1 k2/m1 -cd/m1 cd/m1; 0 -k2/m2 cd/m2 -cd/m2];
Ap      = mk(cmin);                          % H∞ 設計・シミュレーション用
Ap_pass = mk(c2);                            % 比較用 passive（1500）
Ap_c300  = mk(cmin);                         % 比較用 passive（300 = u=0 と同じ）
Ap_c8000 = mk(8000);                         % 比較用 passive（8000）
Cp1 = [Ap(4,:); 0 -1 0 0; 0 0 0 1];          % 車体加速度 / ストローク / 車体速度
% Cp2 = [0 1 0 0; Ap(4,:)];                    % 観測：ストローク / 車体加速度
Cp2 = [0 1 0 0;            % ストローク z2 - z1
    Ap(4,:);            % 車体加速度（追加力の分を補正した値）
    0 0 1 -1];          % ストローク速度 z1dot - z2dot（追加）

Bp1  = [-1; 0; 0; 0];                % w = z0dot
Bp2  = [0; 0; -1/m1; 1/m2];          % 力 F = u*(z1dot - z2dot)  ← u は追加の減衰係数
% Dp21 = [0; 0];
Dp21 = [0; 0; 0];          % 3 出力に合わせる



%% ===== 評価出力に対する重み Ws =====
wa = 2*pi*3.0;  wb = 2*pi*3.3;  a = 3.5;  b = a*900;
Ws1 = tf([1 2*b*wb wb^2]*10^-4.2, [1 2*a*wa wa^2]);
wa = 2*pi*5.2;  wb = 2*pi*5.2;  a = 0.02; b = a*0.4;
Ws2 = tf([1 2*b*wb wb^2]*10^-4.3, [1 2*a*wa wa^2]);
wb = 2*pi*0.001; wa = 2*pi*25;  b = 1.5;  a = 0.65;


lp  = @(k, f0) tf(k*(2*pi*f0)^2, [1 2*0.7*(2*pi*f0) (2*pi*f0)^2]);
kv  = 0.14;                 % 0.05 → 0.2, 0.5 で比較
Ws3 = lp(kv, 1);           % 帯域を 0〜2 Hz に絞って、ばね上共振に集中させる
[Aws,Bws,Cws,Dws] = ssdata(ss(blkdiag(Ws1,Ws2,Ws3)));


%% ===== 制御入力に対する重み Wts =====
wo = 2*pi*3.0;  w1 = 2*pi*3.3;  a = 3.5;  b = a*1100;
Wts1 = tf([1 2*b*w1 w1^2]*1e-5, [1 2*a*wo wo^2]);
wo = 2*pi*2.3;  w1 = 2*pi*2.3*1.3;  a = 0.1;  b = 0.04;
Wts2 = tf([1 2*b*wo wo^2]*1e-4, [1 2*a*w1 w1^2]);
wb = 2*pi*5.1;  wa = 2*pi*12;  b = 3.5;  a = 3.65;
Wts3 = tf([1 2*b*wb wb^2]*1e-5, [1 2*a*wa wa^2]);
[Ats,Bts,Cts,Dts] = ssdata(ss(blkdiag(Wts1,Wts2,Wts3)));


%% ===== 一般化プラント（16 状態 = 4 + 6 + 6）=====
np = size(Ap,1);  nw_s = size(Aws,1);  nu_s = size(Ats,1);
A  = [Ap                 zeros(np,nw_s)   zeros(np,nu_s);
      Bws*Cp1            Aws              zeros(nw_s,nu_s);
      zeros(nu_s,np)     zeros(nu_s,nw_s) Ats];
B1  = [Bp1; zeros(nw_s,1); zeros(nu_s,1)];

% ---- 双線形入力  B2(x) = [ Bp2(xp) ; o ; Bu ]（論文 Type 1 の形）----
%   Bp2(xp) = Bp21*x,  Bp21 = Bp2 * (ストローク速度 z1dot - z2dot を取り出す行)
sd   = [0 0 1 -1 zeros(1,nw_s+nu_s)];          % x からストローク速度を取り出す
Bp21 = cu*Bp2*sd;                               % 4×16（Simulink の G_Bp21）。u_n 基準なので cu 倍
Bu   = Bts*ones(3,1);                           % u は3つの入力重みすべてに入る（Simulink の Bu）

C11 = kW*[Dws*Cp1  Cws  zeros(size(Cws,1),nu_s)];
C12 = [zeros(size(Cts,1),np)  zeros(size(Cts,1),nw_s)  Cts];
D12 = [1; 0; 0];
% C2  = [Cp2  zeros(2,nw_s)  zeros(2,nu_s)];

C2  = [Cp2  zeros(size(Cp2,1),nw_s)  zeros(size(Cp2,1),nu_s)];   % 2 → size(Cp2,1)


D21 = Dp21;
n   = size(A,1);  nw = size(B1,2);
fprintf('max Re eig(A) = %.3g\n', max(real(eig(A))));

%% ===== 補題5.3：オブザーバ（R = Q^-1）=====
ops = sdpsettings('solver','sedumi','verbose',0);
Mo  = C11'*C11 + C12'*C12 - C2'*C2;
R   = sdpvar(n,n,'symmetric');
F1  = [ [-R*A-A'*R-Mo, -R*B1; -B1'*R, gam1^2*eye(nw)-D21'*D21] >= 1e-7*eye(n+nw), ...
        R >= 1e-7*eye(n) ];
s1  = optimize(F1, [], ops);
fprintf('Observer: %s, min eig(R) = %.3g, cond(R) = %.3g\n', ...
        s1.info, min(eig(value(R))), cond(value(R)));
L   = -value(R)\C2';                 % L = -Q*C2'
eo  = eig(A + L*C2);
fprintf('オブザーバ極: max Re = %.3g,  max |λ| = %.3g\n', max(real(eo)), max(abs(eo)));


Rv   = value(R);
Mlmi = [-Rv*A-A'*Rv-Mo, -Rv*B1; -B1'*Rv, gam1^2*eye(nw)-D21'*D21];
lmin = min(eig((Mlmi+Mlmi')/2));
fprintf('オブザーバ LMI: min eig = %.3g（正なら OK）\n', lmin);
if lmin <= 0 || max(real(eo)) >= 0 || max(abs(eo)) > 1e5
    error('オブザーバ設計に失敗 → kv を下げる。ここで停止します。');
end



%% ===== 補題5.4：γ2 の下限と P =====
hn      = norm(ss(A, L, [C11; C12], 0), inf);
gam2min = sqrt(1 + hn^2);
gam2    = 1.05*gam2min;
fprintf('||Gm||inf = %.4g  ->  gam2 の下限 = %.4g,  gam2 = %.4g\n', hn, gam2min, gam2);

c = gam2^2/(gam2^2-1);
M = C11'*C11 + C12'*C12;
% 式(5.18)の等式版（Riccati）を icare で解く
[P, ~, ~, info] = icare(A, [], c*M + 1e-6*eye(n), [], [], [], (L*L')/gam2^2);
if isempty(P)
    warning('icare: 安定化解なし → LMI で解く');
    Pv = sdpvar(n,n,'symmetric');
    F2 = [ [-(Pv*A+A'*Pv+c*M), -Pv*L; -L'*Pv, gam2^2*eye(size(L,2))] >= 1e-7*eye(n+size(L,2)), ...
           Pv >= 1e-7*eye(n) ];
    s2 = optimize(F2, [], ops);
    fprintf('Controller (LMI): %s\n', s2.info);
    P  = value(Pv);
end
res = P*A + A'*P + (1/gam2^2)*P*(L*L')*P + c*M;
fprintf('P: min eig = %.3g,  norm = %.3g,  max eig(残差) = %.3g（負なら式(5.18)を満たす）\n', ...
        min(eig(P)), norm(P), max(eig((res+res')/2)));
fprintf('全体の L2ゲイン上界 gamma1*gamma2 = %.3g\n', gam1*gam2);

%% ===== 制御則 (5.41) 用の変数 =====
% D12 = [1;0;0] なので (5.20) の C12 は D12'*C12 = C12(1,:)
C12u  = (D12'*D12)\(D12'*C12);
xhat0 = zeros(n,1);

%% ===== Simulink モデルの整合（何度実行しても同じ状態になる）=====
Nd     = numel(dx);
Wsimui = [(0:Nd-1).'*dt, dx(:)];
mdl    = 'non_linear_H_inf_and_compare';
load_system(mdl);
blk = @(b) [mdl '/' b];

% (1) ブロックの変数名を m ファイルと揃える
set_param(blk('Zero_w'), 'Value', 'zeros(nw_s,1)');   % nw は外乱次元(=1)なので nw_s を使う
set_param(blk('m1_'),    'Value', 'mw1');             % m1 はばね下質量と名前が衝突するので mw1
set_param(blk('G_C12'),  'Gain',  'C12u');            % 3×16 ではなく 1×16

% (2) B2(x) の x を「外部入力」ではなくプラントの真の状態から取る
%     （Inport を消しても線が G_Bp21 に残るので、線ごと消してからつなぐ）
reconnect(mdl, 'semi active plantl', 1, 'G_Bp21', 1);

% (3) セミアクティブ制約：0 ≤ u ≤ cmax - cmin（正規化単位なので /cu）
%     Neg → Sat_u → Prod_B2u（観測器とプラントの両方に飽和後の u を入れる）
if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'Sat_u'))
    add_block('simulink/Discontinuities/Saturation', blk('Sat_u'));
end
set_param(blk('Sat_u'), 'LowerLimit', '0', 'UpperLimit', '(cmax-cmin)/cu', ...
          'ZeroCross', 'off');   % u≈0 付近のチャタリングでゼロクロッシング検出が止まるのを防ぐ
reconnect(mdl, 'Sat_u', 1, 'Prod_B2u', 2);
reconnect(mdl, 'Neg',   1, 'Sat_u',    1);

% (3') プラントを双線形に：  xdot = A x + B1 w + B2(x) u
%      入力 = [w ; B2(x)u]（B2(x)u は Prod_B2u の出力をそのまま使う）
reconnect(mdl, 'Prod_B2u', 1, 'Mux', 2);
set_param(blk('semi active plantl'), 'A', 'A', 'B', '[B1 eye(n)]', ...
          'C', 'eye(n)', 'D', 'zeros(n,1+n)', 'InitialCondition', '0');

% (4) 比較用パッシブ 3 本（名前どおりの減衰に）
set_param(blk('passive suspension c is 300'),  'A', 'Ap_c300');
set_param(blk('passive suspension c is 1500'), 'A', 'Ap_pass');
set_param(blk('passive suspension c is 8000'), 'A', 'Ap_c8000');

% (5) 記録用 To Workspace
addTW(mdl, 'TW_x',     'yx',     'semi active plantl/1');
addTW(mdl, 'TW_u',     'yu',     'Neg/1');        % 飽和前の指令（正規化）
addTW(mdl, 'TW_usat',  'yusat',  'Sat_u/1');      % 飽和後（実際に入る u、正規化）
addTW(mdl, 'TW_xhat',  'yxhat',  'Int_xhat/1');
addTW(mdl, 'TW_p300',  'yp300',  'passive suspension c is 300/1');
addTW(mdl, 'TW_p1500', 'yp1500', 'passive suspension c is 1500/1');
addTW(mdl, 'TW_p8000', 'yp8000', 'passive suspension c is 8000/1');

% (6) ソルバ（高ゲインオブザーバで硬くなりやすいので ode15s）
set_param(mdl, 'StartTime', '0', 'StopTime', num2str((Nd-1)*dt), ...
          'Solver', 'ode15s', 'MaxStep', num2str(dt));
save_system(mdl);

%% ===== cu の自動調整（物理ゲインは cu^2 にほぼ比例）=====
for it = 1:3
    Bp21 = cu*Bp2*sd;
    out  = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
    [~, Un] = sig(out, 'yu');
    uc = cu*Un;                                   % 物理単位の指令 [Ns/m]
    cm = median(uc(uc > 0));
    fprintf('it %d: cu = %.3g, 正の c 指令の中央値 = %.3g\n', it, cu, cm);
    cu = cu*sqrt(c_target/cm);
end
Bp21 = cu*Bp2*sd;
fprintf('最終 cu = %.3g\n', cu);

%% ===== シミュレーション（最終 cu）=====
out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');

[tx, X ] = sig(out, 'yx');
[tu, U ] = sig(out, 'yusat');  U = cu*U;          % 実際に入った減衰係数 [Ns/m]
[~,  Uc] = sig(out, 'yu');     Uc = cu*Uc;        % 飽和前の指令 [Ns/m]
[th, Xh] = sig(out, 'yxhat');
[t3, P3] = sig(out, 'yp300');
[t1, P1] = sig(out, 'yp1500');
[t8, P8] = sig(out, 'yp8000');

t0 = max([tx(1) tu(1) t3(1) t1(1) t8(1)]);
t1e = min([tx(end) tu(end) t3(end) t1(end) t8(end)]);
tt = (t0:dt:t1e).';
Xu = interp1(tx, X(:,1:4), tt);
Uu = interp1(tu, U(:),     tt);

% 車体加速度  z2ddot = Ap(4,:)xp + Bp2(4) * u * (z1dot - z2dot)
aH = (Ap(4,:)*Xu.').' + Bp2(4)*Uu.*(Xu(:,3) - Xu(:,4));
a3 = (Ap_c300(4,:)  * interp1(t3, P3, tt).').';
a1 = (Ap_pass(4,:)  * interp1(t1, P1, tt).').';
a8 = (Ap_c8000(4,:) * interp1(t8, P8, tt).').';
Acc = [a3 a1 a8 aH];
Acc = Acc(tt >= tt(1) + 3, :);
fprintf('RMS 車体加速度: c=300 %.4f / c=1500 %.4f / c=8000 %.4f / 非線形H∞ %.4f [m/s^2]\n', rms(Acc));

% 診断
fprintf('指令: min %.3g / max %.3g / 負の割合 %.1f%% / 上限飽和 %.1f%%,  実際の追加減衰の平均 %.3g [Ns/m]\n', ...
        min(Uc), max(Uc), 100*mean(Uc<0), 100*mean(Uc>cmax-cmin), mean(U));
% スカイフック（ż2·(ż1−ż2) < 0 で ON）との一致率
Uci = interp1(tu, Uc(:), tx);
sky = X(:,4).*(X(:,3) - X(:,4)) < 0;
fprintf('H∞ ON 率 %.1f%%,  スカイフック ON 率 %.1f%%,  一致率 %.1f%%\n', ...
        100*mean(Uci>0), 100*mean(sky), 100*mean((Uci>0) == sky));
Xi = interp1(tx, X, th);
fprintf('推定誤差 max|x - x̂| = %.3g,  max|x| = %.3g\n', max(abs(Xi(:)-Xh(:))), max(abs(X(:))));

%% ===== PSD =====
fs = 1/dt;  nfft = 8192;
[Pxx, fpsd] = pwelch(Acc, hann(nfft), nfft/2, nfft, fs);
figure
loglog(fpsd, Pxx, 'LineWidth', 1.2); grid on; xlim([0.5 50])
xlabel('周波数 [Hz]'); ylabel('車体加速度 PSD [(m/s^2)^2/Hz]')
legend('passive c=300','passive c=1500','passive c=8000','非線形 H_\infty')
xline(1.16,'--','ばね上共振'); xline(9.4,'--','ばね下共振')

figure; plot(tt, Uu); grid on; xlabel('t [s]'); ylabel('実際の追加減衰係数 [Ns/m]')

%% ===== 時系列（figure 4 つ）=====
k   = tt >= tt(1) + 3;                 % RMS と同じ区間
tk  = tt(k);
X3  = interp1(t3, P3, tt);  X1 = interp1(t1, P1, tt);  X8 = interp1(t8, P8, tt);

Acc_t    = [a3(k)    a1(k)    a8(k)    aH(k)];
Stroke_t = [X3(k,2)  X1(k,2)  X8(k,2)  Xu(k,2)];
Tire_t   = [X3(k,1)  X1(k,1)  X8(k,1)  Xu(k,1)];
lg = {'passive c=300','passive c=1500','passive c=8000','非線形 H_\infty'};

% (1) 車体加速度
figure('Name','車体加速度')
plot(tk, Acc_t); grid on
xlabel('t [s]'); ylabel('車体加速度 [m/s^2]'); legend(lg)
title(sprintf('RMS: %.3f / %.3f / %.3f / %.3f [m/s^2]', rms(Acc_t)))

% (2) サスペンションストローク
figure('Name','ストローク')
plot(tk, Stroke_t*1e3); grid on
xlabel('t [s]'); ylabel('ストローク z_2-z_1 [mm]'); legend(lg)
title(sprintf('RMS: %.2f / %.2f / %.2f / %.2f [mm]', 1e3*rms(Stroke_t)))

% (3) タイヤたわみ
figure('Name','タイヤたわみ')
plot(tk, Tire_t*1e3); grid on
xlabel('t [s]'); ylabel('タイヤたわみ z_1-z_0 [mm]'); legend(lg)
title(sprintf('RMS: %.2f / %.2f / %.2f / %.2f [mm]', 1e3*rms(Tire_t)))

% (4) H∞ の減衰係数
figure('Name','減衰係数')
plot(tk, cmin + Uu(k), 'k'); grid on; hold on
yline([cmin cmax], '--r')
xlabel('t [s]'); ylabel('減衰係数 c_{min}+u [Ns/m]')
title('非線形 H_\infty の減衰係数（赤破線：下限 300 / 上限 8000）')


%% ===== スカイフック（on-off：ż2(ż2−ż1) > 0 で c = cmax、それ以外 c = cmin）=====
wS = Wsimui(:,2);  tS = Wsimui(:,1);  NS = numel(wS);
fS = @(x,w,u) Ap*x + Bp1*w + Bp2*u*(x(3)-x(4));
xs = zeros(4,1);  XS = zeros(NS,4);  US = zeros(NS,1);
for i = 1:NS
    XS(i,:) = xs.';
    uS = (cmax-cmin) * (xs(4)*(xs(4)-xs(3)) > 0);   % 真の状態を使う理想スカイフック
    US(i) = uS;  w = wS(i);
    r1 = fS(xs,w,uS);  r2 = fS(xs+dt/2*r1,w,uS);
    r3 = fS(xs+dt/2*r2,w,uS);  r4 = fS(xs+dt*r3,w,uS);
    xs = xs + dt/6*(r1 + 2*r2 + 2*r3 + r4);
end
XSu = interp1(tS, XS, tt);  USu = interp1(tS, US, tt, 'previous');
aS  = (Ap(4,:)*XSu.').' + Bp2(4)*USu.*(XSu(:,3)-XSu(:,4));

Acc5 = [Acc_t    aS(k)];
Str5 = [Stroke_t XSu(k,2)];
Tir5 = [Tire_t   XSu(k,1)];
lg5  = [lg {'skyhook (on-off)'}];
fprintf('RMS 車体加速度 [m/s^2]: %.4f / %.4f / %.4f / %.4f / %.4f\n', rms(Acc5));
fprintf('RMS ストローク [mm]   : %.2f / %.2f / %.2f / %.2f / %.2f\n', 1e3*rms(Str5));
fprintf('RMS タイヤたわみ [mm] : %.2f / %.2f / %.2f / %.2f / %.2f\n', 1e3*rms(Tir5));

% PSD
[Pxx5, fp5] = pwelch(Acc5, hann(nfft), nfft/2, nfft, fs);
figure('Name','PSD（スカイフック込み）')
loglog(fp5, Pxx5, 'LineWidth', 1.2); grid on; xlim([0.5 50])
xlabel('周波数 [Hz]'); ylabel('車体加速度 PSD [(m/s^2)^2/Hz]'); legend(lg5)
xline(1.16,'--','ばね上共振'); xline(9.4,'--','ばね下共振')

% 時系列
figure('Name','車体加速度（スカイフック込み）')
plot(tk, Acc5); grid on; xlabel('t [s]'); ylabel('車体加速度 [m/s^2]'); legend(lg5)

figure('Name','ストローク（スカイフック込み）')
plot(tk, Str5*1e3); grid on; xlabel('t [s]'); ylabel('ストローク [mm]'); legend(lg5)

figure('Name','タイヤたわみ（スカイフック込み）')
plot(tk, Tir5*1e3); grid on; xlabel('t [s]'); ylabel('タイヤたわみ [mm]'); legend(lg5)

figure('Name','減衰係数：H∞ vs スカイフック')
plot(tk, cmin + Uu(k), 'b', tk, cmin + USu(k), 'r'); grid on
xlabel('t [s]'); ylabel('減衰係数 [Ns/m]'); legend('非線形 H_\infty','skyhook')

figure('Name','減衰係数：H∞ vs スカイフック')
tw = [5 6];                                      % 表示する 1 秒間
ax1 = subplot(2,1,1); stairs(tk, cmin + Uu(k), 'b'); grid on
ylabel('H_\infty [Ns/m]'); ylim([0 8500])
ax2 = subplot(2,1,2); stairs(tk, cmin + USu(k), 'r'); grid on
ylabel('skyhook [Ns/m]'); xlabel('t [s]'); ylim([0 8500])
linkaxes([ax1 ax2],'x'); xlim(tw)

%% ==== ローカル関数（必ずファイルの一番最後）====
function reconnect(mdl, srcBlk, srcPort, dstBlk, dstPort)
% dstBlk の入力端子 dstPort を srcBlk の出力端子 srcPort につなぎ直す。
% 既存の線があれば削除し、その元が Inport ブロックならブロックも削除する。
phD = get_param([mdl '/' dstBlk], 'PortHandles');
phS = get_param([mdl '/' srcBlk], 'PortHandles');
pin  = phD.Inport(dstPort);
pout = phS.Outport(srcPort);
ln = get_param(pin, 'Line');
if ln ~= -1
    if get_param(ln, 'SrcPortHandle') == pout
        return                                   % すでに正しく接続済み
    end
    sb = get_param(ln, 'SrcBlockHandle');
    delete_line(ln);
    if sb ~= -1 && strcmp(get_param(sb, 'BlockType'), 'Inport')
        delete_block(sb);
    end
end
add_line(mdl, pout, pin, 'autorouting', 'on');
end

function addTW(mdl, name, var, src)
% To Workspace を（なければ）追加して src に接続
path = [mdl '/' name];
if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', name))
    add_block('simulink/Sinks/To Workspace', path, ...
              'VariableName', var, 'SaveFormat', 'Timeseries');
end
try, add_line(mdl, src, [name '/1'], 'autorouting', 'on'); end
end

function [t, v] = sig(out, name)
s = out.get(name);
if isa(s,'timeseries')
    t = s.Time;  v = squeeze(s.Data);
elseif isstruct(s)
    v = s.signals.values;
    if isfield(s,'time') && ~isempty(s.time), t = s.time; else, t = out.tout; end
else
    t = out.tout;  v = s;
end
v = squeeze(v);
if size(v,1) ~= numel(t), v = v.'; end
end