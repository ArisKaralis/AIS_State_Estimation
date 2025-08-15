function crlb = calculateCRLB(data, model_type, q_params, R_params)
% CALCULATECRLB  Posterior CRLB for CV/CA/CTRV with per-step Q(dt)
% and robust numerical solves. Works in METERS (no lat/lon).
%
% Now reports VELOCITY bound as SPEED (SOG) CRLB:
%   - CV/CA: delta-method on [vx, vy]
%   - CTRV : direct on state v
%
% Inputs/Outputs: (same as the previous version)

% ---------- basic checks ----------
assert(isfield(data,'x') && isfield(data,'y') && isfield(data,'time'), ...
    'data must have x, y, time (column vectors).');

t = data.time(:);
if isduration(t), t = seconds(t); end
if isdatetime(t), t = seconds(t - t(1)); end
N = numel(t);  assert(N >= 2, 'Need at least 2 time points.');

x = data.x(:); y = data.y(:);
assert(numel(x)==N && numel(y)==N, 'x,y must match time length.');

meas_model   = R_params.meas_model;   % 'pos' or 'pos_sog_cog'
use_nonlinear = strcmp(model_type,'CTRV') || strcmp(meas_model,'pos_sog_cog');

% ---------- nominal states (for F,H evaluation) ----------
states = build_nominal_states(model_type, x, y, t, data);

% ---------- initial covariance (prior) ----------
P0 = initial_covariance(model_type);
nx = size(P0,1);

% ---------- outputs ----------
pos_rmse_lb  = zeros(N,1);
vel_rmse_lb  = zeros(N,1);    % SPEED bound
ellipse_axes = zeros(2,N);
P_seq        = zeros(nx, nx, N);

% first step (prior)
P_seq(:,:,1) = P0;
[pos_rmse_lb(1), vel_rmse_lb(1), ellipse_axes(:,1)] = summarize_bounds(P0, model_type, states(:,1));

% ---------- main loop ----------
if ~use_nonlinear
    % Linear + position-only -> KF covariance == CRLB
    Pk = P0;
    for k = 2:N
        dt   = max(t(k) - t(k-1), eps);
        xkm1 = states(:,k-1);

        Fk = F_matrix(model_type, dt, xkm1);
        Qk = Q_matrix(model_type, q_params, dt, xkm1);
        Hk = H_matrix(model_type, meas_model, xkm1);
        Rk = R_matrix(meas_model, R_params);

        % Predict
        Ppred = Fk * Pk * Fk.' + Qk;

        % Update (Joseph form)
        S  = Hk * Ppred * Hk.' + Rk;
        K  = (Ppred * Hk.') / S;
        I  = eye(nx);
        Pk = (I - K*Hk) * Ppred * (I - K*Hk).' + K*Rk*K.';
        Pk = symmetrize_spd(Pk);

        P_seq(:,:,k) = Pk;
        [pos_rmse_lb(k), vel_rmse_lb(k), ellipse_axes(:,k)] = summarize_bounds(Pk, model_type, states(:,k));
    end
else
    % Nonlinear (CTRV or SOG/COG) -> PCRLB (Tichavský)
    Jk = safe_inv(P0);
    for k = 2:N
        dt   = max(t(k) - t(k-1), eps);
        xkm1 = states(:,k-1);

        Fk = F_matrix(model_type, dt, xkm1);
        Hk = H_matrix(model_type, meas_model, xkm1);
        Qk = Q_matrix(model_type, q_params, dt, xkm1);
        Rk = R_matrix(meas_model, R_params);

        [QiF, QiI] = robust_Q_inv_mult(Qk, Fk);
        D11 = Fk.' * QiF;
        D12 = -Fk.' * QiI;
        D21 = D12.';

        RinvH = robust_R_inv_mult(Rk, Hk);
        D22 = QiI + Hk.' * RinvH;

        A = Jk + D11;
        A = regularize_spd(A, 1e-12);
        X = robust_solve(A, D12);

        Jk = D22 - D21 * X;
        Jk = symmetrize_spd(Jk);

        Pk = safe_inv(Jk);
        P_seq(:,:,k) = Pk;
        [pos_rmse_lb(k), vel_rmse_lb(k), ellipse_axes(:,k)] = summarize_bounds(Pk, model_type, states(:,k));
    end
end

% ---------- package ----------
crlb = struct();
crlb.model_type           = model_type;
crlb.time                 = t;
crlb.position_crlb        = pos_rmse_lb;
crlb.velocity_crlb        = vel_rmse_lb;     % **SOG bound**
crlb.ellipse_axes         = ellipse_axes;    % [major; minor] per column
crlb.P_seq                = P_seq;
crlb.states_nominal       = states;
crlb.final_position_crlb  = pos_rmse_lb(end);
crlb.final_velocity_crlb  = vel_rmse_lb(end);
end

% =================== helpers ===================

function states = build_nominal_states(model, x, y, t, data)
dt = diff(t);
vx = [0; diff(x)./max(dt,eps)];
vy = [0; diff(y)./max(dt,eps)];
switch model
case 'CV'
    states = [x.'; y.'; vx.'; vy.'];                      % [x;y;vx;vy]
case 'CA'
    ax = [0; diff(vx)./max(dt,eps)];
    ay = [0; diff(vy)./max(dt,eps)];
    states = [x.'; y.'; vx.'; vy.'; ax.'; ay.'];          % [x;y;vx;vy;ax;ay]
case 'CTRV'
    v   = hypot(vx,vy);
    psi = unwrap(atan2(vy, vx));
    omg = [0; diff(psi)./max(dt,eps)];
    if isfield(data,'sog'), v   = data.sog(:); end
    if isfield(data,'cog'), psi = unwrap(deg2rad(data.cog(:))); omg = [0; diff(psi)./max(dt,eps)]; end
    states = [x.'; y.'; v.'; psi.'; omg.'];               % [x;y;v;psi;omega]
otherwise
    error('Unknown model %s', model);
end
end

function P0 = initial_covariance(model)
switch model
case 'CV',   P0 = diag([100, 100, 4, 4]);
case 'CA',   P0 = diag([100, 100, 4, 4, 1, 1]);
case 'CTRV', P0 = diag([100, 100, 4, 0.1, 0.01]);
otherwise,   error('Unknown model %s', model);
end
end

function F = F_matrix(model, dt, xk)
switch model
case 'CV'
    F = [1 0 dt 0;
         0 1 0  dt;
         0 0 1  0;
         0 0 0  1];
case 'CA'
    F = [1 0 dt 0  0.5*dt^2 0;
         0 1 0  dt 0  0.5*dt^2;
         0 0 1  0  dt 0;
         0 0 0  1  0  dt;
         0 0 0  0  1  0;
         0 0 0  0  0  1];
case 'CTRV'
    v = xk(3); psi = xk(4); om = xk(5);
    if abs(om) < 1e-8
        F = [1 0  dt*cos(psi)  -v*dt*sin(psi)   0;
             0 1  dt*sin(psi)   v*dt*cos(psi)   0;
             0 0  1             0               0;
             0 0  0             1               dt;
             0 0  0             0               1];
    else
        s1 = sin(psi + om*dt); c1 = cos(psi + om*dt);
        s0 = sin(psi);         c0 = cos(psi);
        sin_term = s1 - s0;    cos_term = c1 - c0;
        dpx_dv   = sin_term/om;
        dpy_dv   = -cos_term/om;
        dpx_dpsi =  v*(c1 - c0)/om;
        dpy_dpsi =  v*(s1 - s0)/om;
        dpx_dom  = v*( dt*c1*om - sin_term )/(om^2);
        dpy_dom  = v*( dt*s1*om + cos_term )/(om^2);
        F = [1 0 dpx_dv  dpx_dpsi  dpx_dom;
             0 1 dpy_dv  dpy_dpsi  dpy_dom;
             0 0 1       0         0;
             0 0 0       1         dt;
             0 0 0       0         1];
    end
otherwise
    error('Unknown model %s', model);
end
end

function H = H_matrix(model, meas_model, xk)
% Robust measurement Jacobian builder.
% model      : 'CV' | 'CA' | 'CTRV'  (case-insensitive)
% meas_model : 'pos' | 'pos_sog' | 'pos_sog_cog' (case-insensitive)

mdl  = upper(strtrim(model));
meas = lower(strtrim(meas_model));

switch meas

case 'pos'
    switch mdl
    case 'CV'
        H = [1 0 0 0;
             0 1 0 0];
    case 'CA'
        H = [1 0 0 0 0 0;
             0 1 0 0 0 0];
    case 'CTRV'
        H = [1 0 0 0 0;
             0 1 0 0 0];
    otherwise
        error('H_matrix:UnknownModelForPOS','Unknown model "%s" for meas_model "pos".', model);
    end

case 'pos_sog'
    % position + speed (no course)
    switch mdl
    case 'CV'
        vx = xk(3); vy = xk(4);
        [dS_dvx,dS_dvy] = speed_partials(vx,vy);
        H = [1 0 0 0;
             0 1 0 0;
             0 0 dS_dvx dS_dvy];
    case 'CA'
        vx = xk(3); vy = xk(4);
        [dS_dvx,dS_dvy] = speed_partials(vx,vy);
        H = [1 0 0 0 0 0;
             0 1 0 0 0 0;
             0 0 dS_dvx dS_dvy 0 0];
    case 'CTRV'
        % [x;y;v;psi;omega] -> pos + SOG=v
        H = [1 0 0 0 0;
             0 1 0 0 0;
             0 0 1 0 0];
    otherwise
        error('H_matrix:UnknownModelForPOSSOG','Unknown model "%s" for meas_model "pos_sog".', model);
    end

case 'pos_sog_cog'
    % position + speed + course
    switch mdl
    case 'CV'
        vx = xk(3); vy = xk(4);
        [dS_dvx,dS_dvy,dpsi_dvx,dpsi_dvy] = speed_course_partials(vx,vy);
        H = [1 0 0 0;
             0 1 0 0;
             0 0 dS_dvx   dS_dvy;
             0 0 dpsi_dvx dpsi_dvy];
    case 'CA'
        vx = xk(3); vy = xk(4);
        [dS_dvx,dS_dvy,dpsi_dvx,dpsi_dvy] = speed_course_partials(vx,vy);
        H = [1 0 0 0 0 0;
             0 1 0 0 0 0;
             0 0 dS_dvx   dS_dvy   0 0;
             0 0 dpsi_dvx dpsi_dvy 0 0];
    case 'CTRV'
        % [x;y;v;psi;omega] -> pos, SOG=v, COG≈psi
        H = [1 0 0 0 0;
             0 1 0 0 0;
             0 0 1 0 0;
             0 0 0 1 0];
    otherwise
        error('H_matrix:UnknownModelForPOSSOGCOG','Unknown model "%s" for meas_model "pos_sog_cog".', model);
    end

otherwise
    error('H_matrix:UnknownMeasModel','Unknown meas_model "%s". Use pos | pos_sog | pos_sog_cog.', meas_model);
end
end

function [dS_dvx,dS_dvy,dpsi_dvx,dpsi_dvy] = speed_course_partials(vx,vy)
% Safe derivatives for speed s = sqrt(vx^2+vy^2) and psi = atan2(vy,vx)
v2 = vx*vx + vy*vy;
v  = sqrt(max(v2, 1e-12));
dS_dvx = vx / v;
dS_dvy = vy / v;
den = max(v2, 1e-12);   % avoid division by ~0
dpsi_dvx = -vy / den;
dpsi_dvy =  vx / den;
end

function [dS_dvx,dS_dvy] = speed_partials(vx,vy)
v2 = vx*vx + vy*vy;
v  = sqrt(max(v2, 1e-12));
dS_dvx = vx / v;
dS_dvy = vy / v;
end



function R = R_matrix(meas_model, Rpar)
switch meas_model
case 'pos',         R = diag([Rpar.pos^2, Rpar.pos^2]);
case 'pos_sog_cog', R = diag([Rpar.pos^2, Rpar.pos^2, Rpar.vel^2, Rpar.cog^2]);
otherwise,          error('Unknown measurement model %s', meas_model);
end
end

function Q = Q_matrix(model, q, dt, xk)
switch model
case 'CV'
    qacc = q.q_acc;
    Q1 = [dt^3/3, dt^2/2; dt^2/2, dt];
    Q  = qacc * blkdiag(Q1, Q1);
case 'CA'
    qj = q.q_jerk;
    Qb = [dt^5/20, dt^4/8, dt^3/6;
          dt^4/8,  dt^3/3, dt^2/2;
          dt^3/6,  dt^2/2, dt];
    Q  = qj * blkdiag(Qb, Qb);
case 'CTRV'
    qa  = q.q_a;  qnu = q.q_nu;
    v   = xk(3); psi = xk(4); om = xk(5);
    if abs(om) < 1e-8
        dpx_dv   = dt*cos(psi);      dpy_dv   = dt*sin(psi);
        dpx_dpsi = -v*dt*sin(psi);   dpy_dpsi =  v*dt*cos(psi);
        dpx_dom  = 0;                dpy_dom  = 0;
    else
        s1 = sin(psi + om*dt); c1 = cos(psi + om*dt);
        s0 = sin(psi);         c0 = cos(psi);
        sin_term = s1 - s0;    cos_term = c1 - c0;
        dpx_dv   = sin_term/om;
        dpy_dv   = -cos_term/om;
        dpx_dpsi =  v*(c1 - c0)/om;
        dpy_dpsi =  v*(s1 - s0)/om;
        dpx_dom  = v*( dt*c1*om - sin_term )/(om^2);
        dpy_dom  = v*( dt*s1*om + cos_term )/(om^2);
    end
    Lx_a = dpx_dv * dt;
    Ly_a = dpy_dv * dt;
    Lx_n = dpx_dpsi * (0.5*dt^2) + dpx_dom * dt;
    Ly_n = dpy_dpsi * (0.5*dt^2) + dpy_dom * dt;
    L = [ Lx_a,  Lx_n;
          Ly_a,  Ly_n;
          dt,    0;
          0,     0.5*dt^2;
          0,     dt ];
    Qw = diag([qa, qnu]);
    Q  = L * Qw * L.';
otherwise
    error('Unknown model %s', model);
end

% ridge + symmetrize
if ~isfield(q,'epsQ'), epsQ = 1e-12; else, epsQ = q.epsQ; end
sQ = trace(Q)/size(Q,1);
Q  = Q + max(epsQ, 1e-9*max(sQ,1)) * eye(size(Q));
Q  = symmetrize_spd(Q);
end

function [pos_lb, speed_lb, axes] = summarize_bounds(P, model, xk)
P = symmetrize_spd(P);
Pxy = P(1:2,1:2);
evals = eig((Pxy+Pxy.')/2);
evals = sort(max(evals,0),'descend');
axes  = sqrt(evals);               % major/minor ellipse axes
pos_lb = sqrt(trace(Pxy));

switch model
case 'CV'
    % speed bound via delta method on [vx,vy]
    vx = xk(3); vy = xk(4);
    v  = hypot(vx,vy);
    Pvv = P(3:4,3:4);
    if v < 1e-9
        speed_lb = sqrt(max(trace(Pvv),0)/2);   % fallback if nearly zero speed
    else
        g = [vx; vy] / max(v,1e-9);
        speed_lb = sqrt(max(g.' * Pvv * g, 0));
    end
case 'CA'
    vx = xk(3); vy = xk(4);
    v  = hypot(vx,vy);
    Pvv = P(3:4,3:4);
    if v < 1e-9
        speed_lb = sqrt(max(trace(Pvv),0)/2);
    else
        g = [vx; vy] / max(v,1e-9);
        speed_lb = sqrt(max(g.' * Pvv * g, 0));
    end
case 'CTRV'
    speed_lb = sqrt(max(P(3,3),0));    % state v
otherwise
    speed_lb = NaN;
end
end

% ---------- robust linear algebra ----------
function X = robust_solve(A, B)
X = [];
[La,pa] = chol((A+A')/2,'lower');
if pa==0, X = La'\(La\B);
else
    [~,U] = lu(A);
    if rcond(U) > 1e-12, X = A \ B; else, X = pinv(A) * B; end
end
end

function [QiF, QiI] = robust_Q_inv_mult(Q, F)
Qsym = (Q+Q')/2;
[Lq,pq] = chol(Qsym,'lower');
n = size(Q,1);
if pq==0
    QiF = Lq'\(Lq\F);
    QiI = Lq'\(Lq\eye(n));
else
    Qi  = pinv(Qsym);
    QiF = Qi * F;
    QiI = Qi;
end
end

function RinvH = robust_R_inv_mult(R, H)
Rsym = (R+R')/2;
[Lr,pr] = chol(Rsym,'lower');
if pr==0, RinvH = Lr'\(Lr\H);
else,     RinvH = pinv(Rsym) * H;
end
end

function A = regularize_spd(A, eps0)
A = (A+A')/2;
sA = trace(A)/size(A,1);
A = A + max(eps0, 1e-12*max(sA,1)) * eye(size(A));
end

function M = symmetrize_spd(M)
M = (M+M')/2;
[eV,eD] = eig(M);
d = max(diag(eD),0);
M = eV*diag(d)*eV';
M = (M+M')/2;
end

function Ainv = safe_inv(A)
A = (A+A')/2;
[La,pa] = chol(A,'lower');
if pa==0, Ainv = La'\(La\eye(size(A)));
else,     Ainv = pinv(A);
end
end

