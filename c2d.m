function [Phi_c, Phi_d, numd, dend] = discretize_params(params)
% Build continuous and discrete TF from the params structure.
    % Build continuous TF
    b2 = params.b2;
    b1 = params.b1;
    b0 = params.b0;

    a2 = params.a2;
    a1 = params.a1;
    a0 = params.a0;

    num_c = params.Sd/params.Bl * [params.a2, params.a1, params.a0];
    den_c = [params.b2, params.b1, params.b0];
    
    % Θ(s)
    Phi_c = tf(num_c, den_c);
    Phi_d = c2d(Phi_c, params.ts_ctr, 'tustin');

    [bz, az] = tfdata(Phi_d, 'v');

    fprintf('/* === DISCRETIZED THETA(z) COEFFICIENTS === */\n');
    fprintf('params.bz[0] = %.12e;\n', bz(1));
    fprintf('params.bz[1] = %.12e;\n', bz(2));
    fprintf('params.bz[2] = %.12e;\n', bz(3));
    fprintf('\n');
    fprintf('params.az[0] = %.12e;   // always 1 for biquad\n', az(1));
    fprintf('params.az[1] = %.12e;\n', az(2));
    fprintf('params.az[2] = %.12e;\n', az(3));
    
    % 5) Print CMSIS DF2T-ready coefficients
    fprintf('\n/* === CMSIS biquad_coeffs[] === */\n');
    fprintf('biquad_coeffs[0] = %.12e;  // b0\n', bz(1));
    fprintf('biquad_coeffs[1] = %.12e;  // b1\n', bz(2));
    fprintf('biquad_coeffs[2] = %.12e;  // b2\n', bz(3));
    fprintf('biquad_coeffs[3] = %.12e;  // -a1\n', -az(2));
    fprintf('biquad_coeffs[4] = %.12e;  // -a2\n', -az(3));

    numd = bz;
    dend = az;
end


function params = init_params_matlab()

    % ===== PHYSICS =====
    params.c0   = 347.13;
    params.rho0 = 1.1839;

    % ===== CONTROL SENSITIVITY =====
    params.sens_p = -1.0 / 37.1e-3;
    params.i2u    = 100.0;

    % ===== SPEAKER PARAMETERS =====
    params.Sd  = 23.5e-4;

    params.Bl = 1.806225000000000;

    params.Rms = 0.742623500000000;
    params.Mms = 0.001329715000000;
    params.Cmc = 1.339291000000000e-04;

    params.f0  = 441.5461;  % not used for the TF directly

    % ===== TARGET parameters =====
    f_tgt = 220;

    params.muM = 0.5;  % given
    params.muR = (1/10) * params.rho0 * params.c0 * params.Sd / params.Rms;
    params.muC = (2*pi*f_tgt)^2 * params.Mms * params.Cmc;

    % ===== Continuous TF coefficients =====
    params.b2 = params.muM * params.Mms * params.Cmc;
    params.b1 = params.muR * params.Rms * params.Cmc;
    params.b0 = params.muC;

    params.a2 = (params.muM - 1.0) * params.Mms * params.Cmc;
    params.a1 = (params.muR - 1.0) * params.Rms * params.Cmc;
    params.a0 = (params.muC - 1.0);

    % ===== Sampling =====
    params.ts_ctr = 40e-6;
    params.fs_ctr = 1 / params.ts_ctr;
end


% clear; clc;
% 
params = init_params_matlab();
 
[Phi_c, Phi_d, bz, az] = discretize_params(params);
% 
% disp('Continuous TF:');
% Phi_c
% 
% disp('Discrete TF:');
% Phi_d

