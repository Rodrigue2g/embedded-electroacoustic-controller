import numpy as np
import scipy.signal as signal
import matplotlib.pyplot as plt
from py.c2d import compute_filter_coeffs
from py.physics import get_physics_params

def handle_plot(self, values, mode_name):
    self.log.append(f"Generating validation plot for {mode_name}...")
    
    p = get_physics_params(values)
    fs = 1.0 / p.ts_ctr
    
    # Comtinuous sys H(s)
    # H(s) = num_c / den_c
    # num_c = np.array([p.a2, p.a1, p.a0]) * p.Sd / p.Bl
    # den_c = np.array([p.b2, p.b1, p.b0])
    num_c = np.array([p.a0, p.a1, p.a2]) * p.Sd / p.Bl
    den_c = np.array([p.b0, p.b1, p.b2])
    Hs_c = (num_c, den_c)
    
    # Signal
    dur = 5.0
    t = np.arange(0, dur, 1/fs)
    x = signal.chirp(t, f0=10, f1=1600, t1=dur, method='linear') 
    try:
        _, y_cont, _ = signal.lsim(Hs_c, U=x, T=t)
    except Exception as e:
        self.log.append(f"lsim error: {e}")
        y_cont = np.zeros_like(x)

    # Discrete tf H(z)
    bz, az = compute_filter_coeffs(p)
    # Filter signal
    y_disc = signal.lfilter(bz, az, x)

    # Use hanning window to reduce spectral leakage
    window = np.hanning(len(x))
    x_win = x * window
    y_cont_win = y_cont * window
    y_disc_win = y_disc * window

    N = len(x)
    Nfft = 2**int(np.ceil(np.log2(N))) 
    
    f_axis = np.fft.fftfreq(Nfft, d=1/fs)
    # Filter valid frequencies (10Hz to Nyquist)
    mask = (f_axis > 10) & (f_axis < fs/2)
    f_plot = f_axis[mask]

    X_fft = np.fft.fft(x_win, n=Nfft)
    Y_cont_fft = np.fft.fft(y_cont_win, n=Nfft)
    Y_disc_fft = np.fft.fft(y_disc_win, n=Nfft)

    # Add small epsilon to avoid division by zero
    H_cont_sim = Y_cont_fft[mask] / (X_fft[mask] + 1e-12)
    H_disc_sim = Y_disc_fft[mask] / (X_fft[mask] + 1e-12)

    # H_cont_sim = Y_cont_fft[mask] * np.linalg.pinv([X_fft[mask]])
    # H_disc_sim = Y_disc_fft[mask] * np.linalg.pinv([X_fft[mask]])

    # Theoretical ref
    w = 2 * np.pi * f_plot
    s = 1j * w
    resp_theory = np.polyval(num_c, s) / np.polyval(den_c, s)

    # Plots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    fig.suptitle(f'Validation: {mode_name}', fontsize=16)
    # Magnitude
    ax1.semilogx(f_plot, 20 * np.log10(np.abs(resp_theory)), 'k', linewidth=2.5, alpha=0.5, label='Theory H(s)')
    ax1.semilogx(f_plot, 20 * np.log10(np.abs(H_cont_sim)), 'g--', linewidth=1.5, label='Sim Cont')
    ax1.semilogx(f_plot, 20 * np.log10(np.abs(H_disc_sim)), 'r--', linewidth=1.5, label='Sim Disc')
    ax1.set_title("Magnitude Response")
    ax1.set_ylabel("Magnitude (dB)")
    ax1.grid(True, which="both", alpha=0.5)
    ax1.legend()
    ax1.set_xlim([10, fs/2])
    # Phase
    ax2.semilogx(f_plot, np.angle(resp_theory, deg=True), 'k', linewidth=2.5, alpha=0.5, label='Theory')
    ax2.semilogx(f_plot, np.angle(H_cont_sim, deg=True), 'g--', linewidth=1.5, label='Sim Cont')
    ax2.semilogx(f_plot, np.angle(H_disc_sim, deg=True), 'r--', linewidth=1.5, label='Sim Disc')
    ax2.set_title("Phase Response")
    ax2.set_ylabel("Phase (deg)")
    ax2.set_xlabel("Frequency (Hz)")
    ax2.grid(True, which="both", alpha=0.5)
    ax2.set_xlim([10, fs/2])

    plt.tight_layout()
    plt.show()