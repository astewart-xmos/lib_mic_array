# Copyright 2023 XMOS LIMITED.
# This Software is subject to the terms of the XMOS Public Licence: Version 1.

from pathlib import Path

import scipy.signal as spsig
import numpy as np
import matplotlib.pyplot as plt

try:
    # for pytest
    from . import filter_tools as ft
except ImportError:
    import filter_tools as ft


def plot_stage(coeff_decim, fs_0, fs_final, axs=None):

    coeff = coeff_decim[0]
    decim_rate = coeff_decim[1]
    fs_1 = fs_0 // decim_rate

    nfft = 1024
    coeff = ft.normalise_coeffs(coeff)
    w, response = spsig.freqz(coeff, fs=fs_0, worN=nfft*decim_rate)

    # calculate sum of all tha aliases
    ars = np.abs(response)
    response_decim = np.copy(ars[:nfft])
    for n in range(1, decim_rate):
        if n % 2 == 0:
            response_decim += ars[n*nfft:(n+1)*nfft]
        else:
            response_decim += np.flip(ars[n*nfft:(n+1)*nfft])

    w_decim = w[:nfft]
    alias_level = response_decim - np.abs(response[:nfft])

    if axs is None:
        fig, axs = plt.subplots(2, 2)
        fig.suptitle("stage response, decim %d" % decim_rate)

    # stage 1
    axs[0, 0].set_title("filter taps")
    axs[0, 0].plot(coeff)
    axs[0, 0].set_xlim([0, len(coeff)])
    axs[0, 0].grid(True)

    axs[1, 0].set_title("filter response")
    axs[1, 0].plot(w, ft.db(np.abs(response)))
    axs[1, 0].set_xlim([0, w[-1]])
    axs[1, 0].set_ylim([-155, 5])
    axs[1, 0].grid(True)
    # shade out the frequencies that will get filtered out by subsequent stages,
    # unless it's the last
    if fs_1 == fs_final:
        w_where = w > fs_1/2
    else:
        w2 = w[:, None]
        w_where = np.invert(((w2 > np.arange(0, fs_0 + fs_1, fs_1) - fs_final/2) &
                             (w2 < np.arange(0, fs_0 + fs_1, fs_1) + fs_final/2)).any(1))
    axs[1, 0].fill_between(w, -155, 5, where=w_where, color="grey", alpha=0.1, hatch='//')

    axs[0, 1].set_title("passband response")
    axs[0, 1].plot(w, ft.db(np.abs(response)), label="response")
    axs[0, 1].plot(w_decim, ft.db(np.abs(response_decim)), label="decimated response")
    axs[0, 1].plot(w_decim, ft.db(np.abs(alias_level)), label="alias level")
    axs[0, 1].set_xlim([0, fs_1/2*1.2])
    axs[0, 1].set_ylim([-200, 10])
    axs[0, 1].legend()
    axs[0, 1].grid(True)

    axs[1, 1].set_ylabel("passband ripple")
    axs[1, 1].plot(w, ft.db(np.abs(response)), label="response")
    axs[1, 1].plot(w_decim, ft.db(np.abs(response_decim)), label="decimated response")
    axs[1, 1].plot(w_decim, ft.db(np.abs(alias_level)), label="alias level")
    axs[1, 1].set_xlim([0, fs_1/2*1.2])
    axs[1, 1].set_ylim([-10, 10])
    axs[1, 1].legend()
    axs[1, 1].grid(True)

    return axs


def plot_filters(coeffs, fs_0, axs=None):

    n_stages = len(coeffs)
    if axs is None:
        fig, axs = plt.subplots(3, (n_stages + 1))

    nfft = 32768
    fs = fs_0
    response_combo = np.ones(nfft, dtype=np.complex64)

    # stage plots
    for stage in range(n_stages):
        stage_coeff = ft.normalise_coeffs(coeffs[stage][0])
        w_1, response_1 = spsig.freqz(stage_coeff, fs=fs, worN=nfft)

        # calculate combined response as we go
        response_combo[:nfft] = response_combo[:nfft]*response_1

        # decimate fs and nfft
        nfft = nfft // coeffs[stage][1]
        fs = fs // coeffs[stage][1]

        axs[0, stage].set_title("stage %d" % (stage+1))
        axs[0, stage].plot(stage_coeff)
        axs[0, stage].set_xlim([0, len(stage_coeff)])
        axs[0, stage].grid(True)

        axs[1, stage].plot(w_1, ft.db(np.abs(response_1)))
        axs[1, stage].set_xlim([0, w_1[-1]])
        axs[1, stage].grid(True)
        if stage == 0:
            axs[1, stage].set_ylim([-155, 5])
        else:
            axs[1, stage].set_ylim([-120, 10])

        axs[2, stage].plot(w_1, ft.db(np.abs(response_1)))
        axs[2, stage].set_xlim([0, fs/2*1.2])
        axs[2, stage].set_ylim([-10, 10])
        axs[2, stage].grid(True)

        if stage == 0:
            axs[0, stage].set_ylabel("filter taps")
            axs[1, stage].set_ylabel("filter response")
            axs[2, stage].set_ylabel("passband ripple")

    # make combined taps by upsampling filter coefficients from last stage backwards
    coeff_combo = coeffs[-1][0]
    for n in range(n_stages - 1):
        coeff_combo = ft.combined_filter(coeffs[-2 - n][0], coeff_combo, coeffs[-2 - n][1])

    coeff_combo = ft.normalise_coeffs(coeff_combo)

    # trim combined response, calculated in frequency domain to avoid artifacts
    w_combo = w_1
    response_combo = response_combo[:len(response_1)]

    # combined
    axs[0, -1].set_title("combined")
    axs[0, -1].plot(coeff_combo)
    axs[0, -1].set_xlim([0, len(coeff_combo)])
    axs[0, -1].grid(True)

    axs[1, -1].plot(w_combo, ft.db(np.abs(response_combo)))
    axs[1, -1].set_xlim([0, (fs * coeffs[stage][-1])/2])
    axs[1, -1].set_ylim([-120, 10])
    axs[1, -1].grid(True)

    axs[2, -1].plot(w_combo, ft.db(np.abs(response_combo)))
    axs[2, -1].set_xlim([0, fs/2*1.2])
    axs[2, -1].set_ylim([-10, 10])
    axs[2, -1].grid(True)

    return axs


def main():
    # load default coefficients & decimation factors
    coeffs = np.load(Path(Path(__file__).parent, "..", "..", "tests", "signal", "BasicMicArray", "default_filters.pkl"),
                     allow_pickle=True)

    # sample rates and decimations
    fs_0 = 3072000
    fs_1 = fs_0 / coeffs[0][1]
    fs_2 = fs_1 / coeffs[1][1]

    axs_1 = plot_stage(coeffs[0], fs_0, fs_2)
    axs_2 = plot_stage(coeffs[1], fs_1, fs_2)
    axs = plot_filters(coeffs, fs_0)
    plt.show()


if __name__ == "__main__":
    main()
