import numpy as np

# --------------------------------------------------------------
# WindFieldReconstruction
# matlab version of the subroutine WindFieldReconstruction in LDP_v1_Subs.f90
# extended to deal with invalid data.
# --------------------------------------------------------------
class WindFieldReconstruction:
    def __init__(self, NumberOfBeams, AngleToCenterline):
        self.NumberOfBeams = NumberOfBeams
        self.AngleToCenterline = AngleToCenterline
        self.u_est_Buffer = np.full(NumberOfBeams, np.nan)

    def __call__(self, v_los, isValid):
        if isValid:
            u_est = v_los / np.cos(np.deg2rad(self.AngleToCenterline))
        else:
            u_est = np.nan

        self.u_est_Buffer = np.insert(self.u_est_Buffer[:-1], 0, u_est)
        REWS = np.nanmean(self.u_est_Buffer)
        return REWS

# --------------------------------------------------------------
# LPFilter
# matlab version of the function LPFilter in FFP_v1_Subs.f90
# --------------------------------------------------------------
class LPFilter:
    def __init__(self, DT, CornerFreq):
        self.DT = DT
        self.CornerFreq = CornerFreq
        self.OutputSignalLast = None
        self.InputSignalLast = None

        a1 = 2 + CornerFreq * DT
        a0 = CornerFreq * DT - 2
        b1 = CornerFreq * DT
        b0 = CornerFreq * DT

        self.coeffs = dict(a1=a1, a0=a0, b1=b1, b0=b0)

    def __call__(self, InputSignal):
        if self.OutputSignalLast is None:
            self.OutputSignalLast = InputSignal
            self.InputSignalLast = InputSignal

        a1 = self.coeffs['a1']
        a0 = self.coeffs['a0']
        b1 = self.coeffs['b1']
        b0 = self.coeffs['b0']

        OutputSignal = (1.0 / a1) * (
            -a0 * self.OutputSignalLast
            + b1 * InputSignal
            + b0 * self.InputSignalLast
        )

        self.InputSignalLast = InputSignal
        self.OutputSignalLast = OutputSignal

        return OutputSignal

# --------------------------------------------------------------
# Buffer
# --------------------------------------------------------------
class Buffer:
    def __init__(self, DT, T_buffer):
        nBuffer = 2000
        self.nBuffer = nBuffer
        self.DT = DT
        self.T_buffer = T_buffer
        self.REWS_f_Buffer = None

        idx = int(np.floor(T_buffer / DT))
        self.Idx = min(max(idx, 1), nBuffer) - 1

    def __call__(self, REWS):
        if self.REWS_f_Buffer is None:
            self.REWS_f_Buffer = np.full(self.nBuffer, REWS)

        self.REWS_f_Buffer = np.insert(self.REWS_f_Buffer[:-1], 0, REWS)
        REWS_b = self.REWS_f_Buffer[self.Idx]
        return REWS_b

# --------------------------------------------------------------
# LDP_v3
# --------------------------------------------------------------
def LDP_v3(time, isValid, beamID, lineOfSightWindSpeed, DT, LDP):
    PreviousBeamID = -1
    n_t = len(time)

    REWS = np.full(n_t, np.nan)
    REWS_f = np.full(n_t, np.nan)
    REWS_b = np.full(n_t, np.nan)

    WFR = WindFieldReconstruction(
        LDP['NumberOfBeams'], LDP['AngleToCenterline']
    )

    if LDP['FlagLPF']:
        lpf = LPFilter(DT, LDP['omega_cutoff'])
    else:
        lpf = None

    buffer = Buffer(DT, LDP['T_buffer'])

    for i_t in range(n_t):
        if beamID[i_t] != PreviousBeamID:
            v_los_i = lineOfSightWindSpeed[i_t]
            isValid_i = bool(isValid[i_t])
            REWS_i = WFR(v_los_i, isValid_i)
            PreviousBeamID = beamID[i_t]

        if LDP['FlagLPF']:
            REWS_f_i = lpf(REWS_i)
        else:
            REWS_f_i = REWS_i

        REWS_b_i = buffer(REWS_f_i)

        REWS[i_t] = REWS_i
        REWS_f[i_t] = REWS_f_i
        REWS_b[i_t] = REWS_b_i

    return REWS, REWS_f, REWS_b