"""Floating-point reference GMMs used in the evaluation."""
import numpy as np


class FloatOpenCVGMM:
    """OpenCV GMM as described by Genovese & Napoli (eq. 1-8), double
    precision, exact (unquantized) alpha_k = alpha_w / w_k."""

    def __init__(self, gray0, alpha=2 ** -7, lam=2.5, T=0.7, var_init=900.0,
                 var_min=16.0, var_max=16383.0, K=3):
        h, w = gray0.shape
        self.K, self.a, self.lam, self.T = K, alpha, lam, T
        self.vi, self.vmin, self.vmax = var_init, var_min, var_max
        self.w = np.zeros((K, h, w))
        self.mu = np.zeros((K, h, w))
        self.var = np.full((K, h, w), var_init)
        self.ms = np.zeros((K, h, w))
        self.w[0] = 1.0
        self.mu[0] = gray0
        self.ms[0] = 1

    def apply(self, x, alpha=None):
        if alpha is not None:
            self.a = alpha
        x = x.astype(np.float64)
        K = self.K
        F = np.where(self.w > 0, self.w / np.sqrt(self.var), -1.0)
        order = np.argsort(-F, axis=0, kind="stable")
        w = np.take_along_axis(self.w, order, 0)
        mu = np.take_along_axis(self.mu, order, 0)
        var = np.take_along_axis(self.var, order, 0)
        ms = np.take_along_axis(self.ms, order, 0)
        d2 = (x[None] - mu) ** 2
        match = (d2 < self.lam ** 2 * var) & (w > 0)
        found = match.any(0)
        m = np.argmax(match, 0)
        isbg = (np.cumsum(w, 0) - w) <= self.T
        fg = ~(found & np.take_along_axis(isbg, m[None], 0)[0])
        M = np.zeros_like(w)
        np.put_along_axis(M, m[None], found[None].astype(float), 0)
        ak = np.minimum(self.a / np.maximum(w, 1e-12), 1.0)
        upd = M > 0
        nmu = np.where(upd, mu + ak * (x[None] - mu), mu)
        nvar = np.where(upd, np.clip(var + ak * (d2 - var), self.vmin, self.vmax), var)
        nw = w - self.a * w + self.a * M
        nms = ms + M
        nf = ~found
        msum = ms[0] + ms[1]
        nw[K - 1][nf] = np.where(msum[nf] > 0, 1.0 / np.maximum(msum[nf], 1), 1.0)
        nmu[K - 1][nf] = x[nf]
        nvar[K - 1][nf] = self.vi
        nms[K - 1][nf] = 1
        self.w, self.mu, self.var, self.ms = nw, nmu, nvar, nms
        return fg


class FloatStaufferGrimson:
    """Classic Stauffer-Grimson GMM (rank w/sigma, normalised weights)."""

    def __init__(self, gray0, K=3, alpha=1 / 128, rho=1 / 64, lam=2.5, T=0.7,
                 var_init=225.0, var_min=16.0, var_max=900.0, w_init=0.05):
        h, w = gray0.shape
        self.K, self.a, self.r, self.lam, self.T = K, alpha, rho, lam, T
        self.vi, self.vmin, self.vmax, self.wi = var_init, var_min, var_max, w_init
        self.w = np.zeros((K, h, w))
        self.mu = np.zeros((K, h, w))
        self.var = np.full((K, h, w), var_init)
        self.w[0] = 1.0
        self.mu[0] = gray0

    def apply(self, x):
        x = x.astype(np.float64)
        K = self.K
        order = np.argsort(-self.w / np.sqrt(self.var), axis=0)
        w = np.take_along_axis(self.w, order, 0)
        mu = np.take_along_axis(self.mu, order, 0)
        var = np.take_along_axis(self.var, order, 0)
        d2 = (x[None] - mu) ** 2
        match = (d2 < self.lam ** 2 * var) & (w > 0)
        found = match.any(0)
        m = np.argmax(match, 0)
        isbg = (np.cumsum(w, 0) - w) < self.T
        fg = ~(found & np.take_along_axis(isbg, m[None], 0)[0])
        M = np.zeros_like(w)
        np.put_along_axis(M, m[None], found[None].astype(float), 0)
        w = (1 - self.a) * w + self.a * M
        upd = M > 0
        mu = np.where(upd, mu + self.r * (x[None] - mu), mu)
        var = np.where(upd, np.clip(var + self.r * (d2 - var), self.vmin, self.vmax), var)
        nf = ~found
        w[K - 1][nf] = self.wi
        mu[K - 1][nf] = x[nf]
        var[K - 1][nf] = self.vi
        w /= w.sum(0, keepdims=True)
        self.w, self.mu, self.var = w, mu, var
        return fg
