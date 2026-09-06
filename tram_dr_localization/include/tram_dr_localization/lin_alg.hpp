#pragma once
// Small dense linear algebra for L<=16. No Eigen. ROS-free.

#include <algorithm>
#include <cmath>

namespace tram_dr {
namespace la {

constexpr int kCap = 16;

inline void zero(double* a, int n) {
  for (int i = 0; i < n; ++i) {
    a[i] = 0.0;
  }
}

inline void copy(const double* src, double* dst, int n) {
  for (int i = 0; i < n; ++i) {
    dst[i] = src[i];
  }
}

// True when every one of the first n entries is finite. Callers that own a
// covariance are expected to use this before handing it to project_pd(); see
// the finiteness contract documented there.
inline bool all_finite(const double* a, int n) {
  if (!a || n < 0) {
    return false;
  }
  for (int i = 0; i < n; ++i) {
    if (!std::isfinite(a[i])) {
      return false;
    }
  }
  return true;
}

inline void add_scaled(const double* a, double s, const double* b, double* out, int n) {
  for (int i = 0; i < n; ++i) {
    out[i] = a[i] + s * b[i];
  }
}

// A, B, C are n x n row-major.
inline double& at(double* A, int n, int i, int j) { return A[i * n + j]; }
inline double at(const double* A, int n, int i, int j) { return A[i * n + j]; }

inline void eye(double* A, int n) {
  for (int i = 0; i < n * n; ++i) {
    A[i] = 0.0;
  }
  for (int i = 0; i < n; ++i) {
    at(A, n, i, i) = 1.0;
  }
}

inline void add_diag(double* A, int n, const double* diag) {
  for (int i = 0; i < n; ++i) {
    at(A, n, i, i) += diag[i];
  }
}

inline void mat_vec(const double* A, int n, const double* x, double* y) {
  for (int i = 0; i < n; ++i) {
    double s = 0.0;
    for (int j = 0; j < n; ++j) {
      s += at(A, n, i, j) * x[j];
    }
    y[i] = s;
  }
}

inline void mat_mul(const double* A, const double* B, double* C, int n) {
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      double s = 0.0;
      for (int k = 0; k < n; ++k) {
        s += at(A, n, i, k) * at(B, n, k, j);
      }
      at(C, n, i, j) = s;
    }
  }
}

// Lower Cholesky of A + jitter*I. False for invalid dimensions/data or non-SPD.
inline bool chol(const double* A, double* L, int n, double jitter = 1e-9) {
  if (!A || !L || n <= 0 || n > kCap || !std::isfinite(jitter) || jitter < 0)
    return false;
  if (!all_finite(A, n * n)) return false;
  double work[kCap * kCap];
  for (int i = 0; i < n * n; ++i) {
    work[i] = A[i];
  }
  for (int i = 0; i < n; ++i) {
    at(work, n, i, i) += jitter;
  }
  for (int i = 0; i < n * n; ++i) {
    L[i] = 0.0;
  }
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j <= i; ++j) {
      double s = at(work, n, i, j);
      for (int k = 0; k < j; ++k) {
        s -= at(L, n, i, k) * at(L, n, j, k);
      }
      if (i == j) {
        if (!(s > 0.0) || !std::isfinite(s)) {
          return false;
        }
        at(L, n, i, j) = std::sqrt(s);
      } else {
        at(L, n, i, j) = s / at(L, n, j, j);
        if (!std::isfinite(at(L, n, i, j))) return false;
      }
    }
  }
  return true;
}

// Solve A x = b for SPD A via Cholesky. x may alias b.
inline bool solve_spd(const double* A, const double* b, double* x, int n) {
  if (!b || !x || n <= 0 || n > kCap) return false;
  if (!all_finite(b, n)) return false;
  double L[kCap * kCap];
  if (!chol(A, L, n, 1e-12)) {
    return false;
  }
  double y[kCap];
  for (int i = 0; i < n; ++i) {
    double s = b[i];
    for (int j = 0; j < i; ++j) {
      s -= at(L, n, i, j) * y[j];
    }
    y[i] = s / at(L, n, i, i);
  }
  for (int i = n - 1; i >= 0; --i) {
    double s = y[i];
    for (int j = i + 1; j < n; ++j) {
      s -= at(L, n, j, i) * x[j];
    }
    x[i] = s / at(L, n, i, i);
    if (!std::isfinite(x[i])) return false;
  }
  return true;
}

// Invert small SPD S (m x m) into Sinv.
inline bool inv_spd(const double* S, double* Sinv, int m) {
  if (!S || !Sinv || m <= 0 || m > kCap) return false;
  double eye_col[kCap];
  double col[kCap];
  for (int j = 0; j < m; ++j) {
    zero(eye_col, m);
    eye_col[j] = 1.0;
    if (!solve_spd(S, eye_col, col, m)) {
      return false;
    }
    for (int i = 0; i < m; ++i) {
      at(Sinv, m, i, j) = col[i];
    }
  }
  return true;
}

inline void symmetrize(double* A, int n) {
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      const double v = 0.5 * (at(A, n, i, j) + at(A, n, j, i));
      at(A, n, i, j) = v;
      at(A, n, j, i) = v;
    }
  }
}

// Symmetric eigenvalue-floor repair. Not a proof of statistical consistency.
// Valid SPD input above the floor is preserved, rather than re-diagonalized.
//
// Finiteness contract (F-09)
// --------------------------
// Returns true only when the input was finite on entry and the result is
// finite and has every eigenvalue at or above lam_floor. Returns false, and
// leaves A byte-for-byte unchanged, when:
//   * the arguments are invalid (null A, n out of range, non-positive floor);
//   * any entry of A is NaN or +/-Inf on entry;
//   * the Jacobi reconstruction produced a non-finite entry.
//
// A is deliberately NOT sanitized on failure. Overwriting a poisoned
// covariance with a large finite diagonal - the obvious "fail safe" choice -
// would make the caller's subsequent Cholesky health check succeed, so a hard
// NaN fault would be silently downgraded to a plausible-looking
// maximum-uncertainty estimate that the integrity monitor cannot see. Keeping
// the poisoned matrix and reporting false lets the caller do the only correct
// thing: reject the frame, roll back to the last healthy state, and raise the
// fault. Establishing finiteness before the call is therefore the caller's
// duty, and la::all_finite() exists for exactly that.
//
// Callers must not treat a false return as "the matrix is now usable".
[[maybe_unused]] inline bool project_pd(double* A, int n, double lam_floor = 1e-12) {
  if (!A || n <= 0 || n > kCap || !std::isfinite(lam_floor) || lam_floor <= 0) {
    return false;
  }
  if (!all_finite(A, n * n)) return false;
  symmetrize(A, n);
  double test[kCap * kCap], lower[kCap * kCap];
  copy(A, test, n * n);
  for (int i = 0; i < n; ++i) at(test, n, i, i) -= lam_floor;
  if (chol(test, lower, n, 0.0)) return true;
  double V[kCap * kCap];
  eye(V, n);
  // A sweep is O(n^2) rotations; 24 rotations was not 24 sweeps for L=12.
  bool converged = false;
  for (int rotation = 0; rotation < 32 * n * n; ++rotation) {
    double max_off = 0.0;
    int p = 0;
    int q = 1;
    for (int i = 0; i < n; ++i) {
      for (int j = i + 1; j < n; ++j) {
        const double aij = std::fabs(at(A, n, i, j));
        if (aij > max_off) {
          max_off = aij;
          p = i;
          q = j;
        }
      }
    }
    if (max_off < 1e-14) {
      converged = true;
      break;
    }
    const double app = at(A, n, p, p);
    const double aqq = at(A, n, q, q);
    const double apq = at(A, n, p, q);
    double t = 0.0;
    if (std::fabs(apq) > 1e-18) {
      const double theta = 0.5 * (aqq - app) / apq;
      const double sign = (theta >= 0.0) ? 1.0 : -1.0;
      t = sign / (std::fabs(theta) + std::hypot(1.0, theta));
    }
    const double c = 1.0 / std::sqrt(1.0 + t * t);
    const double s = t * c;
    for (int k = 0; k < n; ++k) {
      if (k == p || k == q) {
        continue;
      }
      const double akp = at(A, n, k, p);
      const double akq = at(A, n, k, q);
      at(A, n, k, p) = at(A, n, p, k) = c * akp - s * akq;
      at(A, n, k, q) = at(A, n, q, k) = s * akp + c * akq;
    }
    at(A, n, p, p) = app - t * apq;
    at(A, n, q, q) = aqq + t * apq;
    at(A, n, p, q) = at(A, n, q, p) = 0.0;
    for (int k = 0; k < n; ++k) {
      const double vip = at(V, n, k, p);
      const double viq = at(V, n, k, q);
      at(V, n, k, p) = c * vip - s * viq;
      at(V, n, k, q) = s * vip + c * viq;
    }
  }
  // If the iteration budget is exhausted, retain a conservative Gershgorin
  // bound in this orthogonal basis instead of silently discarding correlations.
  double residual_bound = 0.0;
  if (!converged) {
    for (int i = 0; i < n; ++i) {
      double row = 0.0;
      for (int j = 0; j < n; ++j) if (i != j) row += std::fabs(at(A, n, i, j));
      residual_bound = std::max(residual_bound, row);
    }
  }
  double lam[kCap];
  for (int i = 0; i < n; ++i) {
    lam[i] = std::max(at(A, n, i, i) + residual_bound, lam_floor);
  }
  double tmp[kCap * kCap];
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      double acc = 0.0;
      for (int k = 0; k < n; ++k) {
        acc += at(V, n, i, k) * lam[k] * at(V, n, j, k);
      }
      at(tmp, n, i, j) = acc;
    }
  }
  // Rotations and the reconstruction can overflow on inputs that were finite
  // but enormous. Report that instead of handing back a NaN covariance.
  if (!all_finite(tmp, n * n)) return false;
  for (int i = 0; i < n * n; ++i) {
    A[i] = tmp[i];
  }
  symmetrize(A, n);
  return true;
}

}  // namespace la
}  // namespace tram_dr
