# Mathematical formulation

This document collects the mathematical model implemented by the coupled
multi-fidelity coverage simulator. It covers the simulated scalar fields,
sensing, the central autoregressive Gaussian process, controller densities,
HEDAC aerial coverage, ground Voronoi coverage, Lloyd control, MPC control,
robot dynamics, asynchronous scheduling, and evaluation metrics.

GitHub renders the equations in this file using inline dollar delimiters and
display equations delimited by double dollar signs. The source can therefore be
read directly on GitHub without an external LaTeX renderer.

## 1. Scope and notation

The equations are marked conceptually as follows:

- **Model** describes the mathematical model or control objective.
- **Implementation** gives the discrete equation evaluated by the current code.
- **Compatibility path** describes retained legacy behavior that is not active
  when the central multi-fidelity estimator supplies the controllers.

The main symbols are:

| Symbol | Meaning |
| --- | --- |
| $\mathbf q=(x,y)^\top$ | A point in the two-dimensional workspace $\Omega$ |
| $\mathbf p_i$ | Position of robot $i$ |
| $\theta_i$ | Heading of robot $i$ |
| $f_L,f_H$ | Latent LOW- and HIGH-fidelity scalar fields |
| $\delta$ | Discrepancy between the scaled LOW field and the HIGH field |
| $\rho$ | Fixed autoregressive scaling coefficient |
| $\mu_H,\sigma_H^2$ | Posterior mean and marginal variance of $f_H$ |
| $\phi$ | Nonnegative, normalized controller importance density |
| $w_j$ | Numerical quadrature weight associated with query point $\mathbf q_j$ |
| $V_{ij}$ | Indicator that query point $j$ belongs to robot $i$'s Voronoi cell |
| $\Delta t$ | Simulation or controller time step |
| $\mathrm{sat}_{[a,b]}(z)$ | Clipping of $z$ to the interval $[a,b]$ |

Unless stated otherwise, the current multi-fidelity path uses one central
posterior for both teams:

- aerial robots use a normalized combination of posterior interest and
  uncertainty;
- ground robots use posterior interest only;
- ground positions affect aerial motion only indirectly, through the locations
  at which ground robots acquire HIGH-fidelity samples;
- ground Voronoi cells are built from ground-robot positions only.

## 2. Simulator ground truth

### 2.1 HIGH-fidelity field

The simulator constructs a smooth HIGH-fidelity field from a Gaussian mixture.
For $K$ components,

$$
g(\mathbf q)
=
\sum_{k=1}^{K}
\pi_k
\frac{
\exp\!\left[
-\frac{1}{2}
(\mathbf q-\boldsymbol\mu_k)^\top
\boldsymbol\Sigma_k^{-1}
(\mathbf q-\boldsymbol\mu_k)
\right]
}{
2\pi\sqrt{\det(\boldsymbol\Sigma_k)}
},
\qquad
\sum_{k=1}^{K}\pi_k=1.
$$

On the raster, this field is min-max scaled and obstacles are masked:

$$
f_H(\mathbf q_j)
=
\mathbf 1_{\mathrm{free}}(\mathbf q_j)
\frac{g(\mathbf q_j)-g_{\min}}
{g_{\max}-g_{\min}+\varepsilon}.
$$

This is the simulator's hidden truth. The estimator receives noisy samples, not
the mixture parameters or the complete raster.

### 2.2 Simulator LOW field and discrepancy

The LOW truth is a normalized Gaussian smoothing of the masked HIGH truth. If
$G_s$ is a Gaussian filter with standard deviation $s$ cells and $M$ is the
free-space mask, the implemented normalized convolution is

$$
f_L
=
\frac{G_s * (M f_H)}{G_s * M},
$$

where division is pointwise wherever the denominator is nonzero, and the result
is zero outside the mask. The simulator then defines

$$
\delta(\mathbf q)=f_H(\mathbf q)-\rho f_L(\mathbf q),
$$

so the autoregressive identity is exact for the simulated fields:

$$
f_H(\mathbf q)=\rho f_L(\mathbf q)+\delta(\mathbf q).
$$

The simulator uses this identity to generate a coherent test problem. The
estimator does not observe the simulator's $\delta$ field directly.

## 3. Sensor model

An observation from fidelity $s\in\{L,H\}$ at position $\mathbf q_n$ is

$$
y_n=f_s(\mathbf q_n)+\epsilon_n,
\qquad
\epsilon_n\sim\mathcal N(0,\tau_{s,n}^2).
$$

The configured sensor noise values are variances, so the noise standard
deviation used to draw a sample is $\sqrt{\tau_{s,n}^2}$. Field values are read
from the nearest raster cell after the continuous sample position is generated.

### 3.1 Uniform-sector sampling

For a robot at $\mathbf p$ with heading $\theta$, sensor range $R$, and total
field-of-view angle $\Psi$, the sensor's only sampling law draws

$$
\alpha\sim
\mathcal U\!\left(\theta-\frac{\Psi}{2},
\theta+\frac{\Psi}{2}\right),
\qquad
U\sim\mathcal U(0,1),
\qquad
r=R\sqrt{U},
$$

and returns

$$
\mathbf q
=
\mathbf p+
r
\begin{bmatrix}
\cos\alpha\\
\sin\alpha
\end{bmatrix}.
$$

The square root makes sample locations uniform with respect to area inside the
sector because the polar area element is $r\,dr\,d\alpha$. Out-of-bounds draws
are resampled up to the configured attempt limit. There is no sampling-mode
parameter or alternative fixed-ray branch in the production sensor.

## 4. Autoregressive multi-fidelity Gaussian process

### 4.1 Prior

The central estimator assumes independent zero-mean Gaussian processes

$$
f_L\sim\mathcal{GP}(0,k_L),
\qquad
\delta\sim\mathcal{GP}(0,k_\delta),
\qquad
f_L\perp\delta,
$$

with

$$
f_H(\mathbf q)=\rho f_L(\mathbf q)+\delta(\mathbf q).
$$

Both covariance functions are isotropic squared-exponential kernels:

$$
k_a(\mathbf q,\mathbf q')
=
\sigma_a^2
\exp\!\left(
-\frac{\lVert\mathbf q-\mathbf q'\rVert_2^2}
{2\ell_a^2}
\right),
\qquad
a\in\{L,\delta\}.
$$

Here $\ell_a$ is a length scale and $\sigma_a^2$ is the configured kernel
variance.

### 4.2 What is known and what is learned

| Quantity | Current treatment |
| --- | --- |
| Kernel family | Known: isotropic squared-exponential |
| Autoregressive structure | Known: $f_H=\rho f_L+\delta$ |
| $\rho$ | Fixed from configuration |
| Sensor noise variances | Fixed from configuration and stored per observation |
| LOW and HIGH sample positions and values | Observed |
| $\ell_L,\sigma_L^2,\ell_\delta,\sigma_\delta^2$ | Optionally fitted by marginal likelihood |
| Latent field values away from samples | Inferred as a posterior distribution |
| $\mu_H(\mathbf q),\sigma_H^2(\mathbf q)$ | Recomputed on the query grid after a successful update |
| Simulator truth, GMM parameters, and true discrepancy | Unknown to the estimator |

Fitting hyperparameters is empirical Bayes: it produces one bounded
maximum-marginal-likelihood point estimate. It does not maintain a posterior
distribution over the hyperparameters.

### 4.3 Joint observation covariance

Let

$$
\mathbf y
=
\begin{bmatrix}
\mathbf y_L\\
\mathbf y_H
\end{bmatrix},
$$

with LOW inputs $X_L$, HIGH inputs $X_H$, and diagonal observation-noise
matrices $D_L$ and $D_H$. In LOW-then-HIGH order, the training covariance is

$$
K_y
=
\begin{bmatrix}
K_L(X_L,X_L)+D_L
&
\rho K_L(X_L,X_H)
\\
\rho K_L(X_H,X_L)
&
\rho^2 K_L(X_H,X_H)+K_\delta(X_H,X_H)+D_H
\end{bmatrix}.
$$

The implementation adds adaptive diagonal jitter:

$$
\widetilde K_y=K_y+\eta I.
$$

Starting from the configured $\eta$, failed Cholesky attempts multiply it by
the configured jitter multiplier. No explicit matrix inverse is formed.

### 4.4 Cholesky solve

For

$$
LL^\top=\widetilde K_y,
$$

the implementation solves

$$
L\mathbf z=\mathbf y,
\qquad
L^\top\boldsymbol\alpha=\mathbf z.
$$

Thus

$$
\boldsymbol\alpha=\widetilde K_y^{-1}\mathbf y
$$

conceptually, while the numerical computation uses triangular solves.

### 4.5 HIGH-fidelity posterior

For query points $X_*$, the covariance between training observations and the
latent HIGH field is

$$
K_{y*}
=
\begin{bmatrix}
\rho K_L(X_L,X_*)
\\
\rho^2 K_L(X_H,X_*)+K_\delta(X_H,X_*)
\end{bmatrix}.
$$

The prior HIGH covariance is

$$
K_{**}
=
\rho^2K_L(X_*,X_*)+K_\delta(X_*,X_*).
$$

The posterior mean is

$$
\boldsymbol\mu_H
=
K_{y*}^\top\boldsymbol\alpha.
$$

If $V$ solves

$$
LV=K_{y*},
$$

then the posterior marginal variance at query point $j$ is

$$
\sigma_{H,j}^2
=
[K_{**}]_{jj}
-
\sum_r V_{rj}^2.
$$

Only tiny negative values attributable to floating-point roundoff are clipped
to zero. Materially negative variances cause the candidate update to fail.

### 4.6 Hyperparameter fitting

The four positive kernel parameters are optimized in log space:

$$
\boldsymbol\xi
=
\log
\begin{bmatrix}
\ell_L &
\sigma_L^2 &
\ell_\delta &
\sigma_\delta^2
\end{bmatrix}^{\!\top}.
$$

The bounded L-BFGS-B objective is the negative log marginal likelihood

$$
\mathcal L(\boldsymbol\xi)
=
\frac{1}{2}\mathbf y^\top\boldsymbol\alpha
+
\sum_{r=1}^{N}\log L_{rr}
+
\frac{N}{2}\log(2\pi).
$$

Fitting occurs only when optimization is enabled, the retained sample count
reaches its configured minimum, and the current posterior version satisfies the
configured update interval. The value of $\rho$ and both sensor-noise models
remain fixed.

### 4.7 Query-grid quadrature

For each axis, the estimator uses equally spaced query coordinates and
trapezoidal weights. With spacing $h_x$,

$$
w^x_j
=
\begin{cases}
h_x/2, & j\text{ is an endpoint},\\
h_x, & \text{otherwise},
\end{cases}
$$

and similarly for $w^y_k$. The two-dimensional weight is

$$
w_{jk}=w^x_jw^y_k.
$$

These weights approximate integrals and are the $w_j$ used in density
normalization and Lloyd centroid calculations.

## 5. From the GP posterior to controller densities

### 5.1 Positive posterior interest

The shared posterior interest field is the positive part of the HIGH posterior
mean:

$$
\mu_H^+(\mathbf q_j)
=
\max\!\left(\mu_H(\mathbf q_j),0\right).
$$

For a free-space mask $M_j\in\{0,1\}$ and quadrature weights $w_j$, the
published density is

$$
\phi_j
=
\frac{
M_j\mu_H^+(\mathbf q_j)
}{
\sum_k M_k\mu_H^+(\mathbf q_k)w_k
}.
$$

Therefore

$$
\phi_j\ge 0,
\qquad
\sum_j\phi_jw_j=1.
$$

The clipping preserves all positive posterior contrast. It does not exponentiate
the field, shift negative values upward, or retain the removed softplus
transformation. If every posterior mean value is nonpositive, normalization has
zero mass and the estimator transaction preserves the last valid snapshot.

### 5.2 Aerial interest-plus-uncertainty target

Let

$$
\bar\sigma_{H,j}
=
\begin{cases}
\sigma_{H,j}/\max_k\sigma_{H,k},
& \max_k\sigma_{H,k}>0,\\
0, & \text{otherwise}.
\end{cases}
$$

The unnormalized aerial importance is

$$
\widetilde\phi^{\,A}_j
=
\lambda_I\phi_j+\lambda_U\bar\sigma_{H,j},
$$

where $\lambda_I\ge 0$ is the interest weight and $\lambda_U\ge 0$ is the
uncertainty weight. After masking and quadrature normalization,

$$
\phi^A_j
=
\frac{M_j\widetilde\phi^{\,A}_j}
{\sum_kM_k\widetilde\phi^{\,A}_kw_k}.
$$

This density is bilinearly resampled onto the aerial HEDAC raster and normalized
again there. All aerial robots see the same target; HEDAC coverage history and
their different positions make their individual motions differ.

### 5.3 Ground importance

The ground controllers use

$$
\phi^G_j=\phi_j.
$$

Posterior variance is intentionally absent from the current ground objective.
If the controller grid differs from the posterior grid, the density is
bilinearly interpolated and quadrature-normalized again.

## 6. HEDAC aerial coverage

### 6.1 Coverage footprint

For an offset $\mathbf r$ from an aerial robot, the discrete coverage footprint
is

$$
\kappa(\mathbf r)
=
\exp\!\left(
-\frac{\lVert\mathbf r\rVert_2^2}{R_a}
\right),
$$

because the implementation uses the RBF shape parameter $1/R_a$. The square
footprint is truncated using the configured minimum kernel value
$\kappa_{\min}$. With workspace dimension $d=2$, its per-axis half-width is

$$
h_\kappa
=
\left\lceil
\sqrt{
\frac{-R_a\log\kappa_{\min}}{d}
}
\right\rceil.
$$

At HEDAC step $n$, the cumulative raster coverage is updated by adding one
footprint for every aerial robot:

$$
C_j^{n+1}
=
C_j^n+\sum_i\kappa(\mathbf q_j-\mathbf p_i^n),
$$

with each footprint cropped at map boundaries.

### 6.2 Coverage deficit and source

HEDAC uses sum normalization on its raster:

$$
\widehat C_j
=
\frac{M_j C_j}{\sum_kM_kC_k+\varepsilon}.
$$

The current target $\phi^A$ is also sum-normalized on this raster. The positive
coverage deficit and source are

$$
d_j=\max(\phi^A_j-\widehat C_j,0),
\qquad
\widetilde S_j=d_j^2,
$$

$$
S_j
=
A_\Omega
\frac{M_j\widetilde S_j}
{\sum_kM_k\widetilde S_k+\varepsilon},
$$

where $A_\Omega$ is the free map area.

### 6.3 Heat model

The intended heat-equation form is

$$
\frac{\partial T}{\partial t}
=
\alpha\nabla^2T
+
sS
-
\beta T
-
\gamma L_c,
$$

where $\alpha$ is diffusion, $s$ is source strength, $\beta$ is global cooling,
and $L_c$ is local cooling.

The current non-optimized implementation evaluates the following five-point
interior stencil exactly:

$$
\mathcal D_\alpha[T]_{r,c}
=
\alpha
\left(
T_{r+1,c}+T_{r-1,c}+T_{r,c+1}+T_{r,c-1}
\right)
-
4T_{r,c}.
$$

Then

$$
T_{r,c}^{n+1}
=
T_{r,c}^{n}
+
\Delta t_H
\left[
\frac{\mathcal D_\alpha[T^n]_{r,c}}{\Delta x^2}
+
sS_{r,c}
-
\frac{\beta}{A_\Omega}T_{r,c}^n
-
\frac{\gamma}{A_\Omega}(L_c)_{r,c}
\right].
$$

This discrete stencil is recorded exactly as implemented: the central
$-4T_{r,c}$ term is not multiplied by $\alpha$. It therefore differs from the
usual discretization
$\alpha(T_{\mathrm{neighbors}}-4T_{r,c})/\Delta x^2$ when
$\alpha\ne1$.

The active local-cooling raster is reset to zero every step and footprint
accumulation into it is disabled, so the local-cooling contribution is
currently zero. The non-optimized update modifies interior cells only, leaves
outer boundary cells fixed, and does not explicitly skip obstacle cells during
diffusion.

The explicit heat step is limited by

$$
\Delta t_H
=
\min\!\left(
\Delta t,
\frac{c_{\mathrm{CFL}}\Delta x^2}{4\alpha}
\right)
$$

for $\alpha>0$, and $\Delta t_H=\Delta t$ for $\alpha=0$.

### 6.4 Heat-gradient direction

The raster gradients are computed by finite differences. They are first
globally scaled by

$$
g_{\mathrm{rms}}
=
\sqrt{
\mathrm{mean}(T_x^2)
+
\mathrm{mean}(T_y^2)
},
$$

when this value is nonzero. The scaled gradient is bilinearly interpolated at
the robot position. Close to the outer boundary, the relevant component is
replaced by a fixed inward value of magnitude $10$. Obstacle-wall avoidance is
not active in this gradient function.

Finally, only the direction is retained:

$$
\mathbf d_i
=
\begin{cases}
\nabla T(\mathbf p_i)/\lVert\nabla T(\mathbf p_i)\rVert_2,
& \lVert\nabla T(\mathbf p_i)\rVert_2>0,\\
\mathbf 0, & \text{otherwise}.
\end{cases}
$$

The target velocity and heading are

$$
\mathbf v_i^\star=v_{\max}\mathbf d_i,
\qquad
\theta_i^\star=\mathrm{atan2}(d_{i,y},d_{i,x}).
$$

## 7. Robot dynamics

### 7.1 Aerial Dubins dynamics

The canonical aerial configuration uses fixed-speed Dubins dynamics:

$$
\dot x_i=v_A\cos\theta_i,
\qquad
\dot y_i=v_A\sin\theta_i,
\qquad
\dot\theta_i=u_i.
$$

The coordinated-turn limit derived from maximum bank angle $\varphi_{\max}$ is

$$
u_{\max}
=
\frac{g\tan\varphi_{\max}}{v_A}.
$$

The HEDAC heading tracker uses

$$
e_{\theta,i}
=
\mathrm{atan2}
\left(
\sin(\theta_i^\star-\theta_i),
\cos(\theta_i^\star-\theta_i)
\right),
$$

$$
u_i
=
\mathrm{sat}_{[-u_{\max},u_{\max}]}
\left(2e_{\theta,i}\right).
$$

The implementation updates heading first and then position:

$$
\theta_i^{n+1}
=
\mathrm{wrap}
\left(\theta_i^n+u_i^n\Delta t_A\right),
$$

$$
\mathbf p_i^{n+1}
=
\mathbf p_i^n
+
v_A\Delta t_A
\begin{bmatrix}
\cos\theta_i^{n+1}\\
\sin\theta_i^{n+1}
\end{bmatrix}.
$$

Positions are clipped to the map after the controller step.

### 7.2 Ground unicycle dynamics

Both current ground controllers require or predict the unicycle model

$$
\dot x_i=v_i\cos\theta_i,
\qquad
\dot y_i=v_i\sin\theta_i,
\qquad
\dot\theta_i=\omega_i.
$$

The real ground agent clips $(v_i,\omega_i)$ to its configured limits, updates
heading first, and then position:

$$
\theta_i^{n+1}
=
\mathrm{wrap}
\left(\theta_i^n+\omega_i^n\Delta t_G\right),
$$

$$
\mathbf p_i^{n+1}
=
\mathbf p_i^n
+
v_i^n\Delta t_G
\begin{bmatrix}
\cos\theta_i^{n+1}\\
\sin\theta_i^{n+1}
\end{bmatrix}.
$$

The MPC prediction model uses forward Euler with the old heading instead:

$$
\begin{bmatrix}
x_{h+1}\\y_{h+1}\\\theta_{h+1}
\end{bmatrix}
=
\begin{bmatrix}
x_h+v_h\cos\theta_h\,\Delta t\\
y_h+v_h\sin\theta_h\,\Delta t\\
\theta_h+\omega_h\Delta t
\end{bmatrix}.
$$

This one-step discretization difference is part of the present implementation.

## 8. Ground-only Voronoi partition

For ground positions $\{\mathbf p_1,\ldots,\mathbf p_R\}$, query point
$\mathbf q_j$ is assigned by

$$
i^\star(j)
=
\mathrm*{arg\,min}_{i\in\{1,\ldots,R\}}
\lVert\mathbf q_j-\mathbf p_i\rVert_2^2,
$$

$$
V_{ij}
=
\begin{cases}
1, & i=i^\star(j)
\text{ and }
\lVert\mathbf q_j-\mathbf p_i\rVert_2\le R_V,\\
0, & \text{otherwise}.
\end{cases}
$$

Only ground positions are passed to this partition. Aerial positions do not
compete for ground Voronoi cells. The Lloyd controller chooses a range larger
than the map diagonal, so its cells cover the entire query grid. The MPC
controller currently uses $R_V=50$ in map-coordinate units.

## 9. Lloyd ground controller

### 9.1 Continuous coverage objective

The classical locational coverage objective is

$$
\mathcal H(\mathbf p_1,\ldots,\mathbf p_R)
=
\sum_{i=1}^{R}
\int_{\mathcal V_i}
\lVert\mathbf q-\mathbf p_i\rVert_2^2
\phi^G(\mathbf q)\,d\mathbf q.
$$

For fixed Voronoi cells, its minimizer places each generator at the
density-weighted centroid of its cell.

### 9.2 Discrete mass and centroid

The implemented cell mass is

$$
m_i
=
\sum_j
V_{ij}\phi^G_jw_j.
$$

The centroid is

$$
\mathbf c_i
=
\frac{
\sum_j
\mathbf q_jV_{ij}\phi^G_jw_j
}{
m_i
}.
$$

Thus $w_j$ is the area represented by grid point $j$ under trapezoidal
quadrature. On a uniform interior grid it is approximately
$\Delta x\Delta y$; boundary weights are halved per boundary axis. These
weights are what make the vectorized sum approximate the slower nested
area-integral calculation.

If $m_i$ is numerically zero, the current robot position is used as the
centroid.

### 9.3 Centroid tracking

Define

$$
\mathbf e_i=\mathbf c_i-\mathbf p_i,
\qquad
r_i=\lVert\mathbf e_i\rVert_2,
\qquad
\theta_i^\star=\mathrm{atan2}(e_{i,y},e_{i,x}),
$$

$$
e_{\theta,i}
=
\mathrm{wrap}(\theta_i^\star-\theta_i).
$$

Outside the centroid tolerance, the controls are

$$
v_i
=
\mathrm{sat}_{[-v_{\max},v_{\max}]}
\left(k_p r_i\cos e_{\theta,i}\right),
$$

$$
\omega_i
=
\mathrm{sat}_{[-\omega_{\max},\omega_{\max}]}
\left(k_\theta e_{\theta,i}\right).
$$

Inside the tolerance, both controls are zero. The factor
$\cos e_{\theta,i}$ reduces forward motion when the centroid is lateral and
allows bounded reverse motion when it lies behind the robot.

## 10. MPC ground controller

### 10.1 Per-robot importance weights

Robot $i$ receives the masked weights

$$
W_{ij}=V_{ij}\phi^G_j.
$$

These weights do not include quadrature factors in the current MPC objective.

### 10.2 Implemented limited-FOV stage cost

For predicted state
$\mathbf x_h=(x_h,y_h,\theta_h)^\top$, define

$$
r_{hj}^2
=
\left\lVert
\mathbf q_j-
\begin{bmatrix}x_h\\y_h\end{bmatrix}
\right\rVert_2^2,
$$

$$
a_{hj}
=
\mathrm{atan2}
(q_{j,y}-y_h,q_{j,x}-x_h)-\theta_h,
$$

$$
F_{hj}=2-\frac{r_{hj}^2}{R_F^2},
\qquad
M_{hj}
=
\exp\!\left[
-\frac{(a_{hj}-\Psi/2)^2}{2(\Psi/2)^2}
\right].
$$

The exact current stage cost is

$$
\ell_i(\mathbf x_h)
=
-
\sum_jF_{hj}M_{hj}W_{ij}
-
3\sum_j
W_{ij}
\exp\!\left(
-\frac{r_{hj}^2}{3R_F^2}
\right).
$$

This formula is documented as implemented. In particular, the angular Gaussian
is centered at $+\Psi/2$, not at zero, and the relative angle is not explicitly
wrapped. The function parameter intended for mask steepness is currently
unused.

### 10.3 Finite-horizon problem

With horizon $H$, the solver computes

$$
\min_{\{v_h,\omega_h\}_{h=0}^{H-1}}
\quad
10\sum_{h=0}^{H-1}\ell_i(\mathbf x_h)
$$

subject to the unicycle prediction dynamics and control bounds

$$
-a_{\max}\le v_h\le a_{\max},
\qquad
-a_{\max}\le\omega_h\le a_{\max}.
$$

The same configured quantity named maximum acceleration is currently used as
the bound for both MPC control components. Only the terminal predicted position
is constrained:

$$
0\le x_H\le W_{\mathrm{map}},
\qquad
0\le y_H\le H_{\mathrm{map}}.
$$

There is currently no active collision penalty, control-effort penalty, or
orientation penalty in this optimization. IPOPT is warm-started from the
previous solution, and only the first optimized pair $(v_0,\omega_0)$ is sent
to the real unicycle agent.

### 10.4 Other retained cost functions

The codebase also defines, but the current coupled MPC does not use,

$$
J_{\mathrm{coverage}}(\mathbf p_i)
=
\sum_j
\lVert\mathbf q_j-\mathbf p_i\rVert_2^2W_{ij},
$$

and the collision penalty

$$
J_{\mathrm{collision}}
=
\alpha_c
\exp\!\left[
-\beta_c
\left(
\lVert\mathbf p_i-\mathbf p_{\mathrm{obs}}\rVert_2^2-D_s^2
\right)
\right].
$$

They are included here to distinguish available mathematical utilities from the
active MPC objective.

## 11. Asynchronous estimation and causal order

For a periodic event with period $P$, start time $t_0$, and fire count $k$, the
next scheduled time is computed without accumulated drift:

$$
t_{\mathrm{next}}=t_0+kP.
$$

An event is due when

$$
t+\varepsilon_t\ge t_{\mathrm{next}}.
$$

At simulation step $n$, $t_n=n\Delta t$. The current coupled order is:

1. collect LOW samples from aerial robots if due;
2. read the latest already-published posterior and move aerial robots with
   HEDAC;
3. collect HIGH samples from ground robots if due;
4. update and publish the central GP if due;
5. read that posterior density and move ground robots.

Consequently, a HIGH sample collected at step $n$ can influence ground control
at step $n$ after a successful update, but influences aerial control no earlier
than step $n+1$.

Estimator publication is transactional. A candidate update must complete
retention, optional hyperparameter fitting, Cholesky factorization, prediction,
positive-part conversion, and density validation before it replaces the cached
posterior.

## 12. Evaluation quantities

### 12.1 Current HEDAC coverage error

The quantity reported by HEDAC as an ergodic metric is a discrete spatial
$L^2$ difference. With sum-normalized cumulative coverage

$$
\widehat C_j
=
\frac{C_j}{\sum_kC_k+\varepsilon},
$$

the reported value is

$$
E
=
\sqrt{
\sum_j
\left[
M_j(\widehat C_j-\phi_j^{\mathrm{reference}})
\right]^2
}.
$$

This is not a Fourier-coefficient ergodic metric. In the present HEDAC step,
$\phi^{\mathrm{reference}}$ is the original normalized goal field stored when
HEDAC was constructed, not necessarily the latest external multi-fidelity
aerial target. It should therefore be interpreted carefully in coupled runs.

### 12.2 Posterior reconstruction diagnostics

Useful field diagnostics include root mean squared error

$$
\mathrm{RMSE}
=
\sqrt{
\frac{1}{N}
\sum_{j=1}^{N}
\left(
\mu_H(\mathbf q_j)-f_H(\mathbf q_j)
\right)^2
},
$$

and mean negative log predictive density, when evaluating against simulator
truth:

$$
\mathrm{MNLPD}
=
\frac{1}{N}
\sum_{j=1}^{N}
\left[
\frac{1}{2}\log(2\pi\sigma_{H,j}^2)
+
\frac{
\left(f_H(\mathbf q_j)-\mu_H(\mathbf q_j)\right)^2
}{
2\sigma_{H,j}^2
}
\right].
$$

These diagnostics use hidden truth only for evaluation; they are not controller
inputs.

## 13. Legacy compatibility path

When no central multi-fidelity posterior is supplied, the retained ground path
can fuse separate aerial and ground GP estimates. Let
$\bar\sigma_A,\bar\sigma_G$ be each standard-deviation field divided by its own
maximum. The implemented weights are

$$
d_j
=
\frac{1}{\bar\sigma_{G,j}+\varepsilon}
+
\frac{1}{\bar\sigma_{A,j}+\varepsilon},
$$

$$
\lambda_{A,j}
=
\frac{
1/(\bar\sigma_{A,j}+\varepsilon)
}{d_j},
\qquad
\lambda_{G,j}
=
\frac{
1/(\bar\sigma_{G,j}+\varepsilon)
}{d_j},
$$

and

$$
\mu_{\mathrm{fused},j}
=
\lambda_{A,j}\mu_{A,j}
+
\lambda_{G,j}\mu_{G,j}.
$$

This is a compatibility fallback, not the central autoregressive GP. In
multi-fidelity mode the controllers consume the shared posterior products
defined in Section 5.

## 14. Equation-to-code map

| Mathematical component | Primary implementation |
| --- | --- |
| HIGH truth and Gaussian mixture | [src/core/gmm.py](../src/core/gmm.py), [src/coupled_simulation.py](../src/coupled_simulation.py) |
| LOW smoothing and sensor observations | [src/models/sensors.py](../src/models/sensors.py) |
| Autoregressive GP and marginal likelihood | [src/core/multifidelity_gp.py](../src/core/multifidelity_gp.py) |
| Asynchronous estimator and posterior publication | [src/core/multifidelity_estimator.py](../src/core/multifidelity_estimator.py) |
| Density clipping, normalization, and aerial target | [src/core/density.py](../src/core/density.py) |
| Event schedule and controller-density caching | [src/simulation.py](../src/simulation.py) |
| HEDAC source, heat update, and gradient law | [src/core/hedac.py](../src/core/hedac.py), [src/utils/math_utils.py](../src/utils/math_utils.py) |
| Dubins and unicycle state updates | [src/models/agents.py](../src/models/agents.py) |
| MPC prediction dynamics | [src/models/models.py](../src/models/models.py) |
| Voronoi partition and Lloyd centroids | [src/utils/voronoi.py](../src/utils/voronoi.py), [src/coupled_simulation.py](../src/coupled_simulation.py) |
| MPC objective | [src/core/costFunctions.py](../src/core/costFunctions.py), [src/coupled_simulation.py](../src/coupled_simulation.py) |
| Coverage error | [src/core/base.py](../src/core/base.py) |

The associated configuration parameters and their tuning effects are documented
in [Multi-fidelity configuration reference](multifidelity_config_reference.md).
