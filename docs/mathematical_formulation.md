# Mathematical formulation

The mathematical reference is maintained as LaTeX source at
[`docs/latex/mathematical_formulation.tex`](latex/mathematical_formulation.tex).

The current rendered version is published by GitHub Actions on GitHub Pages:

[Read the mathematical formulation (PDF)](https://arscontrol.github.io/air_ground_coverage/mathematical_formulation.pdf)

Pull requests that change the LaTeX source attach the compiled PDF as a workflow
artifact. The PDF is generated from source and is intentionally not committed to
the repository.

## Implemented circular ground obstacles

The coupled builder maintains two occupancy rasters. $M^A_{yx}$ is the loaded
aerial/HEDAC map. Generated circles are added only to the ground map $M^G$;
aerial robots can overfly them. Robot motion and controller grids use raster
index coordinates, so the production obstacle rasterizer uses cell centre
$\mathbf q_{yx}=(x,y)^\top$ even when the HEDAC heat discretization has another
physical cell-size parameter. With obstacle count $K_o$ and common radius
$R_o$, the obstacle RNG uses the episode-derived seed
$s_o=s+30000$. Candidate centres are sampled uniformly inside the inset map
boundary and accepted sequentially only when

$$
\|\mathbf c_k-\mathbf c_l\|_2>2R_o
\qquad \text{for every previously accepted }l.
$$

The exact rasterization and ground-map union are

$$
O_{yx}
=
\mathbf 1\!\left[
\min_{k=1,\ldots,K_o}\|\mathbf q_{yx}-\mathbf c_k\|_2\le R_o
\right],
\qquad
M^G_{yx}=\max(M^A_{yx},O_{yx}).
$$

Zero obstacles produce an all-free generated raster. An infeasible radius or
an obstacle set that cannot be placed without overlap is rejected rather than
silently reducing the requested count. By default, robot initial positions
retain the historical uniform-free-cell sampling. The composition experiment
instead uses a common lower-left corner deployment. For robot class
$r\in\{A,G\}$, map width $W$, height $H$, and $\gamma=0.2$, its candidate set
is

$$
\mathcal C^r
=
\{(x,y):M^r_{yx}=0,\ 0\le x<\gamma W,\ 0\le y<\gamma H\}.
$$

The aerial and ground candidate permutations use episode-derived seeds
$s_A=s+40000$ and $s_G=s+50000$, respectively. The first $N_r$ permuted cells
are assigned to class $r$ without replacement; headings are subsequent draws
from $\mathcal U[0,2\pi)$. Thus a class's first initial states are nested
across compositions for the same episode. An undersized candidate set is
rejected rather than expanded silently.

The hidden HIGH field is zeroed on $M^G=1$;
nearest-cell samples of $M^G$ form the estimator and evaluation free-query
mask. Hence the published ground density and every reconstruction metric omit
obstacle queries. Aerial HEDAC motion still uses $M^A$.

For the Lloyd controller, let $\mathcal O_i$ be occupied cell centres within
the influence distance $R_I=\max(2,2.5R_o)$ of ground robot $i$. For
$d_{il}=\|\mathbf p_i-\mathbf o_l\|_2$, the implementation computes

$$
\mathbf r_i=
\operatorname{clip}_{\|\cdot\|\le1}
\left[
\sum_{l\in\mathcal O_i}
\frac{R_I-d_{il}}{R_I}
\frac{\mathbf p_i-\mathbf o_l}{d_{il}}
\right]
$$

and replaces the centroid displacement by

$$
\widetilde{\mathbf e}_i
=
\mathbf c_i-\mathbf p_i
+w_o\max(\|\mathbf c_i-\mathbf p_i\|_2,1)\mathbf r_i,
$$

where $w_o$ is `ground.agents.wall_avoidance_weight`. The ordinary Lloyd
heading and speed law uses $\widetilde{\mathbf e}_i$. A final discrete safety
guard predicts the heading-first unicycle segment and samples it at spacing no
greater than $0.25$ map units. Translation is set to zero if any sample is
outside the raster or occupied; the clipped angular command is still applied
so the robot can turn away. The same guard is applied to the legacy MPC path
when that path uses a unicycle agent. The guard is bypassed exactly when the
map contains no obstacles, preserving prior obstacle-free motion.

Implementations: `src/core/obstacles.py`, `src/coupled_simulation.py`, and the
ground-map and initialization-policy recording in
`evaluation/run_multifidelity_composition.py`.

## Implemented zero-mass posterior-density fallback

Let $M_j$ be the free-query mask, $w_j$ the integration weights,
$\mu^+_{H,j}=\max(\mu_{H,j},0)$, and
$Z=\sum_jM_j\mu^+_{H,j}w_j$. The controller-density input is

$$
v_j=
\begin{cases}
\mu^+_{H,j}, & Z>0,\\
1, & Z=0,
\end{cases}
\qquad
\phi_j=\frac{M_jv_j}{\sum_kM_kv_kw_k}.
$$

Thus a completely nonpositive first posterior publishes a uniform free-space
density instead of failing normalization. The raw posterior mean and variance
are unchanged, so reconstruction metrics still evaluate the actual GP output.
Other update failures continue to preserve the latest valid snapshot.

Implementation: `src/core/multifidelity_estimator.py`.

## Implemented team-composition reconstruction evaluation

The closed-loop composition sweep fixes the total team size $N=N_a+N_g$ and
changes only the number of aerial and ground robots. The publication protocol
uses $N=10$ and aerial counts $N_a\in\{10,2,0\}$. Every condition runs the
same coupled simulation and autoregressive MFGP; no endpoint substitutes a
different estimator. Two named scenarios supply declared mission and ground
geometry parameters:

$$
(K_s,T_s,n_{o,s},r_{o,s})
=
\begin{cases}
(200,20,5,1), & s=\texttt{easy\_long},\\
(100,10,15,2), & s=\texttt{hard\_short},
\end{cases}
$$

where $K_s$ is the number of integration steps, $T_s=K_s\Delta t$ seconds,
$n_{o,s}$ is the number of ground obstacles, $r_{o,s}$ is their radius, and
$\Delta t=0.1$ s. Both geometry and duration change between scenarios, so the
implemented comparison evaluates bundled operating regimes and does not
identify either factor independently.

For composition $c$, episode $e$, and posterior publication $r$, let
$\mathcal J_{c,e}$ be the free query cells. Define the nonnegative evaluated
reconstruction $\mu^+_{H,c,e,r,j}=\max(\mu_{H,c,e,r,j},0)$. The implemented
field error is

$$
\operatorname{NRMSE}_{c,e,r}
=
\frac{
\sqrt{\frac{1}{|\mathcal J_{c,e}|}
\sum_{j\in\mathcal J_{c,e}}
(\mu^+_{H,c,e,r,j}-f_{H,c,e,j})^2}
}{
\max_{j\in\mathcal J_{c,e}}f_{H,c,e,j}
-\min_{j\in\mathcal J_{c,e}}f_{H,c,e,j}
}.
$$

Let $w_j$ be the integration weight and $\phi_{c,e,j}$ the normalized hidden
truth density. The posterior density is clipped to a positive numerical floor,
renormalized with the same weights, and evaluated using

$$
D_{\mathrm{KL},c,e,r}
=
\sum_{j:\phi_{c,e,j}>0}
w_j\phi_{c,e,j}
\log\!\left(\frac{\phi_{c,e,j}}{\widehat\phi_{c,e,r,j}}\right).
$$

For the probabilistic field score, define
$\widetilde\sigma^2_{H,c,e,r,j}=\max(\sigma^2_{H,c,e,r,j},10^{-12})$.
The implemented integration-weighted marginal Gaussian negative log predictive
density is

$$
\operatorname{NLPD}_{c,e,r}
=
\frac{1}{\sum_{j\in\mathcal J_{c,e}}w_j}
\sum_{j\in\mathcal J_{c,e}}w_j
\left[
\frac{1}{2}\log(2\pi\widetilde\sigma^2_{H,c,e,r,j})
+
\frac{(f_{H,c,e,j}-\mu_{H,c,e,r,j})^2}
{2\widetilde\sigma^2_{H,c,e,r,j}}
\right].
$$

NLPD uses the raw latent HIGH posterior mean and variance: it applies neither
nonnegative clipping nor density normalization and does not add sensor noise.
It evaluates independent marginal predictive densities at the query cells, not
their joint cross-query covariance. Lower is better, and well-calibrated narrow
continuous densities can yield negative values.

The nonnegative clipping is applied before NRMSE only; no density normalization
is applied to either field score. The secondary calibration diagnostic
intentionally remains centred on the raw GP mean because it evaluates the
Gaussian posterior:

$$
C_{95,c,e,r}
=
\frac{1}{|\mathcal J_{c,e}|}
\sum_{j\in\mathcal J_{c,e}}
\mathbf 1\!\left[
|f_{H,c,e,j}-\mu_{H,c,e,r,j}|
\le 1.96\sqrt{\sigma^2_{H,c,e,r,j}}
\right].
$$

For any posterior metric $m_r$ published at times
$t_0<\cdots<t_R\le T_s$, the offline evaluator holds the latest value to the
selected scenario horizon $T_s$ and computes the exact implemented trapezoidal
average

$$
\overline m
=
\frac{1}{T_s-t_0}
\left[
\sum_{r=0}^{R-1}
\frac{m_r+m_{r+1}}{2}(t_{r+1}-t_r)
+m_R(T_s-t_R)
\right].
$$

To prevent mixed teams from receiving a larger GP training set merely because
the estimator has two fidelity buffers, one active retention budget $B$ is
split in proportion to composition. For $N_a,N_g>0$, the implemented caps are

$$
B_L=\left\lfloor B\frac{N_a}{N}\right\rfloor,
\qquad
B_H=B-B_L.
$$

At a homogeneous endpoint, the active fidelity receives $B$ and the inactive
buffer receives an unused placeholder cap of one because the retention class
requires a positive capacity. LOW and HIGH sensing use the same event period
and the same samples per robot per event, so the total observation opportunity
per sensing event is constant at $sN$. Submitted LOW/HIGH counts and retained
counts are saved and never feed back into evaluation.

The composition experiment uses online empirical-Bayes kernel fitting. Every
composition and episode starts from the same configured parameter vector
$\boldsymbol\theta_0=(\ell_L,\sigma_L^2,\ell_\delta,\sigma_\delta^2)$ and uses
the same bounds, sample threshold $M_{\min}$, fit interval $K_{\mathrm{fit}}$,
restart count, and iteration limit. For the candidate that would publish
posterior version $r\ge1$, the implemented schedule is

$$
\boldsymbol\theta_{c,e,r}
=
\begin{cases}
\displaystyle\arg\min_{\boldsymbol\theta\in\mathcal B}
\mathcal L_{c,e,r}(\boldsymbol\theta),
& n_{L,c,e,r}+n_{H,c,e,r}\ge M_{\min}
\ \text{and}\ (r-1)\bmod K_{\mathrm{fit}}=0,\\
\boldsymbol\theta_{c,e,r-1}, & \text{otherwise}.
\end{cases}
$$

Thus the optimization policy is controlled across conditions, but the fitted
kernel values are data-dependent and are not constrained to match between
compositions or scenarios. At A10/G0 the discrepancy-kernel parameters are not
identified by LOW data; at A0/G10 the LOW and discrepancy covariance components
are not separately identified from HIGH data alone. These endpoint values are
therefore diagnostics of the bounded optimizer, not independent physical
estimates. Every posterior record saves the fit flag, optimization duration,
and all four realized kernel parameters.

For the ten-robot publication protocol, $B=400$ and the ordered
$(B_L,B_H)$ caps are $(400,1)$, $(80,320)$, and $(1,400)$ for A10/G0, A2/G8,
and A0/G10; the `1` values are inactive placeholders. With $s=10$ samples per
active robot and a shared sensing period of $0.5$ s, every composition submits
100 observations at each sensing event. `easy_long` contains 40 events and
4000 submitted observations per episode; `hard_short` contains 20 events and
2000. Within either scenario, the total opportunity is independent of
composition and is split between fidelities in proportion to team makeup.

Implementations: `evaluation/run_multifidelity_composition.py`,
`evaluation/evaluate_multifidelity_composition.py`,
`evaluation/plot_multifidelity_composition.py`, and
`evaluation/plot_multifidelity_scenario_comparison.py`,
`evaluation/plot_multifidelity_composition_trajectories.py`,
`evaluation/run_multifidelity_scenario_pipeline.py`, and
`evaluation/multifidelity_metrics.py`.

## Implemented footprint-normalized coverage metric

The evaluation pipeline retains the raw hidden-truth mass visible through the
union of the ground robots' instantaneous sensing sectors,

$$
M_{\mathrm{vis}}(t)
=
\int_{\cup_i F_i(t)}\phi(q)\,dq.
$$

For $N_g$ ground robots with range $R$ and field-of-view angle $\theta$ in
radians, the nominal combined footprint-area budget is

$$
A_B
=
\min\left(A_{\mathrm{free}},\;N_g\frac{\theta R^2}{2}\right).
$$

The implemented discrete oracle sorts free query cells by decreasing truth
probability-density value $\phi_j$. It accumulates their integration weights
$w_j$ until area $A_B$ is reached and uses the required fraction of the final
cell. Equivalently, it computes

$$
M_\star(A_B)
=
\max_{0\le z_j\le 1}
\sum_j z_j\phi_jw_j,
\qquad
\sum_j z_jw_j\le A_B,
$$

with obstacle cells fixed to $z_j=0$. The reported footprint-normalized
coverage effectiveness is

$$
E_{\mathrm{cov}}(t)
=
\frac{M_{\mathrm{vis}}(t)}{M_\star(A_B)}.
$$

The denominator is fixed within an episode. Because it relaxes sector shape
and placement while preserving their nominal total area, it is an optimistic
upper bound: $E_{\mathrm{cov}}=1$ means the actual sectors capture the mass in
the densest free-space region of equal area. Actual-sector overlap, boundary
clipping, obstacles, and poor placement reduce only the numerator. The raw
$M_{\mathrm{vis}}$ is saved separately.

For the paired proposed-method/Egerstedt comparison, method $m$ uses its saved
ground states, field-of-view angle, and sensing range. The exact implemented
discrete numerator at saved state time $t_k$ is

$$
M_{\mathrm{vis},k}^{(m,e)}
=
\sum_j w_j\phi_j^{(e)}
\mathbf 1\!\left[q_j\in\bigcup_iF_{i,k}^{(m,e)}\right].
$$

The proposed method uses its saved headings to orient sectors. The Egerstedt
baseline has no heading state and declares a $360$-degree disk, making the
inserted zero headings immaterial. Each method receives its own equal-area
oracle $M_\star(A_B^{(m,e)})$ because the saved footprint geometries can differ.
The evaluator saves the final value and the trapezoidal time average

$$
\overline E_{\mathrm{cov}}^{(m,e)}
=
\frac{1}{t_K-t_0}
\sum_{k=0}^{K-1}
\frac{E_{\mathrm{cov},k}^{(m,e)}
+E_{\mathrm{cov},k+1}^{(m,e)}}{2}
(t_{k+1}-t_k).
$$

Runtime comparison uses the saved total wall-clock duration of each complete
method step. The evaluated archive stores the raw per-step totals and their
per-episode mean, median, empirical 95th percentile, and maximum. Runtime and
coverage are diagnostic outputs only and never feed back into either
controller. Implementations: `evaluation/multifidelity_metrics.py`,
`evaluation/evaluate_multifidelity_ablation.py`, and
`evaluation/evaluate_baseline_comparison.py`.

## Implemented Egerstedt comparison baseline

The separate baseline runner uses the saved obstacle-free scenario and the
control functions in `evaluation/egerstedt.py`. It assumes the hidden truth
$\phi_j$ is known. At each discrete time $k$, aerial robot $a$ is assigned the
range-limited Euclidean Voronoi set $V^A_a$ and moves toward its unweighted
grid centroid $c^A_a$:

$$
p^A_{a,k+1}=p^A_{a,k}+\Delta t\,0.5(c^A_{a,k}-p^A_{a,k}).
$$

Ground robot $i$ similarly computes the truth-density-weighted centroid
$c^G_i$ of its range-limited Voronoi set. Ground robots are also assigned to
their nearest aerial robot. For aerial cell $a$, the implemented allocation
error and threshold are

$$
\sigma_a=\frac{n_a}{N_g}
-\frac{\sum_{j\in V^A_a}\phi_j}{\sum_j\phi_j},
\qquad
\widehat\sigma_a=
\begin{cases}
\sigma_a,&\sigma_a>1/N_g,\\
0,&\text{otherwise}.
\end{cases}
$$

Let $a^- = \arg\min_a\sigma_a$. If ground robot $i$ belongs to aerial cell
$a(i)$, its point position is updated by

$$
p^G_{i,k+1}=p^G_{i,k}+\Delta t\left[
(1-\widehat\sigma_{a(i)})(c^G_{i,k}-p^G_{i,k})
+\widehat\sigma_{a(i)}(c^A_{a^-,k}-p^G_{i,k})
\right].
$$

The runner uses the paired mission timestep $\Delta t$, clips ground positions
to the rectangular domain, and rejects maps containing obstacles. Both robot
classes are first-order point robots with omnidirectional range-limited sensing
in this baseline; no heading state is invented. Its stored locational cost is

$$
H_k=\Delta A\sum_j\phi_j
\left\|q_j-p^G_{\operatorname{Vor}(j),k}\right\|^2.
$$

This cost is a baseline diagnostic, not the common comparison coverage metric.
The baseline has oracle access to $\phi$ and therefore does not produce a
reconstructed density or posterior uncertainty.
