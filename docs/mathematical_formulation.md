# Mathematical formulation

The mathematical reference is maintained as LaTeX source at
[`docs/latex/mathematical_formulation.tex`](latex/mathematical_formulation.tex).

The current rendered version is published by GitHub Actions on GitHub Pages:

[Read the mathematical formulation (PDF)](https://arscontrol.github.io/air_ground_coverage/mathematical_formulation.pdf)

Pull requests that change the LaTeX source attach the compiled PDF as a workflow
artifact. The PDF is generated from source and is intentionally not committed to
the repository.

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
$M_{\mathrm{vis}}$ is saved separately. Implementation:
`evaluation/multifidelity_metrics.py`; archive integration:
`evaluation/evaluate_multifidelity_ablation.py`.

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
