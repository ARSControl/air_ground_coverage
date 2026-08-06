# Experimental Evaluation Plan

## Status and purpose

This document records the agreed plan for completely rewriting Section VI,
"Experimental Evaluation and Discussion," of `paper/updated.tex`. It is a
forward-looking plan, not a report of completed experiments or measured
results.

The evaluation will be organized into exactly three main subsections:

1. Closed-Loop Reconstruction Across Team Compositions
2. Comparison with Baselines
3. Software-in-the-Loop and Computational Performance

The evaluation will not use Wasserstein distance. It will not include a
Mixture-of-Experts baseline because that version is not part of the current
implementation. It will also not include a standalone deterministic paired
causality figure.

## Common metrics

The following metrics cover different outputs of the proposed system and
should not be treated as interchangeable.

### KL divergence of the normalized density

Let `phi` be the normalized ground-truth target density and `phi_hat` the
normalized density obtained from the posterior mean and supplied to the
controllers. Evaluate

$$
D_{\mathrm{KL}}(\phi\|\hat\phi)
=
\int_Q \phi(q)\log\!\left(\frac{\phi(q)}{\hat\phi(q)}\right)dq.
$$

Lower is better. This measures reconstruction of the normalized controller
target. The numerical treatment of zero or near-zero density values must be
fixed and documented before producing results.

### NRMSE of the zero-clipped HIGH-fidelity reconstruction

Define the evaluated nonnegative reconstruction before density normalization as

$$
\mu_H^+(q_j)=\max(\mu_H(q_j),0).
$$

Then evaluate

$$
\operatorname{RMSE}
=
\sqrt{\frac{1}{N}\sum_{j=1}^N
\left(\mu_H^+(q_j)-f_H(q_j)\right)^2},
$$

$$
\operatorname{NRMSE}
=
\frac{\operatorname{RMSE}}
{\max_j f_H(q_j)-\min_j f_H(q_j)}.
$$

Lower is better. This measures the nonnegative HIGH-field reconstruction used
before controller-density normalization; negative GP means contribute as zero
rather than as negative field values. It does not normalize away amplitude
error. A deterministic policy for a zero-range truth field must be specified
in the evaluation implementation.

### Empirical 95% posterior calibration

For latent HIGH-fidelity truth, calculate per episode

$$
C_{95}
=
\frac{1}{N}\sum_{j=1}^N
\mathbf{1}\!\left[
f_H(q_j)\in
[\mu_H(q_j)-1.96\sigma_H(q_j),
 \mu_H(q_j)+1.96\sigma_H(q_j)]
\right].
$$

Calibration remains a diagnostic of the raw Gaussian posterior and therefore
uses the unclipped $\mu_H$ in its interval centre. The ideal value is
approximately `0.95`. Values below `0.95` indicate
overconfidence; values above `0.95` indicate conservative intervals. Report
the raw value `C_95`, not only `abs(C_95 - 0.95)`. Calibration is not applicable
to baselines that do not produce a posterior variance.

Grid points are spatially correlated and must not be treated as independent
experimental repetitions. Calculate one calibration value per episode and
perform statistical summaries across episodes.

### Footprint-normalized ground-truth coverage effectiveness

Use the union of the ground robots' sensing footprints to avoid double-counting
overlap:

$$
M_{\mathrm{cov}}(t)
=
\int_{\bigcup_i F_i(t)}\phi(q)dq,
\qquad 0\le M_{\mathrm{cov}}(t)\le 1.
$$

The raw mass is retained, but the principal physical score normalizes it by
the largest density mass that can fit within the team's nominal total sensing
area. For $N_g$ sector footprints with range $R$ and angle $\theta$ in radians,

$$
A_B=\min\left(A_{\mathrm{free}},N_g\frac{\theta R^2}{2}\right).
$$

Sort free-space cells by decreasing hidden truth-density value and accumulate
their integration areas up to $A_B$, fractionally counting the final cell. Let
the resulting mass be $M_\star(A_B)$. Evaluate

$$
E_{\mathrm{cov}}(t)=\frac{M_{\mathrm{cov}}(t)}{M_\star(A_B)}.
$$

Higher is better and 1 is the densest-area upper bound for the available
nominal sensing area. The numerator uses the actual footprint union, so overlap
and boundary or obstacle clipping remain penalized. Always evaluate this
quantity using the hidden ground-truth density, not the estimate used
internally by a controller. Report the raw mass alongside the normalized score,
and report the score both at the fixed mission horizon and as its time average

$$
\overline E_{\mathrm{cov}}
=
\frac{1}{T}\int_0^T E_{\mathrm{cov}}(t)dt.
$$

This will be the principal physical coverage metric unless a later documented
decision replaces it consistently across all methods.

## Common simulation and statistical protocol

- Use a fixed mission horizon for every compatible setting.
- Use paired scenarios: episode `k` uses the same target field, obstacle map,
  and compatible initial robot states for every method or ablation.
- Separate random-number streams should be used for scenario generation,
  initialization, and sensing so that changing one method does not silently
  change unrelated randomness.
- Use 20 paired episodes per setting as the minimum publication protocol. Use
  30 paired episodes for the main results if computationally affordable. Ten
  episodes are acceptable only for development and pilot runs.
- Record each metric over time. Report both fixed-horizon and time-averaged
  values where applicable.
- Report paired bootstrap 95% confidence intervals and paired method
  differences. Report an effect size; a paired permutation or Wilcoxon test is
  optional and should not replace confidence intervals.
- Use the simulation episode, not individual grid cells or time samples, as the
  independent statistical unit.
- Conclusions must be based on repeated-run statistics. A displayed trajectory
  is illustrative only.

## VI-A. Closed-Loop Reconstruction Across Team Compositions

### Research question and scope

This study asks how the composition of a fixed-size air--ground team affects
closed-loop reconstruction of the latent HIGH-fidelity field. It deliberately
evaluates the complete system: aerial ergodic exploration, ground coverage
control, heterogeneous sensing, and the central autoregressive MFGP all remain
active. It is not an estimator-only replay experiment.

The result therefore supports a system-level claim about the effect of team
composition. It must not be described as independently isolating sensor type,
controller choice, vehicle dynamics, or estimator architecture.

### Team compositions

Fix the total number of robots to ten and evaluate the following three
predeclared configurations, where `A` is the number of aerial robots and `G`
the number of ground robots:

| Label | Aerial robots | Ground robots | Aerial fraction |
| --- | ---: | ---: | ---: |
| A10/G0 | 10 | 0 | 1.00 |
| A2/G8 | 2 | 8 | 0.20 |
| A0/G10 | 0 | 10 | 0.00 |

Use all three configurations in the reported result. A2/G8 is the declared
operational hypothesis of a small aerial scout group supporting a predominantly
HIGH-fidelity ground team; it must not be relabelled as globally optimal. The
earlier broad composition results are pilot evidence, while confirmatory runs
must use fresh predeclared seeds.

The aerial-only and ground-only endpoints are included for reconstruction
metrics. Ground-footprint coverage is not comparable for A10/G0 and is not a
primary outcome of this study.

### Closed-loop behavior

- Aerial robots use the proposed ergodic controller and the current published
  aerial target derived from the cached central posterior.
- Ground robots use the proposed coverage controller and the current published
  ground target density.
- Every condition uses `estimator_mode: multifidelity`, the same configured
  value of `rho`, the same initial kernels and optimization bounds, the same
  noise assumptions, the same update periods, and the same online
  hyperparameter schedule. Realized fitted kernels may differ because each
  composition and episode fits its own retained observations.
- Within a named scenario, mission duration, map distribution, field
  distribution, and all controller parameters unrelated to team size remain
  fixed.

### Declared operational scenarios

The study compares two bundled operating regimes rather than claiming to
separately identify obstacle and duration effects:

| Scenario | Steps | Duration | Ground circles | Radius |
| --- | ---: | ---: | ---: | ---: |
| `easy_long` | 200 | 20 s | 5 | 1.0 |
| `hard_short` | 100 | 10 s | 15 | 2.0 |

The hard scenario combines a shorter opportunity horizon with more and larger
ground obstacles. Aerial motion still overflies those circles. Therefore the
supported claim is explicitly conditional on a short, mobility-constrained
mission; the two-scenario result alone must not attribute an observed
difference uniquely to obstacle count, radius, or duration. A future 2-by-2
easy/hard versus long/short study would be required for that attribution.

### Obstacle protocol

Each named scenario uses the declared non-overlapping circular ground
obstacles. Episode `k` deterministically reuses the same obstacle centres for
all three compositions within that scenario. Circles constrain ground
initialization and trajectories, mask the HIGH field and reconstruction
metrics, and remain traversable by aerial robots flying above them. The
publication runner records the binary ground map in every raw episode and
validates paired hidden truth across compositions. Obstacle count, radius, and
duration must not be tuned after viewing composition outcomes.

The endpoint posterior semantics must remain those of the same autoregressive
model rather than switching to unrelated single-fidelity estimators:

- **A10/G0:** no HIGH observations are available. LOW observations update
  `f_L`; the HIGH posterior is induced by `rho * f_L`, while the unobserved
  discrepancy retains its prior contribution to uncertainty.
- **A0/G10:** no LOW observations are available. HIGH observations update the
  posterior of `f_H` through the autoregressive model's HIGH--HIGH covariance.
  Decomposing that information uniquely between `f_L` and `delta` is not
  required for evaluating the posterior of `f_H`.
- **Mixed teams:** LOW and HIGH observations are admitted normally and both
  affect the published HIGH posterior.

Before publication runs, deterministic tests must verify that the estimator,
scheduler, result writer, and plotting path support both zero-team endpoints.

### Observation and retained-data budget

Fixed headcount must not accidentally give a mixed team a larger GP training
set merely because LOW and HIGH samples have separate retention caps.

- Use a fixed observation opportunity per robot over the mission, unless a
  different modality-specific sampling rate is justified as part of the
  physical sensor model before viewing results.
- Use one fixed total retained-sample budget, or an equivalent fixed per-robot
  retained-sample budget, so the maximum total training-set size is independent
  of composition.
- Do not allow a mixed team to retain the full LOW cap plus the full HIGH cap
  while a homogeneous endpoint can use only one cap.
- Record and report `n_L(t)`, `n_H(t)`, their sum, submitted observation counts,
  and accepted observation counts for every episode.

Exact sample counts, sensing periods, and the retention-allocation rule must be
frozen in the experiment configuration before publication results are viewed.
The frozen ten-robot configuration uses ten observations per robot every
`0.5` s, hence 100 submitted observations per event. This produces 4000
submissions in `easy_long` and 2000 in `hard_short`, independently of
composition. The fixed active retention budget is 400, with LOW/HIGH caps
`(400,1)`, `(80,320)`, and `(1,400)` in A10/G0, A2/G8, and A0/G10 order.
Endpoint caps of one are inactive implementation placeholders and do not
increase the active budget.

### Pairing and initialization

- Use the same predeclared target fields, maps, mission horizon, and sensing
  noise seeds for every composition.
- Use separate deterministic random-number streams for scenario generation,
  robot initialization, and sensing.
- Define one initialization policy that does not favor a particular
  composition. Candidate aerial and ground initial states may be generated
  from fixed class-specific distributions, but the rule for selecting the
  required number of each class must be fixed before viewing results.
- The implemented follow-up protocol uses the lower-left 20% by 20% corner
  rectangle as a common deployment region. Independent class-specific seeded
  permutations make initial states nested across compositions. Because this
  policy was selected after inspecting uniformly initialized results, those
  results are exploratory and fresh confirmatory seeds are required.
- Use at least 20 paired episodes; use 30 for the final figure if computationally
  affordable.

### Primary and secondary outcomes

The primary reconstruction outcome is:

1. time-averaged NRMSE of the zero-clipped HIGH-fidelity reconstruction over
   each scenario's declared horizon.

Secondary reconstruction outcomes are final NRMSE, time-to-threshold if a
threshold is frozen before confirmatory runs, final and time-averaged KL
divergence, final and time-averaged marginal Gaussian NLPD, and empirical 95%
posterior calibration. NLPD uses the saved raw latent HIGH posterior mean and
variance on free cells, integration weights, and a fixed variance floor of
$10^{-12}$; it does not add observation noise. Coverage effectiveness is
reported separately and is not collapsed with reconstruction into an
arbitrarily weighted scalar score.

Retain complete metric histories and final LOW/HIGH sample counts in the raw
result archive. Cross-scenario final values are descriptive because the
missions have different durations; inferential contrasts compare compositions
within the same scenario.

Summaries use the episode as the independent unit. Report episode-bootstrap
95% confidence intervals and paired differences between each mixed team and
both homogeneous endpoints. Do not treat grid cells, posterior publications,
or time samples as independent repetitions.

### Future closed-loop MFGP ablation

The composition sweep alone shows whether the proposed heterogeneous system
outperforms its homogeneous endpoints; it does not establish that explicitly
modeling the fidelity discrepancy causes the improvement. To support the MFGP
claim without adding an estimator-only experiment, a later milestone may add
one closed-loop comparison at a mixed composition:

1. full autoregressive MFGP;
2. the same system with the discrepancy process disabled (`delta = 0`).

All observations, controllers, schedules, budgets, scenarios, and seeds must
remain paired, and the same NRMSE and KL outcomes should be reported. The mixed
composition is now frozen as A2/G8, matching the scout-plus-ground operational
hypothesis. The ablation must use fresh paired confirmatory seeds rather than
the pilot episodes used to motivate that choice.

This ablation is outside the current composition-sweep implementation
milestone. If it is omitted from the paper, the composition sweep must not be
claimed to independently prove superiority over legacy fusion or another
estimator.

### Publication figure and table

The preferred compact figure contains:

1. easy/long NRMSE histories for all three compositions;
2. easy/long KL-divergence histories for all three compositions;
3. easy/long marginal-NLPD histories for all three compositions;
4. hard/short NRMSE histories for all three compositions;
5. hard/short KL-divergence histories for all three compositions;
6. hard/short marginal-NLPD histories for all three compositions.

Every panel uses physical mission time and episode-bootstrap 95% confidence
bands. Like-metric panels share one y-axis scale across scenarios, so vertical
changes are comparable; x-axis ranges remain scenario-specific because mission
horizons differ. Final and
time-averaged values remain in the evaluated archives and publication tables
but are not displayed in this time-series figure.

Representative truth/reconstruction/error maps for A10/G0, A2/G8, and A0/G10
may be placed in supplementary material using one predeclared episode.

If four panels are too dense, move the representative maps to supplementary
material and retain the metric histories and composition summaries. A compact
table should report exact final and time-averaged NRMSE, KL, and NLPD results.
A later ablation result should be added only after its mixed composition has
been chosen and documented.

### Implemented scenario-aware command sequence

Each scenario retains a rectangular raw archive with its own time axis. The
runner selects named overrides from the shared configuration, the unchanged
offline equations evaluate each archive independently, and a read-only
plotter combines the evaluated archives:

```bash
MPLCONFIGDIR=/tmp/ral_marta_mpl \
  .venv/bin/python evaluation/run_multifidelity_scenario_pipeline.py \
  --config configs/multifidelity_composition.yaml \
  --composition A2/G8 --episode 0 \
  --output-dir output/multifidelity_scenario_pipeline
```

After a metric or plotting-only change, reuse the saved simulation archives:

```bash
MPLCONFIGDIR=/tmp/ral_marta_mpl \
  .venv/bin/python evaluation/run_multifidelity_scenario_pipeline.py \
  --config configs/multifidelity_composition.yaml \
  --composition A2/G8 --episode 0 \
  --output-dir output/multifidelity_scenario_pipeline \
  --reuse-raw
```

This is the preferred one-command interface. It runs both declared scenarios,
evaluates each raw archive, renders one aggregate metric figure and one selected
episode trajectory/reconstruction/error figure per scenario, and finally
renders the combined scenario metric figure. The equivalent individual stages
remain available:

```bash
.venv/bin/python evaluation/run_multifidelity_composition.py \
  --output output/composition_raw.npz
.venv/bin/python evaluation/evaluate_multifidelity_composition.py \
  --input output/composition_easy_long_raw.npz \
  --output output/composition_easy_long_evaluated.npz
.venv/bin/python evaluation/evaluate_multifidelity_composition.py \
  --input output/composition_hard_short_raw.npz \
  --output output/composition_hard_short_evaluated.npz
MPLCONFIGDIR=/tmp/ral_marta_mpl \
  .venv/bin/python evaluation/plot_multifidelity_scenario_comparison.py \
  --easy output/composition_easy_long_evaluated.npz \
  --hard output/composition_hard_short_evaluated.npz
```

With named scenarios present and no `--scenario` flag, that first command runs
both declarations in YAML order and writes
`output/composition_easy_long_raw.npz` and
`output/composition_hard_short_raw.npz`; it never writes or overwrites the
unsuffixed requested path. Supplying `--scenario easy_long` or
`--scenario hard_short` runs only that scenario and uses the exact `--output`
path.

The publication configuration is `configs/multifidelity_composition.yaml` and
the deterministic test configuration is
`configs/multifidelity_composition_smoke.yaml`. The raw archive stores padded
team states, identical paired truth within each scenario, scenario name and
resolved parameters, posterior histories, per-step submitted LOW/HIGH counts,
retained counts, phase timings, fit flags, hyperparameter-optimization duration,
and all four realized kernel parameters. The evaluator propagates the optimizer
diagnostics alongside reconstruction metrics and sample/timing diagnostics;
plotting does not import or rerun the simulation. The fixed-hyperparameter
composition guard has been removed: optimization is part of the declared
experiment rather than an unsupported override.

One saved episode can be rendered directly from the raw archive without a
simulation rerun:

```bash
MPLCONFIGDIR=/tmp/ral_marta_mpl \
  .venv/bin/python \
  evaluation/plot_multifidelity_composition_trajectories.py \
  --input output/composition_hard_short_raw.npz \
  --composition A2/G8 --episode 0
```

`--composition` is deliberately required, while `--episode` is an explicit
zero-based archive index and defaults to the predeclared first episode. The
figure contains trajectories over the hidden HIGH field, the standalone ground
truth, the final saved zero-clipped HIGH reconstruction on the same color scale,
and absolute zero-clipped reconstruction error. It overlays the saved ground obstacles and marks
trajectory starts/ends. Any trajectory used in the paper must report its
composition, episode index, and seed in the caption and must be selected by a
predeclared index or another documented rule rather than visual appeal. This
plotting interface does not select a mixed composition for the future MFGP
ablation.

## VI-B. Comparison with Baselines

The intended baselines are:

- proposed full MFGP method;
- the air-ground approach of Rudolph, Wilson, and Egerstedt;
- the air-ground approach of Zhang et al.;
- optionally, a controller supplied with the oracle ground-truth density as a
  reference bound.

The aerial-only and ground-only comparisons belong to VI-A rather than this
baseline subsection.

Each external baseline must be implemented faithfully and first compared in a
common scenario compatible with its published assumptions. Do not silently
extend a baseline to unsupported dynamics, obstacles, or environment geometry.
If a method does not produce a comparable global density or posterior variance,
report the corresponding reconstruction or calibration entry as `N/A`.

### Baseline table

The quantitative baseline table is the primary evidence and should contain:

- fixed-horizon covered probability mass for methods with ground robots;
- time-averaged covered probability mass for methods with ground robots;
- fixed-horizon footprint-normalized coverage effectiveness;
- time-averaged footprint-normalized coverage effectiveness;
- runtime only when implementations and measured scopes are sufficiently
  comparable.

KL, NRMSE, and calibration may be reported only for a baseline that natively
produces the corresponding density or probabilistic HIGH-field estimate. Do
not reconstruct a baseline output after the fact merely to fill the table.

Report repeated-run summaries and 95% confidence intervals.

### Baseline trajectory figure

Use one panel each for:

1. proposed MFGP;
2. Rudolph--Wilson--Egerstedt;
3. Zhang et al.;
4. optionally, the oracle-density reference if it is included in the study.

Use a one-by-three layout without the optional oracle or a two-by-two layout
with it. The ground-only condition is already represented in the VI-A
composition study and need not consume another baseline panel.

Every panel must use the same illustrative scenario and show:

- the same ground-truth density as the background;
- the same obstacles, axes, aspect ratio, and density color scale;
- ground trajectories as solid lines;
- aerial trajectories as thinner dashed lines;
- initial positions as crosses;
- final positions as filled markers;
- final ground sensing footprints or FOV sectors;
- the final covered-mass value as a small annotation.

Do not use each method's estimated density as its background. Use the first
predeclared evaluation seed, or another selection rule declared before viewing
comparative results. State in the caption that the trajectories are
illustrative and that the table contains the repeated-run evidence.

## VI-C. Software-in-the-Loop and Computational Performance

Any SITL result used in the revised paper must run the actual central
autoregressive MFGP and current controller-feedback paths. Results from a
different estimator or fusion architecture do not validate the revised
method.

### SITL/runtime figure

The preferred figure contains:

1. **Top-down SITL result:** ground-truth or clearly identified estimated
   density, obstacles, aerial and ground trajectories, initial/final positions,
   and final ground sensing footprints.
2. **Estimator latency:** every central GP update duration against simulation
   time or retained sample count, with a horizontal line marking the configured
   GP update period. Distinguish updates that perform hyperparameter fitting.
3. **Optional runtime breakdown:** distributions for observation processing,
   hyperparameter optimization, GP factorization/update, posterior prediction,
   aerial control, and ground control. This panel may be replaced by a compact
   runtime table if space is limited.

Report at least median, 95th percentile, and maximum latency; the fraction of
GP updates meeting their deadline; the full-loop real-time factor; retained
LOW/HIGH sample counts; and the hardware/software configuration. Average
runtime alone is insufficient.

## Optional qualitative MFGP figure

If page space remains, an additional four-panel figure may show:

1. LOW-fidelity truth with aerial samples;
2. HIGH-fidelity truth with ground samples;
3. final HIGH posterior mean;
4. final HIGH posterior standard deviation.

Use a predeclared seed, preferably the first evaluation seed. This is the first
figure to omit when space is constrained because the three main result figures
and their tables carry the core evidence.

## Plotting and reporting conventions

- Use a consistent, colorblind-safe method color throughout the section.
- Use identical limits and color normalization for directly comparable panels.
- Label whether higher or lower is better.
- Do not use bar charts when episode-level distributions are available.
- Do not use a trajectory example as evidence of statistical superiority.
- Keep plotting code standalone from production estimator, controller, robot,
  and simulation-loop code.
- Save deterministic scripts and generated artifacts under the repository's
  documented evaluation locations, with paper-ready figures copied or linked
  into `paper/pics/` only after validation.

## Execution checklist

- [ ] Freeze exact definitions and numerical edge-case policies for all four
      metrics.
- [ ] Implement and test metric collection independently of controller inputs.
- [x] Add and test support for the three declared A/G compositions, including both
      zero-team endpoints.
- [ ] Freeze the composition-independent observation and retained-data budget.
- [x] Freeze A2/G8 as the operationally motivated mixed composition for a
      future no-discrepancy ablation, with fresh confirmatory seeds required.
- [x] Implement named easy/long and hard/short scenario selection and a paired
      evaluated-archive comparison plot.
- [ ] Verify faithful implementations and assumption-compatible scenarios for
      both literature baselines.
- [ ] Freeze evaluation seeds, scenario generation, mission horizon, team
      budgets, and initial-state policy before viewing comparative results.
- [ ] Run pilot episodes and inspect failures before launching publication runs.
- [ ] Run at least 20 paired publication episodes per setting, preferably 30
      for the main comparisons.
- [ ] Generate the composition figure and table from saved raw results.
- [ ] Generate the baseline table and fixed-seed trajectory figure from saved
      raw results.
- [ ] Run the current central MFGP in the SITL/feasibility setup and collect
      component-level timing data.
- [ ] Generate the SITL/runtime figure and timing table.
- [ ] Rewrite Section VI only after all reported numbers and figures have been
      reproduced from saved experiment outputs.
- [ ] Reconcile the abstract, contribution list, overview figure, method text,
      conclusion, and SITL description with the final MFGP implementation.
