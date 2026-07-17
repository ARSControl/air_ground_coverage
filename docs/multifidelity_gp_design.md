# Central Asynchronous Multi-Fidelity GP Design

## 1. Goal

Replace the current post-hoc fusion of separate aerial and ground GP estimates with a joint probabilistic model.

The revised system must allow high-fidelity observations collected by ground robots to influence:

1. the estimated high-fidelity environmental field;
2. its posterior uncertainty;
3. the density used by the ground coverage controller;
4. the dynamic target used by aerial ergodic or HEDAC exploration.

The implementation remains a deterministic Python simulation.

## 2. Statistical Model

Let the low-fidelity aerial latent process be:

    f_L(q) ~ GP(m_L(q), k_L(q, q'))

Let the discrepancy process be:

    delta(q) ~ GP(m_delta(q), k_delta(q, q'))

Assume `f_L` and `delta` are independent. Define the high-fidelity field as:

    f_H(q) = rho * f_L(q) + delta(q)

Aerial observations are:

    y_L = f_L(q_L) + epsilon_L

Ground observations are:

    y_H = f_H(q_H) + epsilon_H

Use separate, configurable observation-noise variances.

For low-fidelity positions `X_L` and high-fidelity positions `X_H`, use:

    K_LL = K_L(X_L, X_L) + Sigma_L

    K_LH = rho * K_L(X_L, X_H)

    K_HL = rho * K_L(X_H, X_L)

    K_HH = rho^2 * K_L(X_H, X_H)
           + K_delta(X_H, X_H)
           + Sigma_H

The estimator must predict the posterior mean and marginal variance of `f_H`.

Use Cholesky factorization and triangular solves. Add configurable diagonal jitter. Do not explicitly invert covariance matrices.

## 3. Meaning of Fidelity

The revised model is justified when aerial measurements are not merely noisier, but represent a systematically coarser view of the field.

Examples include:

- larger spatial footprint;
- lower spatial resolution;
- altitude-dependent smoothing;
- inability to resolve narrow modes;
- systematic scale or bias differences.

If the current simulator models only different additive noise on otherwise identical point measurements, document this limitation. Do not hide it. Consider extending the aerial sensor simulation so that low fidelity includes spatial averaging or smoothing.

## 4. Central Asynchronous Estimator

The central estimator is a normal Python object owned by the simulation.

“Asynchronous” means:

- aerial and ground sensors may produce data at different simulated times;
- observations are submitted independently;
- observations are buffered;
- the GP is updated at a configurable period;
- the controller integration step is independent of the GP update period;
- controllers use the latest cached posterior between GP updates.

No real networking or concurrency is required.

Conceptual simulation flow:

    advance robot states
    collect aerial observations when aerial sensor period expires
    collect ground observations when ground sensor period expires
    submit observations to estimator
    update estimator when GP update period expires
    predict and cache a new posterior
    update controller density inputs
    continue control using cached posterior

## 5. Observation Information

Each observation should contain:

- simulation timestamp;
- robot identifier;
- spatial position;
- measured scalar value;
- fidelity level: LOW or HIGH;
- observation-noise variance.

Reuse existing repository data structures when suitable.

The estimator must handle:

- different sensor periods;
- delayed observations;
- out-of-order timestamps;
- intervals with no new data;
- invalid values;
- duplicate or nearly coincident observations.

## 6. Bounded Dataset

Maintain separate limits for low- and high-fidelity observations.

The first implementation should use a simple deterministic retention policy, such as:

- minimum spatial separation;
- maximum sample count;
- removal of old or spatially redundant samples.

Keep retention logic behind a separate interface so it can later be replaced by:

- information-gain selection;
- discrepancy-residual selection;
- inducing-point methods;
- sparse variational inference.

Do not add sparse variational inference in the first implementation unless the repository already contains appropriate infrastructure.

## 7. Posterior Representation

The cached posterior should contain:

- simulation timestamp;
- high-fidelity posterior mean;
- high-fidelity posterior variance;
- normalized nonnegative density;
- estimator version or update counter;
- query-grid information;
- number of retained low-fidelity observations;
- number of retained high-fidelity observations;
- update status or validity.

If an update fails, preserve the previous valid posterior.

## 8. Probability-Density Conversion

A standard GP mean is neither guaranteed to be nonnegative nor normalized.

For controller use:

1. apply a nonnegative transformation to the high-fidelity posterior mean (the implemented user-approved choice is positive-part clipping, `max(mean, 0)`, superseding the original softplus suggestion);
2. numerically normalize the result over the simulation domain.

Keep the original posterior mean and variance separate from the normalized density.

The numerical integral of the density must equal one within a configured tolerance.

## 9. Aerial Feedback

The aerial controller must use a target derived from the joint high-fidelity posterior, not from aerial observations alone.

Construct:

    aerial_target(q) = normalize(
        lambda_interest * density_H(q)
        + lambda_uncertainty * normalized_std_H(q)
    )

where:

    normalized_std_H(q) =
        std_H(q) / max_q std_H(q)

Both weights must be configurable.

Interpretation:

- `lambda_interest` promotes monitoring likely regions of interest;
- `lambda_uncertainty` promotes exploration where the high-fidelity field remains uncertain.

Before the first valid posterior is available, preserve the current initial or static target.

Between GP updates, continue using the latest valid aerial target.

## 10. Ground Controller

Use the normalized high-fidelity density as the density input of the existing ground coverage or MPC controller.

Initially preserve the existing objective and constraints. Replace only the source of the density map.

A future information-gain term may be prepared behind a disabled configuration option, but it must not expand the scope of the first implementation.

## 11. Configuration

Expose through the existing configuration mechanism:

- estimator mode;
- GP update period;
- aerial sensor period;
- ground sensor period;
- `rho`;
- low-fidelity kernel parameters;
- discrepancy-kernel parameters;
- low-fidelity noise variance;
- high-fidelity noise variance;
- numerical jitter;
- maximum low-fidelity samples;
- maximum high-fidelity samples;
- sample-retention settings;
- `lambda_interest`;
- `lambda_uncertainty`;
- density-normalization tolerance.

Do not hard-code simulation bounds, grid resolution, update periods, sample limits, kernel parameters, or noise levels.

## 12. Required Tests

### Mathematical tests

Use a deterministic synthetic field where:

- `f_L` contains broad smooth modes;
- `delta` contains narrow local corrections;
- `f_H = rho * f_L + delta`.

Verify:

1. low-fidelity observations recover broad structure;
2. high-fidelity observations introduce local corrections;
3. posterior variance decreases near observations;
4. predictions match a direct joint-covariance reference on a small dataset;
5. predicted variances are finite and nonnegative;
6. duplicate points remain numerically stable with jitter.

### Estimator tests

Verify:

1. low-only operation;
2. high observations arriving later;
3. out-of-order observations;
4. delayed observations;
5. periods with no new data;
6. dataset limits;
7. nonnegative normalized density;
8. density integral equal to one;
9. failed updates do not replace the latest valid posterior.

### Closed-loop tests

Verify:

1. a ground observation changes the high-fidelity posterior;
2. the changed posterior changes the aerial target;
3. the changed target changes an aerial command or trajectory;
4. the ground controller uses the same updated high-fidelity density.

## 13. Evaluation Outputs

The revised experiment should record:

- high-fidelity posterior mean over time;
- high-fidelity posterior variance over time;
- normalized density used by ground robots;
- target density used by aerial robots;
- low- and high-fidelity sample counts;
- GP update time;
- prediction time;
- integrated posterior uncertainty;
- field-reconstruction metrics;
- aerial and ground trajectories.

Keep plotting and experiment logging outside the core GP implementation.
