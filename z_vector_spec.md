# Z Vector Interface Specification

## Purpose
The Z vector encodes human occupancy/risk inferred from WiFi CSI and feeds the exogenous term
\(\Lambda(t)\) in the ant-kernel update:

\[
W(t+1) = (1-\rho)W(t) + \Delta_w M(t+1) + \Lambda(t)
\]

In this implementation, \(\Lambda(t)\) is applied as a multiplicative risk penalty on edges around
human-associated vertices.

## Field specification
Each Z vector is a Python dictionary with the following fields:

- `bearing: float`
  - Units: degrees
  - Range: `[0, 360)`
  - Meaning: direction from robot/grid center toward detected human.
- `distance: float`
  - Units: meters
  - Range: `>= 0`
  - Meaning: radial distance from robot/grid center.
- `activity_class: str`
  - One of: `stationary`, `walking`, `lifting`, `running`
  - Meaning: semantic class for logging/policy extensions.
- `niosh_risk: float`
  - Range: `[0.0, 1.0]`
  - Meaning: ergonomic/safety risk intensity.
- `confidence: float`
  - Range: `[0.0, 1.0]`
  - Meaning: confidence in the CSI estimate.

## Bearing-to-grid coordinate mapping
The grid origin is the center of the graph, and polar coordinates are mapped to nearest node:

- `0°` -> east (`+x`)
- `90°` -> north (`-y` in grid row indexing)
- `180°` -> west (`-x`)
- `270°` -> south (`+y`)

Distance in meters is converted to cells via `distance / cell_size_m`, then rounded and clipped to
valid graph bounds.

## Risk penalty mapping
For each active Z vector, all edges adjacent to the mapped target vertex are down-weighted:

\[
W_{edge} \leftarrow \max(\varepsilon, W_{edge} \cdot (1 - niosh\_risk \cdot confidence_{eff}))
\]

Where:
- `epsilon` is the strict positive minimum edge weight.
- `confidence_eff` is confidence modulated by temporal decay.

Interpretation:
- higher `niosh_risk` => stronger avoidance
- higher `confidence` => stronger immediate penalty
- lower confidence => gentler influence

## Multiple simultaneous Z vectors
Multiple Z vectors may be active at once. Their effects are composed sequentially per step.
If two vectors affect disjoint regions, each region is penalized independently. If they overlap,
penalties multiply (stronger aggregate suppression).

## Temporal decay when humans move away
Each injected Z vector stores a current influence strength:

\[
strength_0 = niosh\_risk \cdot confidence
\]

Per kernel step:

\[
strength_{t+1} = strength_t \cdot z\_decay\_rate
\]

When strength becomes negligible, the Z vector is removed from the active set.
This gives a fading-memory exogenous term without abrupt discontinuities.

## Example Z vectors

### Stationary
```python
{
  "bearing": 45.0,
  "distance": 1.8,
  "activity_class": "stationary",
  "niosh_risk": 0.20,
  "confidence": 0.92,
}
```

### Walking
```python
{
  "bearing": 120.0,
  "distance": 2.4,
  "activity_class": "walking",
  "niosh_risk": 0.45,
  "confidence": 0.88,
}
```

### Lifting
```python
{
  "bearing": 210.0,
  "distance": 1.2,
  "activity_class": "lifting",
  "niosh_risk": 0.80,
  "confidence": 0.95,
}
```

### Running
```python
{
  "bearing": 300.0,
  "distance": 3.0,
  "activity_class": "running",
  "niosh_risk": 0.95,
  "confidence": 0.90,
}
```

## Link to convergence proof (Jarne & Mazo corollary case)
The prototype enforces Theorem 2 structural assumptions by keeping:
- `epsilon > 0` for every edge at all times
- graph topology connected and unchanged

The added \(\Lambda(t)\) term is exogenous (comes from CSI sensing, not from `M(t)` itself),
so it does not create direct feedback dependence on individual sampled moves. Under bounded,
decaying perturbations, this aligns with the corollary-style robustness argument where the
underlying ant update remains a stable stochastic process around a stationary distribution.
