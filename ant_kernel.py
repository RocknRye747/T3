"""Ant colony navigation kernel with exogenous human-risk injection.

This module implements a prototype of the update law
W(t+1) = (1-rho)W(t) + Delta_w M(t+1) + Lambda(t)
from Jarne & Mazo (HSCC 2020), extended with a CSI-derived Z vector term.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import networkx as nx
import numpy as np

Node = Tuple[int, int]


@dataclass
class ZInfluence:
    """Represents one active exogenous occupancy/risk influence.

    Attributes:
        bearing: Direction to detected human in degrees [0, 360).
        distance: Estimated distance to the human in meters.
        activity_class: Semantic activity label from CSI post-processing.
        niosh_risk: Risk level in [0.0, 1.0], where 1.0 is max risk.
        confidence: Confidence in [0.0, 1.0] for this detection.
        strength: Current multiplicative penalty strength in [0.0, 1.0].
        target_node: Graph node affected by this influence.
    """

    bearing: float
    distance: float
    activity_class: str
    niosh_risk: float
    confidence: float
    strength: float
    target_node: Node


class AntKernel:
    """Bio-inspired ant navigation over a weighted 2D grid graph.

    The edge-weight update follows the paper's structure:
    - evaporation term: (1-rho)W(t)
    - reinforcement term: Delta_w * M(t+1)
    - exogenous term: Lambda(t), implemented by decaying Z-vector penalties

    Args:
        grid_size: (width, height) room discretization.
        initial_weight: Uniform starting edge weight.
        evaporation_rate: rho in Eq. 5 (Jarne & Mazo 2020).
        reinforcement: Delta_w amount added to traversed edges.
        min_weight: epsilon lower bound to satisfy Theorem 2 condition epsilon > 0.
        z_decay_rate: Per-step decay factor for active Z-vector influence.
        cell_size_m: Physical meters represented by one grid cell.
        rng_seed: Optional random seed for deterministic testing.
        convergence_window: Number of recent distributions to compare.
        convergence_tol: L1-distance threshold for stabilization.
    """

    def __init__(
        self,
        grid_size: Tuple[int, int] = (10, 10),
        initial_weight: float = 1.0,
        evaporation_rate: float = 0.05,
        reinforcement: float = 0.2,
        min_weight: float = 1e-3,
        z_decay_rate: float = 0.90,
        cell_size_m: float = 1.0,
        rng_seed: Optional[int] = None,
        convergence_window: int = 50,
        convergence_tol: float = 0.01,
    ) -> None:
        self.width, self.height = grid_size
        self.evaporation_rate = evaporation_rate
        self.reinforcement = reinforcement
        self.min_weight = min_weight
        self.z_decay_rate = z_decay_rate
        self.cell_size_m = cell_size_m
        self.convergence_window = convergence_window
        self.convergence_tol = convergence_tol

        self.graph: nx.Graph = nx.grid_2d_graph(self.width, self.height)
        for u, v in self.graph.edges:
            self.graph.edges[u, v]["weight"] = float(initial_weight)

        self.active_z_vectors: List[ZInfluence] = []
        self.rng = np.random.default_rng(rng_seed)

        self.visit_counts: Dict[Node, int] = {node: 0 for node in self.graph.nodes}
        self._distribution_history: List[np.ndarray] = []
        self._node_index: Dict[Node, int] = {
            node: i for i, node in enumerate(sorted(self.graph.nodes))
        }

        if self.min_weight <= 0:
            raise ValueError("min_weight must be > 0 to preserve convergence conditions")
        if not nx.is_connected(self.graph):
            raise ValueError("Graph must be connected for convergence guarantees")

    def inject_z_vector(self, z_vector: Dict[str, object]) -> Node:
        """Inject one CSI-derived Z vector as the exogenous Lambda(t) term.

        The Z vector is converted to a nearest grid node using bearing+distance,
        then stored as an active influence. During each step, adjacent edges receive
        multiplicative penalty:
            W(edge) *= (1 - niosh_risk * confidence_effective)

        where confidence_effective decays over time with z_decay_rate.

        Args:
            z_vector: Dict with keys bearing, distance, activity_class,
                niosh_risk, confidence.

        Returns:
            The grid node targeted by this Z-vector influence.
        """

        bearing = float(z_vector["bearing"])
        distance = float(z_vector["distance"])
        activity_class = str(z_vector["activity_class"])
        niosh_risk = float(np.clip(float(z_vector["niosh_risk"]), 0.0, 1.0))
        confidence = float(np.clip(float(z_vector["confidence"]), 0.0, 1.0))

        target_node = self._bearing_distance_to_node(bearing, distance)
        influence = ZInfluence(
            bearing=bearing,
            distance=distance,
            activity_class=activity_class,
            niosh_risk=niosh_risk,
            confidence=confidence,
            strength=niosh_risk * confidence,
            target_node=target_node,
        )
        self.active_z_vectors.append(influence)
        return target_node

    def step(self, agent_position: Node) -> Node:
        """Advance dynamics by one time-step and sample the next node.

        This performs Eq. 5 style update in plain-English stages:
        1) Evaporate all edge weights by factor (1-rho).
        2) Apply exogenous decaying Z penalties (Lambda term).
        3) Sample transition probabilities proportional to adjacent edge weights.
        4) Reinforce traversed edge by Delta_w (movement matrix term M(t+1)).

        Args:
            agent_position: Current node position.

        Returns:
            The sampled adjacent node for the agent's next position.
        """

        if agent_position not in self.graph:
            raise ValueError("agent_position must exist in graph")

        # Eq. 5 from Jarne & Mazo 2020: evaporation component (1-rho)W(t)
        self._apply_evaporation()

        # Eq. 5 extension: Lambda(t) from external human occupancy/risk sensing.
        self._apply_z_influences()

        neighbors = list(self.graph.neighbors(agent_position))
        weights = np.array(
            [self.graph.edges[agent_position, n]["weight"] for n in neighbors],
            dtype=float,
        )
        probs = weights / weights.sum()
        idx = int(self.rng.choice(len(neighbors), p=probs))
        next_position = neighbors[idx]

        # Eq. 5 movement/reinforcement component Delta_w * M(t+1).
        updated = self.graph.edges[agent_position, next_position]["weight"] + self.reinforcement
        self.graph.edges[agent_position, next_position]["weight"] = max(updated, self.min_weight)

        self.visit_counts[next_position] += 1
        self._update_distribution_history()
        return next_position

    def stationary_distribution(self) -> Dict[Node, float]:
        """Return empirical stationary distribution estimated from visit counts."""

        total = sum(self.visit_counts.values())
        if total == 0:
            uniform = 1.0 / self.graph.number_of_nodes()
            return {node: uniform for node in self.graph.nodes}
        return {node: self.visit_counts[node] / total for node in self.graph.nodes}

    def has_converged(self) -> bool:
        """Return True if recent empirical distributions are within tolerance."""

        if len(self._distribution_history) < self.convergence_window:
            return False
        window = self._distribution_history[-self.convergence_window :]
        baseline = window[0]
        distances = [float(np.linalg.norm(d - baseline, ord=1)) for d in window[1:]]
        return max(distances, default=np.inf) < self.convergence_tol

    def is_connected(self) -> bool:
        """Return connectivity status (must stay True per Theorem 2 assumptions)."""

        return nx.is_connected(self.graph)

    def _apply_evaporation(self) -> None:
        """Decay all edges by (1-rho), clamped to epsilon > 0.

        Keeping epsilon strictly positive prevents any edge from disappearing,
        preserving graph connectivity assumptions used in Theorem 2.
        """

        factor = 1.0 - self.evaporation_rate
        for u, v in self.graph.edges:
            decayed = self.graph.edges[u, v]["weight"] * factor
            self.graph.edges[u, v]["weight"] = max(decayed, self.min_weight)

    def _apply_z_influences(self) -> None:
        """Apply and decay all active Z influences.

        Each influence penalizes edges adjacent to its mapped target node using
        the multiplicative law W *= (1 - strength). Strength decays each step,
        and negligible influences are removed.
        """

        retained: List[ZInfluence] = []
        for influence in self.active_z_vectors:
            penalty_factor = 1.0 - float(np.clip(influence.strength, 0.0, 1.0))
            for nbr in self.graph.neighbors(influence.target_node):
                w = self.graph.edges[influence.target_node, nbr]["weight"]
                self.graph.edges[influence.target_node, nbr]["weight"] = max(
                    w * penalty_factor,
                    self.min_weight,
                )

            influence.strength *= self.z_decay_rate
            if influence.strength > 1e-4:
                retained.append(influence)

        self.active_z_vectors = retained

    def _bearing_distance_to_node(self, bearing_deg: float, distance_m: float) -> Node:
        """Map polar bearing/distance to nearest grid node.

        Mapping convention:
        - 0 deg points east (+x)
        - 90 deg points north (-y in grid row coordinates)
        - 180 deg points west (-x)
        - 270 deg points south (+y)

        The origin is the center of the grid.
        """

        center_x = (self.width - 1) / 2.0
        center_y = (self.height - 1) / 2.0

        theta = np.deg2rad(bearing_deg % 360.0)
        radius_cells = max(distance_m / self.cell_size_m, 0.0)

        x = center_x + radius_cells * np.cos(theta)
        y = center_y - radius_cells * np.sin(theta)

        i = int(np.clip(int(round(x)), 0, self.width - 1))
        j = int(np.clip(int(round(y)), 0, self.height - 1))
        return (i, j)

    def _update_distribution_history(self) -> None:
        """Store latest empirical distribution vector for convergence checks."""

        distribution = self.stationary_distribution()
        vec = np.zeros(self.graph.number_of_nodes(), dtype=float)
        for node, p in distribution.items():
            vec[self._node_index[node]] = p
        self._distribution_history.append(vec)
        if len(self._distribution_history) > self.convergence_window:
            self._distribution_history.pop(0)
