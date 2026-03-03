import networkx as nx
import numpy as np

from ant_kernel import AntKernel


def edge_weights(kernel: AntKernel) -> np.ndarray:
    return np.array([kernel.graph.edges[e]["weight"] for e in kernel.graph.edges], dtype=float)


def test_graph_initializes_with_uniform_weights() -> None:
    kernel = AntKernel(grid_size=(4, 4), initial_weight=2.5)
    weights = edge_weights(kernel)
    assert np.allclose(weights, 2.5)


def test_evaporation_reduces_weights_correctly_over_time() -> None:
    kernel = AntKernel(grid_size=(3, 3), initial_weight=1.0, evaporation_rate=0.1, reinforcement=0.0)
    kernel.step((1, 1))
    weights = edge_weights(kernel)
    assert np.allclose(weights, 0.9)


def test_minimum_weight_prevents_any_edge_reaching_zero() -> None:
    kernel = AntKernel(
        grid_size=(3, 3),
        initial_weight=0.01,
        evaporation_rate=0.9,
        reinforcement=0.0,
        min_weight=1e-3,
    )
    for _ in range(20):
        kernel.step((1, 1))
    assert np.all(edge_weights(kernel) >= kernel.min_weight)


def test_z_vector_injection_reduces_weights_near_human_position() -> None:
    kernel = AntKernel(grid_size=(7, 7), evaporation_rate=0.0, reinforcement=0.0)
    target = kernel.inject_z_vector(
        {
            "bearing": 0,
            "distance": 2,
            "activity_class": "walking",
            "niosh_risk": 0.5,
            "confidence": 1.0,
        }
    )
    before = [kernel.graph.edges[target, n]["weight"] for n in kernel.graph.neighbors(target)]
    kernel.step((3, 3))
    after = [kernel.graph.edges[target, n]["weight"] for n in kernel.graph.neighbors(target)]
    assert max(after) < min(before)


def test_high_niosh_risk_causes_strong_avoidance() -> None:
    low = AntKernel(grid_size=(7, 7), evaporation_rate=0.0, reinforcement=0.0)
    high = AntKernel(grid_size=(7, 7), evaporation_rate=0.0, reinforcement=0.0)

    z_base = {"bearing": 0, "distance": 2, "activity_class": "running", "confidence": 1.0}
    low_target = low.inject_z_vector({**z_base, "niosh_risk": 0.1})
    high_target = high.inject_z_vector({**z_base, "niosh_risk": 0.9})

    low.step((3, 3))
    high.step((3, 3))

    low_weight = np.mean([low.graph.edges[low_target, n]["weight"] for n in low.graph.neighbors(low_target)])
    high_weight = np.mean([high.graph.edges[high_target, n]["weight"] for n in high.graph.neighbors(high_target)])
    assert high_weight < low_weight


def test_low_confidence_z_vector_has_minimal_effect() -> None:
    kernel = AntKernel(grid_size=(7, 7), evaporation_rate=0.0, reinforcement=0.0)
    target = kernel.inject_z_vector(
        {
            "bearing": 90,
            "distance": 2,
            "activity_class": "stationary",
            "niosh_risk": 1.0,
            "confidence": 0.05,
        }
    )
    kernel.step((3, 3))
    affected = [kernel.graph.edges[target, n]["weight"] for n in kernel.graph.neighbors(target)]
    assert min(affected) > 0.9


def test_agent_step_returns_valid_adjacent_node() -> None:
    kernel = AntKernel(grid_size=(5, 5), rng_seed=1)
    pos = (2, 2)
    nxt = kernel.step(pos)
    assert nxt in set(kernel.graph.neighbors(pos))


def test_agent_never_steps_to_non_adjacent_node() -> None:
    kernel = AntKernel(grid_size=(5, 5), rng_seed=4)
    pos = (2, 2)
    for _ in range(100):
        nxt = kernel.step(pos)
        assert kernel.graph.has_edge(pos, nxt)
        pos = nxt


def test_multiple_z_vectors_handled_correctly() -> None:
    kernel = AntKernel(grid_size=(9, 9), evaporation_rate=0.0, reinforcement=0.0)
    t1 = kernel.inject_z_vector(
        {
            "bearing": 0,
            "distance": 2,
            "activity_class": "walking",
            "niosh_risk": 0.7,
            "confidence": 0.9,
        }
    )
    t2 = kernel.inject_z_vector(
        {
            "bearing": 180,
            "distance": 2,
            "activity_class": "lifting",
            "niosh_risk": 0.6,
            "confidence": 0.8,
        }
    )
    kernel.step((4, 4))
    a1 = [kernel.graph.edges[t1, n]["weight"] for n in kernel.graph.neighbors(t1)]
    a2 = [kernel.graph.edges[t2, n]["weight"] for n in kernel.graph.neighbors(t2)]
    assert max(a1) < 1.0
    assert max(a2) < 1.0


def test_distribution_convergence_over_1000_steps() -> None:
    kernel = AntKernel(grid_size=(6, 6), rng_seed=2, convergence_window=40, convergence_tol=0.2)
    pos = (3, 3)
    for _ in range(1000):
        pos = kernel.step(pos)
    dist = kernel.stationary_distribution()
    assert abs(sum(dist.values()) - 1.0) < 1e-9
    assert kernel.has_converged()


def test_theorem2_condition_epsilon_positive_maintains_connectivity() -> None:
    kernel = AntKernel(grid_size=(5, 5), min_weight=1e-4, evaporation_rate=0.5)
    pos = (2, 2)
    for _ in range(200):
        pos = kernel.step(pos)
    assert kernel.min_weight > 0
    assert kernel.is_connected()
    assert nx.is_connected(kernel.graph)


def test_z_vector_bearing_0_maps_to_east_region() -> None:
    kernel = AntKernel(grid_size=(9, 9), cell_size_m=1.0)
    target = kernel.inject_z_vector(
        {
            "bearing": 0,
            "distance": 3,
            "activity_class": "walking",
            "niosh_risk": 0.2,
            "confidence": 0.8,
        }
    )
    center_x = (kernel.width - 1) / 2
    assert target[0] > center_x


def test_z_vector_bearing_180_maps_to_west_region() -> None:
    kernel = AntKernel(grid_size=(9, 9), cell_size_m=1.0)
    target = kernel.inject_z_vector(
        {
            "bearing": 180,
            "distance": 3,
            "activity_class": "walking",
            "niosh_risk": 0.2,
            "confidence": 0.8,
        }
    )
    center_x = (kernel.width - 1) / 2
    assert target[0] < center_x
