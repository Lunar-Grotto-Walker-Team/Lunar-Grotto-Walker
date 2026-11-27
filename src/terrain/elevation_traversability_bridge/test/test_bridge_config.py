"""Regression tests for the bridge's topic configuration.

The bug these tests pin down: the bridge subscribed ``/elevation_map``
(elevation_mapping's *fused* map -- it only carries ``elevation`` plus the
confidence bounds ``upper_bound`` / ``lower_bound`` / ``uncertainty_range``)
while demanding the terrain-analysis layers ``slope`` and ``roughness``.
Those layers are produced by the postprocessing filter chain and are
published on ``/elevation_map_raw_post`` instead.  Every single frame was
therefore dropped with::

    process_gridmap failed ...: missing required layer 'slope'
    (required: ['slope', 'roughness'],
     have: ['elevation', 'upper_bound', 'lower_bound', 'color', 'uncertainty_range'])

and ``/terrain_traversability`` was never published, so the Nav2
``terrain_traversability_layer`` stayed empty.

The tests also make sure the packaged yaml and the node defaults cannot
drift apart again (the yaml is what ``bridge.launch.py`` actually loads).
"""
from pathlib import Path

import yaml

from elevation_traversability_bridge.bridge_config import (
    DEFAULT_INPUT_TOPIC,
    DEFAULT_OUTPUT_TOPIC,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
PARAMS_YAML = REPO_ROOT / "config" / "bridge_params.yaml"


def _yaml_params() -> dict:
    with PARAMS_YAML.open() as handle:
        doc = yaml.safe_load(handle)
    return doc["terrain_traversability_bridge"]["ros__parameters"]


def test_input_topic_is_the_postprocessed_map():
    """slope / roughness only exist on the post-processed map."""
    assert DEFAULT_INPUT_TOPIC == "/elevation_map_raw_post"


def test_output_topic_is_unchanged():
    """Nav2's TraversabilityLayer is configured against this topic."""
    assert DEFAULT_OUTPUT_TOPIC == "/terrain_traversability"


def test_packaged_yaml_matches_node_defaults():
    params = _yaml_params()
    assert params["input_topic"] == DEFAULT_INPUT_TOPIC
    assert params["output_topic"] == DEFAULT_OUTPUT_TOPIC
