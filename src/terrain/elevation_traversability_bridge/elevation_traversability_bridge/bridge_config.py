"""Single source of truth for the bridge's topic / geometry defaults.

The input topic MUST be elevation_mapping's *post-processed* map, not its
*fused* map:

  /elevation_map          fused map.  Layers: elevation, upper_bound,
                          lower_bound, color, uncertainty_range.
                          -> NO terrain-analysis layers.
  /elevation_map_raw_post raw map + postprocessing filter chain output.
                          Layers: ... elevation_inpainted, elevation_smooth,
                          normal_vectors_x/y/z, slope, roughness.
                          -> this is where slope / roughness live.

The bridge needs ``slope`` and ``roughness`` (see
``bridge_processor.REQUIRED_LAYERS``), so it has to read the post-processed
map.  Pointing it at the fused map makes every frame fail with
"missing required layer 'slope'" and nothing is ever published downstream.
"""

#: elevation_mapping's post-processed map (carries slope / roughness).
DEFAULT_INPUT_TOPIC = "/elevation_map_raw_post"

#: Nav2 ``terrain_traversability_layer`` subscribes this (see
#: go2-nav2-terrain nav2_params.yaml).
DEFAULT_OUTPUT_TOPIC = "/terrain_traversability"

#: The GridMap frame the bridge accepts; must match the Nav2 global frame.
DEFAULT_REQUIRED_FRAME_ID = "map"

#: Output OccupancyGrid cell pitch in meters.  elevation_mapping publishes
#: at 0.10 m, so this triggers a 2x nearest-neighbour upsample.
DEFAULT_TARGET_RESOLUTION = 0.05
