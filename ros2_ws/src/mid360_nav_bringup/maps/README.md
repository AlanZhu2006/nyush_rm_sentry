FAST-LIO/ICP map sets are stored here. `arena` is the previously deployed
navigation fixture. `arena_nuc` preserves the map set from the original NUC
working tree. They cover different map coordinates and must be selected as
complete sets. Newly generated maps remain ignored until deliberately reviewed.

Verify both tracked sets with `sha256sum -c SHA256SUMS` from this directory.

Create and save a map named `arena` with the chassis on level ground:

  just map arena
  # Ctrl-C in that terminal saves PCD + PGM/YAML. `just map-save arena`
  # still works if you want a snapshot without stopping mapping.

FAST-LIO gravity-aligns `camera_init`. Verify the ground orientation and
landmark alignment against the current sensor mount before driving on a saved
map. Rebuild the map if its original frame is incompatible.

The resulting `arena.pcd`, `arena.pgm`, and `arena.yaml` must stay together.
The PCD is used by ICP localization; the PGM/YAML pair is used by Nav2.

Start navigation with:

  just nav arena

Keep the robot still while ICP searches within 2.25 m of the supplied initial
pose and checks all headings. The current 1225-candidate full-map search can
take about 50 seconds on the Jetson. It publishes `map -> odom` only after the
fitness, overlap, RMSE, and seed-correction gates pass. If the robot is outside
the search volume, use RViz `2D Pose Estimate` as a new seed.
