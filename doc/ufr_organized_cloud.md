# UFR patch: organized cloud + range/signal/reflector images (multiScan)

UFR-specific addition on top of upstream `sick_scan_xd`, built for the multiScan136 driver
(`sick_scansegment_xd` / `RosMsgpackPublisher`). Not applicable to picoScan or the classic
single-layer scanners handled by the rest of this repo.

## Why

Upstream `sick_scan_xd` never publishes an organized (image-shaped) point cloud — even its
"structured" custom pointclouds are flattened to `height=1` in
`convertPointsToCustomizedFieldsCloud`. UFR wanted multiScan's output shaped like an Ouster
sensor's (`height=layers`, `width=azimuth`, plus range/signal images) so filters written for
Ouster's organized-cloud/range-image conventions can run against this lidar too.

## What it adds

Gated by one new `Config` field, `organized_cloud_enable` (default `false`; mirrors the existing
`imu_enable` pattern — same `-organized_cloud_enable=0|1` CLI/ROS-param wiring). When on, four new
topics are published per full revolution, built from a single echo (echo 0):

- **`organized_cloud`** (`PointCloud2`) — `height` = number of layers reported this frame, `width`
  = point count of the densest layer, fields `x,y,z,i,range,t,ring`. `is_dense=false`; a column
  with no matching return gets `x/y/z=NaN, range=0` (same convention as a dropped shot on a real
  organized cloud).
- **`range_image`** / **`signal_image`** (`sensor_msgs/Image`, `mono8`) — same grid as the cloud,
  normalized per-frame for direct viewing (0 reserved for "no return"). `range_image` uses a log
  scale (a scene with both a close obstacle and open room behind it spans orders of magnitude;
  linear scaling crushes the near end to 1-2 grey levels). `signal_image` uses linear scaling.
  Full-precision values are only on `organized_cloud`'s `range`/`i` fields — matches Ouster's own
  `os_image_node`, which does the same 8-bit clamp-for-viewing and says as much in its header
  comment.
- **`reflector_image`** (`mono8`, 0 or 255) — multiScan's `reflectorbit`, a firmware flag set when
  a point hits a designated retroreflector (nav landmark). **Not** a calibrated reflectivity value
  (see limitations below).

Each image is additionally row-repeated (`expandImageRows`) so it doesn't render as a
14-pixel-tall sliver: the repeat factor is computed per-frame from the layers' own measured
elevation spacing vs. the column azimuth spacing (no hardcoded layer count or vertical FOV), so it
keeps working if a different multiScan/picoScan configuration reports a different layer count.

New `Config`/launch parameters (same wiring pattern as `imu_enable`/`imu_topic` throughout):
`organized_cloud_enable`, `organized_cloud_topic`, `range_image_topic`, `signal_image_topic`,
`reflector_image_topic`.

## How column/row placement works

- **Row** = layer index, but flipped (`height-1-layer`) so image row 0 is the highest-elevation
  layer — verified empirically (layer 3 sits at -7°, layer 13 at +35° on the unit tested), since
  layer index increases with elevation but image row 0 is conventionally drawn at the top.
- **Column** = the point's own measured azimuth (negated + wrapped so increasing azimuth reads
  left-to-right), not its position in the row's point list. An earlier version used
  index-proportional placement and produced a visible shear on straight edges: rows drop points
  independently (a beam with no return isn't added to that row at all), so after a drop every
  later point in that row's list shifts by one slot relative to a row that didn't drop there —
  same real-world edge, different column, per row.
- Placement is **gather, not scatter**: for each output column, the nearest real point (by
  azimuth) is looked up via binary search over that row's azimuth-sorted points, rather than
  scattering each point into its own rounded bin. Scattering left single-pixel gaps scattered
  through even a fully-covered row (real per-shot azimuth jitter meant neighbouring points would
  occasionally round into the same bin, leaving the next one empty — rendered as salt-and-pepper
  "TV static").
- The gather's acceptance radius is **per-point, not a single row-wide average**: it's derived
  from that point's own neighbour spacing (capped so it can bridge a local cluster gap without
  painting across a genuinely empty arc). A row-wide average pitch was tried first and failed
  visibly on layer 7, which — confirmed directly from the vendor's own unprocessed
  `cloud_all_fields_fullframe` output, independent of any of this code — samples in tight pairs
  (~0.12° apart) separated by bigger gaps (~0.37-0.5°), a real per-beam hardware characteristic.
  A global average threshold was too tight for the wide side of that pattern and rejected roughly
  half of a row that was actually fully (2160/2160) covered.

## Known limitations — sensor side, not fixable in this driver

- **No calibrated reflectivity channel.** Ouster's `REFLECTIVITY` is range-compensated and
  material-calibrated by firmware. multiScan has no equivalent — only raw signal amplitude (`i`)
  and the binary `reflectorbit`. Observed on real surfaces: signal amplitude barely varies with
  material/colour/texture on a flat surface at a given range/angle; almost all visible contrast
  in `signal_image` comes from edges/corners (angle-of-incidence and partial-beam effects), not
  material. Checked that this isn't a display/clipping artifact — raw values do vary smoothly
  across a frame (confirmed via direct percentile inspection, not just visually).
- **No per-beam calibration correction available.** This code trusts the sensor's reported
  x/y/z/azimuth per point as-is. Ouster's driver applies a per-unit beam-calibration table; there's
  no equivalent data available from sick_scan_xd for multiScan. Attempted to check for a systematic
  per-ring azimuth offset by comparing where a test object's edge appeared in each ring — result
  was inconclusive: a real 3D object's silhouette legitimately shifts with elevation angle (each
  ring slices the object at a different height), and at least two of the rings being compared
  appeared to be looking at a different close object entirely during that test. **Open item**: a
  clean test needs one large flat target square to the sensor (removes the 3D-parallax confound)
  observed with nothing else in range, to determine whether real per-beam misalignment exists.
- **Layer count observed vs. spec'd.** multiScan136 is documented as 16 layers, 2 of them
  higher-resolution (~8x denser). Only 14 layers were present in `scandata` on the unit and
  configuration tested here; the 2 hi-res layers never showed up (possibly filtered out via
  `host_LFPlayerFilter`'s stored on-device state, not confirmed). `organized_cloud`'s height
  reflects however many layers the sensor actually reports each frame — it isn't hardcoded to 16 —
  so this isn't a bug in this patch, just worth knowing if you're expecting a specific row count.
- **Pre-existing, unrelated:** `sick_multiscan.launch`'s `imu_enable` `<param>` is hardcoded to
  `value="True"` rather than reading `$(arg imu_enable)` — no `<arg name="imu_enable">` exists at
  all. The CLI/launch override for it has always been a no-op, on both `master` and this branch.
  Left as-is (out of scope), flagging so it isn't mistaken for behaviour this patch introduced.
- **Pre-existing, unrelated:** boolean/int `key:=value` overrides passed to
  `ros2 launch sick_scan_xd sick_multiscan.launch.py` collide with ROS2's legacy "remap rule"
  argument parsing and raise a type-mismatch exception internally (visible as
  `rosGetParam(..., 0, b) failed, InvalidParameterTypeException` in the log), silently leaving
  the launch-file default in effect instead of applying the override. String-typed overrides
  (`hostname`, topic names, etc.) are unaffected. Worked around here by setting defaults directly
  in `sick_multiscan.launch` rather than fighting it at the CLI. A real fix belongs upstream in
  this driver's ROS2 argument handling — out of scope for this patch.

## Testing performed

Built and run against a real multiScan136 unit (bench test, `hostname=192.168.0.1`) for several
sessions across the changes above; not run against picoScan or in CI. `organized_cloud`,
`range_image`, `signal_image`, `reflector_image`, and `imu` all verified live over UDP, including
frame-to-frame stability spot checks and the per-ring diagnostics described above.
