"""Typed application configuration.

Defaults live here (single source of truth for types); the YAML file
``config/default.yaml`` overrides them and is the file operators edit in
the field. Unknown YAML keys raise immediately — a misspelled key in a
pre-experiment rush must fail loudly, not silently do nothing.
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, List, Optional

import yaml


@dataclass
class RosTopics:
    """Every topic the application touches, in one place."""

    processed_ping: str = "/sss_processor/processed"
    odom: str = "/blueboat/odom"
    navsat: str = "/mavros/global_position/global"
    compass_hdg: str = "/mavros/global_position/compass_hdg"
    vfr_hud: str = "/mavros/vfr_hud"
    ping_enable: str = "/side_scan_sonar/ping/enable"
    svlog_enable: str = "/sss_processor/log/enable"
    # Raw per-side profiles (blueboat_interfaces/OmniscanProfile) from the
    # sonar driver: the verbatim native bins the processor also consumes.
    port_profile: str = "/side_scan_sonar/port/profile"
    starboard_profile: str = "/side_scan_sonar/starboard/profile"
    # ---- placeholders (repositories not present yet) ----------------------
    detections: str = "/sss_ai/detections"     # see ros/detections_listener.py
    pinger: str = "/blueboat/pinger_coordinates"  # Float32MultiArray [x, y]
    # Planned mission path (nav_msgs/Path), published by path_publisher.py.
    planned_path: str = "/set_path"
    # AI seabed analysis output (std_msgs/String, JSON; schema in HANDOVER).
    seabed_analysis: str = "/sss_ai/seabed_analysis"


@dataclass
class PipelineConfig:
    """START/STOP acquisition behaviour (bottom toolbar)."""

    # Command run by START. The app never launches sss_node.py: this launch
    # file starts the *processing* pipeline only (see launch/ directory).
    launch_command: List[str] = field(default_factory=lambda: [
        "ros2", "launch", "blueboat_sss", "SSS_processing_launch.py",
    ])
    # If true, START also publishes `true` on topics.ping_enable so the
    # already-running sss_node begins firing, and STOP publishes `false`.
    publish_ping_enable: bool = True
    # DEPRECATED — kept only so existing YAML files still load. Recording
    # is controlled by the Record ON/OFF toolbar toggle (recording
    # sessions); this flag is no longer read by the launcher.
    enable_svlog_on_start: bool = False
    # Delay between launching the pipeline and enabling pinging, to let
    # the processor node come up and subscribe.
    start_delay_s: float = 2.0
    # Shutdown escalation ladder: SIGINT -> (grace) -> SIGTERM -> (grace)
    # -> SIGKILL. SIGKILL is a last resort because `ros2 launch` cannot
    # forward a shutdown it never sees — which is how nodes get orphaned.
    stop_grace_s: float = 5.0
    stop_term_grace_s: float = 3.0
    # After the launch process exits, any process still matching one of
    # these patterns (pgrep -f) is killed: guarantees no orphaned node
    # (e.g. sss_processor_node) survives a STOP, no matter what.
    leftover_process_patterns: List[str] = field(default_factory=lambda: [
        "sss_processor_node",
    ])


@dataclass
class MosaicConfig:
    # Mosaic ground-sample distance. NOT a fixed constant any more:
    # with auto_cell_size the grid adapts to the data actually loaded
    # (across-track sample spacing = range / num_results, clamped), so a
    # 20 m / 600-sample log renders at ~3 cm instead of being blurred to
    # 25 cm, which is most of the resolution gap against SonarView.
    cell_size_m: float = 0.10
    auto_cell_size: bool = True
    min_cell_size_m: float = 0.02
    max_cell_size_m: float = 1.00
    initial_half_extent_m: float = 30.0
    render_hz: float = 4.0             # GUI raster refresh rate
    contrast_percentiles: List[float] = field(default_factory=lambda: [2.0, 98.0])
    # Cell-value policy where survey lines overlap (mapping/mosaic.py
    # PRIORITY_MODES): average | closest | oldest | newest. "closest"
    # (smallest slant range wins) is SonarView's default and the sharpest
    # single-pass choice.
    priority_mode: str = "closest"
    # ---- seabed-referenced EGN + window (SonarView parity, core/contrast) ----
    # When ``nadir_contrast`` is on (default) the waterfall/seabed pictures
    # are seabed-referenced-EGN'd (per-column seabed mean subtracted so the
    # pre-TVG range falloff is flattened and the whole swath is visible),
    # then windowed over the equalized values: high handle at a high seabed
    # percentile, low handle at a low seabed percentile. The un-referenced
    # water column keeps its raw low level and maps toward black; nothing is
    # erased. The window + EGN reference are stored in the seabed-image
    # metadata / npz so a picture stays losslessly invertible back to dB.
    nadir_contrast: bool = True
    seabed_high_pct: float = 99.5      # high handle: bright seabed / returns
    seabed_low_pct: float = 5.0        # low handle: dark seabed / shadows
    # Retained for signature/config stability; NO LONGER sets a handle — the
    # low handle now comes from the seabed distribution (its bright
    # near-field would otherwise re-crush the far range, the SonarView bug).
    water_column_pct: float = 90.0
    # Transmit-ringing core [m slant] excluded from the EGN reference and the
    # window statistics: a thin bright artefact that would otherwise skew
    # them. Still displayed (a thin bright centre line), never erased.
    water_column_min_m: float = 1.0
    # DEPRECATED — the waterfall is no longer a fixed ring; kept so old
    # YAML files load. Sizing now comes from waterfall_max_rows below.
    waterfall_rows: int = 1500
    # DEPRECATED — the waterfall draws native slant bins now (one column
    # per device range bin, width adapting to the acquisition), so
    # nothing is resampled onto a fixed 800-column grid any more. Kept
    # so old YAML files load.
    waterfall_columns: int = 800
    # Live waterfall memory cap in rows. The buffer grows with the
    # mission (every ping stays scrollable); past the cap the oldest
    # rows are dropped. Replay raises the cap to the log's own ping
    # count so a whole file is always scrollable.
    waterfall_max_rows: int = 100_000
    # Performance bounds (the GUI thread must stay responsive at 20 Hz
    # pings — an overloaded GUI starves the ROS executor and BEST_EFFORT
    # then drops real pings):
    # * max_grid_cells — cap on total mosaic cells; past it the grid is
    #   coarsened (existing data preserved via resample_from). 8 M cells
    #   ≈ 250 MB of planes.
    # * max_render_pixels — cap on the raster actually colormapped per
    #   render pass; larger grids render decimated (full extent, every
    #   Nth cell). Display only; the grid keeps full resolution.
    max_grid_cells: int = 8_000_000
    max_render_pixels: int = 2_000_000
    # * max_extent_m — a single ping whose samples would grow the grid
    #   past this extent is refused (pose glitch guard), never allocated.
    # * mosaic_render_hz — the mosaic's own colormap cadence (the
    #   waterfall keeps render_hz); a ping touches a sliver, so the
    #   mosaic re-colormaps only that sliver between full renders.
    # * waterfall_max_samples — the live waterfall's memory cap in
    #   samples (rows x columns), which is what the native-bin buffer
    #   actually costs: 100 k rows were sized for 800 columns, at 1200 /
    #   2400 native columns the row cap alone reached 1-2 GB.
    max_extent_m: float = 5000.0
    mosaic_render_hz: float = 2.0
    waterfall_max_samples: int = 120_000_000
    # * ping_lag_throttle — pings queued behind the GUI thread before
    #   rendering is skipped until it catches up (no ping is dropped from
    #   the buffers; only the display waits). 0 disables.
    ping_lag_throttle: int = 40
    # Professional-quality rasterization (see mapping/rasterizer.py):
    # across/along-track ping densification + bilinear splatting. Set
    # both to false to recover the legacy point-scatter mosaic (A/B).
    densify: bool = True
    bilinear_splat: bool = True


@dataclass
class DisplayConfig:
    """The one dB->pixel model every picture goes through
    (core/display_model.py; docs/SCIENTIFIC_BACKGROUND.md §9).

    * ``tl_k`` / ``tl_alpha_db_per_m`` — deterministic two-way
      transmission loss ``k·log10 r + 2·α·r`` removed first (the stream is
      pre-TVG): 40 dB/decade spherical spreading, 0.1 dB/m absorption at
      450 kHz.
    * ``x_bins`` / ``x_max`` — the empirical seabed curve ``A(r/h)`` is
      estimated per side on ``x_bins`` log-spaced bins of normalised slant
      range ``x = r/h`` up to ``x_max`` (30 = a 3 m altitude at 90 m).
    * ``min_bin_count`` — below this many samples a bin blends toward the
      Lambert prior.
    * ``warmup_rows`` — live, the model accumulates this many rows then
      freezes (one re-render of every tile); replay fits in one pass.
    * ``hi_pct`` — the window top ``hi`` is this percentile of the
      normalised seabed level; there is NO low handle (shadows go black by
      the power-law transfer on their own).
    * ``gamma`` — transfer exponent ``u = 10^(gamma·(e−hi)/10)``: 1.0 is
      linear power (SonarView), 0.5 amplitude. Seeds the Contrast slider.
    """

    tl_k: float = 40.0
    tl_alpha_db_per_m: float = 0.10
    x_bins: int = 40
    x_max: float = 30.0
    min_bin_count: int = 100
    warmup_rows: int = 300
    hi_pct: float = 95.0
    gamma: float = 0.7
    # Soft knee of the transfer: above this unit brightness highlights are
    # compressed instead of clipped (1.0 = hard clip). A wall face keeps
    # its texture for a few dB past ``hi``.
    knee: float = 0.7
    # How far [dB] the empirical seabed curve may leave the robust
    # physical line fitted through it: bins dominated by a wall's shadow
    # (or its face) on every ping are pulled back onto physics.
    curve_tolerance_db: float = 6.0


@dataclass
class SonarStreamConfig:
    """Live sonar stream reception (ros/sonar_listener.py).

    ``queue_depth``: subscriber history depth. The processor publishes
    ProcessedSSSPing BEST_EFFORT, so a subscriber may only be
    BEST_EFFORT too (RELIABLE would be QoS-incompatible and receive
    nothing at all). BEST_EFFORT does not retransmit, so the only
    protection against losing pings while the GUI thread is busy is a
    deep queue: 10 slots is 0.5 s at 20 Hz, which a single mosaic
    re-render can overrun. 200 slots is ~10 s of headroom and costs
    only a few MB.

    ``warn_on_ping_gap``: log when the device's own ping_number skips,
    so real acquisition loss is visible in the console instead of
    silently thinning the mosaic (our sea-trial logs lost ~8 % of pings
    upstream of the GCS; SonarView's recordings lose none).
    """

    queue_depth: int = 200
    warn_on_ping_gap: bool = True
    # Raw OmniscanProfile subscriptions (the native slant bins the
    # waterfall and the AI pictures draw): same depth reasoning as above
    # (a shallower queue drops profiles exactly under GUI load), a bounded
    # per-side cache keyed by ping_number, and how long a processed row
    # waits for a late profile before falling back to the re-projection.
    profile_queue_depth: int = 200
    profile_cache_per_side: int = 512
    profile_wait_ms: int = 60


@dataclass
class DepthConfig:
    """Depth compensation — the altitude used for slant-range correction.

    Mirrors SonarView's "Depth Compensation" source selector, because the
    same trade-off applies to us (see docs/SONARVIEW_SVLOG_ANALYSIS.md):

    * ``auto``   — bottom detection (FBR). Never drops a ping: an
      unlocked tracker falls back to a provisional or last-known value.
    * ``manual`` — fixed altitude in ``manual_m``.
    * ``off``    — no correction (ground range = slant range). This is
      SonarView's "Manual / 0 m"; for shallow water with h << R it is
      geometrically almost identical and far more robust than a wrong
      altitude, which both deletes real samples and warps the near range.

    The selector governs the **correction** only. Separately from it, and
    in every mode, the first ``nadir_blank_m`` of slant range is removed:
    the profile leaves the transducer at ~55 dB — brighter than any seabed
    return — and that transmit ringing used to sit as a bright core right
    under the boat in every ``off`` image and splat onto the track line in
    the mosaic.

    The blank is deliberately **narrow, and is not the water column**.
    Measured on ``diffDepthCompensation.svlog`` the ringing decays through
    43 dB at 0.27 m to 33 dB at 1 m, after which the water column settles
    to 16–28 dB — darker than the seabed, honest data, and needed on
    screen: the waterfall, the mosaic and the AI tiles all have to be
    continuous. Blanking out to the tracked altitude (9.4 m on that log)
    punches a hole through every one of them.

    ``blank_nadir``: set False to disable the blank entirely.
    ``nadir_max_fraction``: cap on the blank as a share of the ping's
    slant extent, so a mis-set ``nadir_blank_m`` can never empty a ping —
    downstream reads an all-NaN row as a session gap.

    ``warn_bottom_fraction``: if the detected bottom sits closer than
    this fraction of the ping, the range setting is too long for the
    depth and bottom detection becomes unreliable (our 80 m sea-trial
    logs put the bottom at 8 % — SonarView reported "Detected N/A" on
    exactly those files).
    """

    mode: str = "off"                  # auto | manual | off  (default off)
    manual_m: float = 0.0
    # Nadir blank: still applied to the GROUND projection (the mosaic) so
    # the transmit ringing never splatters the boat track in `off` mode.
    # The raw-slant waterfall and the AI pictures no longer erase it —
    # they carry the full raw dB and darken the nadir by the colour window
    # instead (see mosaic.nadir_contrast).
    blank_nadir: bool = True
    nadir_blank_m: float = 0.75
    nadir_max_fraction: float = 0.5
    warn_bottom_fraction: float = 0.12


@dataclass
class AlignmentConfig:
    """Sea-trial pose alignment (utils/pose_alignment.py).

    pose_source:
      * "auto"     — use the pose embedded in ProcessedSSSPing unless it
        is frozen at the origin while GCS telemetry shows the boat
        elsewhere; then re-stamp pings from the GCS's own RobotState
        (fixes the live '(0,0) pings with GPS on' pathology; rosbag
        replays are unaffected because their synthesized odom is sane);
      * "embedded" — always trust the ping's pose (legacy behaviour);
      * "gcs"      — always re-stamp from GCS telemetry.
    gps_fallback: synthesize RobotState from NavSatFix + compass when
    /blueboat/odom is silent or zero-frozen (GPS dead reckoning).

    heading_source: yaw used for *live* ping/marker orientation:
      * "compass"  — /mavros/global_position/compass_hdg (converted once
        at ingestion), falling back to odom yaw when the compass has
        been silent for compass_stale_s. The compass is the true-north
        reference and is what fixed the field's misaligned live mosaic;
      * "embedded" — the ping's own odom-quaternion yaw (legacy).
    Replay from .svlog is unaffected (it derives yaw from mavlink
    ATTITUDE and never passes through the live alignment path).

    pinger_frame:
      * "auto"  — trust the wire shape: a 3-vector is the USBL-native
        vehicle-relative [x fwd, y port] (rotated through the robot
        pose nearest the fix), a 2-vector is the corrected world
        position (the producer's fixed_pinger path);
      * "robot" / "world" — force one interpretation (legacy setups).
    pinger_stale_after_s: hide the pinger marker when no fix arrived
    for this long (0 disables)."""

    pose_source: str = "auto"          # auto | embedded | gcs
    frozen_epsilon_m: float = 0.05
    frozen_after_pings: int = 20
    gps_fallback: bool = True
    heading_source: str = "compass"    # compass | embedded
    compass_stale_s: float = 2.0
    pinger_frame: str = "auto"         # auto | robot | world
    pinger_stale_after_s: float = 10.0


@dataclass
class SeabedConfig:
    """Waterfall-domain AI imaging (core/seabed_imager.py).

    rows/stride: 256/128 = 50 % overlap; see the module docstring for
    the along-track-footprint and tiling-guarantee justification."""

    rows: int = 256          # picture rows per image (window height)
    stride: int = 128        # emit every N rows; overlap = rows - stride
    # Along-track geometry of a picture row (decision 2026-09-05,
    # PROVISIONAL — revert to "ping" if detector results are worse):
    #   "square" — rows are resampled to one across-track bin pitch each
    #              (nearest-ping selection, ping index recorded per row),
    #              so a pixel is the same size along- and across-track and
    #              objects keep their shape at any boat speed;
    #   "ping"   — one row per ping, the previous dataset contract.
    row_geometry: str = "square"
    # DEPRECATED — images are raw waterfall now (one column per native
    # slant bin, width adapting to the acquisition); kept so old YAML
    # files load.
    columns: int = 800


@dataclass
class InterpolationConfig:
    """Small-gap fill between consecutive sonar lines (render-time only)."""

    max_gap_m: float = 0.75     # never fill farther than this from real data
    min_neighbors: int = 3      # cells with fewer valid neighbours stay empty


@dataclass
class GeoConfig:
    """odom <-> GPS anchoring (mapping/geo.py, ported from BlueBoat-MCS).

    The map scene is local east/north metres about the first accepted
    GPS fix; the odom frame is reconciled to it by a translation-only
    fit estimated online (median of EN − world offsets over
    ``fit_window_s``). A translation is observable while stationary, so
    the anchor converges within ~1 s of fixes with no motion."""

    fit_window_s: float = 180.0
    min_pairs: int = 5
    refit_period_s: float = 5.0
    max_residual_m: float = 6.0


@dataclass
class MapConfig:
    # DEPRECATED — no longer applied. The odom frame is local ENU with
    # absolute yaw (robot-side guarantee); a rotation here was only ever
    # a workaround for the old hybrid frame. Kept so old YAMLs load.
    frame_yaw_offset_deg: float = 0.0
    # Gate the whole map on the GPS anchor: nothing is drawn and clicks
    # are refused until the odom<->GPS fit is valid. Set false to draw
    # immediately with an identity anchor (bench runs with no GPS
    # source; tiles stay off because there is nothing to georeference).
    require_gps_anchor: bool = True
    # Background tile sources ({z}/{x}/{y} slippy scheme).
    osm_url: str = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
    satellite_url: str = ("https://server.arcgisonline.com/ArcGIS/rest/services/"
                          "World_Imagery/MapServer/tile/{z}/{y}/{x}")
    use_satellite: bool = True
    tile_cache_dir: str = "~/.cache/blueboat_gcs/tiles"
    max_concurrent_tile_requests: int = 6


@dataclass
class AcquisitionConfig:
    """Runtime sonar acquisition control (the range slider).

    ``sss_node`` re-reads its parameters only on the ping/enable RISING
    edge (its worker short-circuits an enable while already pinging), so
    a live range change is the documented three-step dance the GCS now
    performs itself: publish enable=false, wait ``settle_delay_s``, set
    ``range_length_mm`` via the node's ``set_parameters`` service, and
    re-enable on the result (always — acquisition is never left off).
    """

    range_min_m: float = 5.0
    range_max_m: float = 50.0
    settle_delay_s: float = 0.5
    param_node: str = "side_scan_sonar"


@dataclass
class RecordingConfig:
    """Recording sessions (core/recording_session.py).

    adopt_delay_s: how long after Record OFF the .svlog adoption sweep
    runs. The log_enable=False message travels asynchronously; adopting
    the instant the session ends can move the file while the processor
    still holds its old absolute path open, whose next append then
    recreates a headerless stub in data_root. STOP and app-close adopt
    synchronously instead (pinging is already off there, so the
    processor writes nothing more)."""

    adopt_delay_s: float = 1.5


@dataclass
class SimConfig:
    """Built-in simulator (`--sim`) for bench-testing without ROS."""

    origin_lat: float = 43.6961
    origin_lon: float = 7.3080
    ping_hz: float = 15.0
    speed_mps: float = 0.8


@dataclass
class AppConfig:
    topics: RosTopics = field(default_factory=RosTopics)
    pipeline: PipelineConfig = field(default_factory=PipelineConfig)
    mosaic: MosaicConfig = field(default_factory=MosaicConfig)
    interpolation: InterpolationConfig = field(default_factory=InterpolationConfig)
    seabed: SeabedConfig = field(default_factory=SeabedConfig)
    alignment: AlignmentConfig = field(default_factory=AlignmentConfig)
    depth: DepthConfig = field(default_factory=DepthConfig)
    sonar_stream: SonarStreamConfig = field(default_factory=SonarStreamConfig)
    display: DisplayConfig = field(default_factory=DisplayConfig)
    map: MapConfig = field(default_factory=MapConfig)
    geo: GeoConfig = field(default_factory=GeoConfig)
    recording: RecordingConfig = field(default_factory=RecordingConfig)
    acquisition: AcquisitionConfig = field(default_factory=AcquisitionConfig)
    sim: SimConfig = field(default_factory=SimConfig)
    data_root: str = "../../../../data/SSS_data"   # same root as the existing pipeline


def _apply(obj: Any, data: dict, path: str = "") -> None:
    for key, value in data.items():
        if not hasattr(obj, key):
            raise KeyError(f"Unknown config key: {path}{key}")
        current = getattr(obj, key)
        if dataclasses.is_dataclass(current) and isinstance(value, dict):
            _apply(current, value, path=f"{path}{key}.")
        else:
            setattr(obj, key, value)


def load_config(yaml_path: Optional[Path] = None) -> AppConfig:
    """Build the config from defaults, then overlay the YAML file if present."""
    cfg = AppConfig()
    if yaml_path is None:
        yaml_path = Path(__file__).parent / "default.yaml"
    if yaml_path.is_file():
        with open(yaml_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        _apply(cfg, data)
    return cfg
