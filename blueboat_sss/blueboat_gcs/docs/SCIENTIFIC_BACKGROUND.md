# Scientific background — side-scan sonar display, nadir handling and radiometric normalisation

*Written 2026-09-05 for the BlueBoat GCS display rework. Every DOI in §10 was resolved
through Crossref on 2026-09-05 with `python3 -m blueboat_gcs.analysis.check_dois
blueboat_gcs/docs/SCIENTIFIC_BACKGROUND.md` (32 DOIs, 0 problems). Measurements in §8
come from the logs named there and are reproducible with the commands given.*

This document answers one question: **what does the literature say a side-scan sonar
(SSS) waterfall should look like, and how should the raw Omniscan 450 stream be turned
into the pictures the operator sees and the detector is trained on?** It exists because
the GCS rendered the same `.svlog` visibly worse than Cerulean's SonarView (bright
"nadir", vertical bands, grey noise inside shadows, a bright far-range rim), and the fix
had to be grounded rather than tuned by eye. §9 states the design that follows from it;
the code lives in `blueboat_gcs/core/display_model.py`.

---

## 1. Geometry: what a ping contains and why there is a nadir gap

A side-scan ping is a time series of echo intensity. Sample `i` of `N` over a configured
range `R` was received at two-way travel time `t_i`, i.e. from **slant range**
`r_i = c·t_i/2 = i·R/(N−1)` (Blondel 2009, ch. 2; Lurton 2010, ch. 2). The transducer sits
at height `h` above the seabed (the *altitude*; on a surface vessel it is the water
depth under the hull). Geometrically:

* **Water column** — samples with `r < h` cannot contain seabed. They carry transmit
  ringing right at `r ≈ 0`, volume reverberation, and any object in the water (fish,
  hulls, surface multipath). In a raw slant-range display this is the dark band down the
  middle of the waterfall; it is *not* an artefact, it is the geometry (Johnson &
  Helferty 1990; Blondel 2009, §4.2).
* **First bottom return (FBR)** — the echo at `r ≈ h` from the seabed directly below.
  It is strong (near-normal incidence) and marks the seabed onset; tracking it is how
  the altitude is recovered when no altimeter is available (Al-Rawi et al. 2017).
* **Ground range** — `y = sqrt(r² − h²)`. Projecting onto ground range ("slant-range
  correction") removes the water column and de-compresses the near range; it is what a
  mosaic needs. The raw slant-range waterfall keeps the water column and is the domain
  SonarView calls the conventional side-scan display: *"when Depth Compensation is off
  … the sonar signal [is] plotted from the center outward, and there is a blank zone at
  the beginning corresponding to the depth of the water"* (Cerulean, SonarView display
  controls). With compensation on, *"SonarView calculates the depth from the sonar
  signal and does the non-linear compensation required to remove the 'nadir'"* — the
  same slant-to-ground projection as above.
* **Along-track scale** — consecutive pings are `v·T` apart (`v` boat speed, `T` ping
  period). Drawing one pixel per ping and one pixel per range bin therefore squashes or
  stretches the picture unless the along-track axis is resampled to the across-track
  pitch ("speed correction", Chavez 1986; Blondel 2009, §4.3). At 20 Hz and 0.5–0.9 m/s a
  ping row is 26–45 mm along-track against a 25–80 mm range bin (§8), so an unscaled
  display is anisotropic by a factor of up to two — exactly the difference the user saw
  between our waterfall and SonarView's.

The nadir gap is thus closed by geometry, not by filtering: a display either shows the
water column (slant domain) or projects it away (ground domain). What the literature
*does* filter is the bottom estimate that separates the two (§4).

## 2. Radiometry: why intensity falls with range, and what TVG and EGN correct

The received echo level obeys the sonar equation (Lurton 2010, ch. 2 and ch. 6)

    EL = SL − 2·TL(r) + BS(θ) + 10·log10 A(r) + D(φ)

with source level `SL`, one-way transmission loss `TL(r) = k/2·log10 r + α·r` (spherical
spreading `k = 40` dB/decade two-way; absorption `α ≈ 0.10 dB/m` at 450 kHz in seawater,
from the Francois & Garrison 1982 model tabulated in Lurton 2010), the seabed
backscattering strength `BS(θ)` at grazing angle `θ = asin(h/r)`, the insonified area
`A(r)` and the vertical beam pattern `D(φ)`. Two consequences matter for display:

1. **Range falloff.** For a flat seabed `BS` follows approximately Lambert's law,
   `BS(θ) = μ + 10·log10 sin²θ` (Lurton 2010, §7; Hellequin et al. 2003), so the echo
   falls with range through both `TL` and the shrinking grazing angle. On our device this
   totals **60–70 dB per decade of slant range** (§8), i.e. the far half of a 20 m swath is
   30–45 dB darker than the near half. A single fixed brightness window over raw values
   therefore either crushes the far range to black or saturates the near range.
2. **Time-varying gain (TVG).** Analogue sonars compensate the deterministic part of
   the falloff in the receiver with a gain increasing with time-since-transmit —
   conventionally "20 log R" (extended targets) or "40 log R" (point targets) plus
   `2αR` (Lurton 2010, ch. 6). The Omniscan 450 stream is **not** TVG-compensated: the
   vendor documentation describes no such gain, `pwr_results` are documented only as
   "power results scaled from min_pwr_db to max_pwr_db … normally converted back to linear
   power or signal levels" (Cerulean, Omniscan API), and the falloff measured on the field
   logs is the full physical one (§8). The simulator reproduces this deliberately
   (`tvg_compensation: 0`, matched to the corpus).

The data-driven correction of the residual (beam pattern, grazing angle, sediment) is
the **empirical gain normalisation** (EGN) family: estimate the mean echo as a function
of range or grazing angle from the imagery itself and divide it out. Cervenka & de
Moustier (1993) introduced it for beam-pattern correction of SeaMARC imagery; Capus et
al. (2008) separate a *range* term and an *angular* term, both estimated from the
data, explicitly for visualisation *and* classification; Chang et al. (2010) normalise
by the mean intensity per grazing angle; Burguera & Oliver (2016) apply the same idea
for high-resolution mapping; Zhao et al. (2017) refine it with unsupervised sediment
classes so that a sediment change is not mistaken for range falloff; Xu et al. (2023)
evaluate a "canonical" representation — beam-pattern correction, Lambertian
incidence-angle correction (cos, cos², cot variants) and slant-range correction — and
show that it improves cross-survey consistency for learning. Retinex-style methods
(Ye et al. 2019; Zhou et al. 2024) are the image-processing cousins: divide by a smooth
illumination estimate.

Two properties of a good EGN reference follow from these papers and from our own
failure (§8): the reference must be **smooth in range** (a per-column mean is not, and
every discontinuity becomes a vertical band) and **robust to scene content** (shadows,
walls and targets at a fixed range bias a mean; Capus et al. fit smooth curves, Chang
et al. average over many pings per grazing-angle bin). A median per bin followed by
smoothing satisfies both; a fitted physical law (`k·log10 r + 2αr`) with an empirical
angular residual is the same thing with the deterministic part made explicit — that is
the form adopted in §9.

## 3. Speckle and shadows: the statistics the display has to respect

Fully developed speckle makes the intensity of a homogeneous seabed patch
exponentially distributed (Rayleigh amplitude), so in decibels it has a standard
deviation of about 5.6 dB and a heavy bright tail; real seabeds are heavier-tailed still
(K-distributed reverberation, Abraham & Lyons 2002). Measured on our logs the seabed
residual about its smooth range curve spans ~30 dB between the 5th and 95th percentiles
(§8). Acoustic shadows — the absence of return behind an object or a wall — sit at the
receiver noise floor, 15–40 dB below the local seabed. Reed, Petillot & Bell (2003) show
that the shadow, not the highlight, is the most discriminative feature for object
detection in SSS; a display that lifts the noise floor into mid-grey (as a data-driven
low handle contaminated by the shadows themselves does) destroys precisely that
feature.

Despeckling filters exist in two families: local-statistics minimum-mean-square-error
filters for multiplicative noise (Lee 1980, 1981; Frost et al. 1982; Kuan et al. 1985),
and edge-preserving diffusion or non-local methods (Perona & Malik 1990; SRAD, Yu &
Acton 2002; non-local means, Buades et al. 2005; BM3D, Dabov et al. 2007), with
sonar-specific variants (Grabek & Cyganek 2019). They trade texture for smoothness.
**For the detector feed we do not despeckle**: speckle texture is signal for seabed
classification, modern detectors are trained on it (Sethuraman et al. 2025), and any
filter would be an irreversible choice baked into the dataset. Despeckling remains an
optional operator-view enhancement, applied after the pipeline below, never before
export.

## 4. Bottom tracking

The water column / seabed boundary must be known per ping to (a) classify samples for
the reference estimate and (b) darken the nadir consistently. Classical trackers
threshold the profile against a noise window with a persistence test (Al-Rawi et al.
2017); this is what `sss_helper.detect_fbr_slant_m` / `FBRTracker` do, with bootstrap
and re-lock logic. Learned trackers replace the heuristics with a 1-D CNN (Yan et al.
2019), a 1-D U-Net (Yan et al. 2021) or a 2-D semantic segmentation of the waterfall
into water column and seabed (Zheng et al. 2021); Yu et al. (2020) address the same
problem for AUV missions with blind zones. The tracked bottom is noisy at the sample
level (our tracker tolerates ±0.3 m), so anything that depends on it must be robust to
jitter — the reference estimate uses per-bin medians, and the display uses a causally
smoothed altitude while metadata keeps the raw value.

## 5. Display mapping: log or linear, and where the window comes from

Sonar intensities are stored in dB for dynamic range, but the industry displays are
closer to linear power or amplitude: SonarView's API notes that the dB scaling is "just
to keep high dynamic range in the u16 sized data elements" and that values are
"normally converted back to linear power". The difference is what happens to shadows: a
region 15 dB below the seabed is 0.03 of it in power (near black), 0.18 in amplitude
(dark grey), and, under a 40 dB dB-linear window, 62 % grey. Linear-power display is
what makes SonarView's shadows crisp; the price is a heavy-tailed histogram where a few
percent of the seabed saturates, which is the "grainy" look of every commercial SSS
viewer.

Histogram-based enhancements — global histogram equalisation, CLAHE (Pizer et al.
1987) — adapt the window locally and give attractive operator displays, but they are
scene-dependent and non-invertible, so they belong to the operator view only. The
detector feed needs one **mission-wide** window so that the same seabed reads at the
same brightness in every tile (Capus et al. 2008 make the same argument for
classification), and a stored, closed-form mapping so the picture inverts back to dB
(our standing rule; the raw float dB is kept in the companion `.npz` regardless).
Robustness matters here too: a low window handle taken as a low percentile of "seabed"
samples is dragged down by the shadows it should be excluding (§8). The window top is
therefore set from a robust high percentile of the range-normalised seabed, and there is
no low handle at all: the power-law transfer sends anything far below the seabed to
black on its own.

## 6. What SonarView documents

From the Cerulean documentation (fetched 2026-09-05):

* Display controls (Omniscan 450): Contrast, Brightness, Heading Up, 3D mode,
  Waterfall, **Depth Compensation** ("when off … a blank zone at the beginning
  corresponding to the depth of the water"; "when on, SonarView calculates the depth from
  the sonar signal and does the non-linear compensation required to remove the
  'nadir'"), Color Picker. No TVG, gain-versus-range, smoothing or stretch control is
  documented.
* Device controls: Gain / Auto Gain (device receiver gain ladder, index 0–7), Range,
  Pulse Length (percentage of the round-trip time at the current range), Profile
  Resolution (600 fits one Ethernet packet). No TVG.
* API: `pwr_results` "power results scaled from min_pwr_db to max_pwr_db"; "the
  amplitude of the results are in dB. This is just to keep high dynamic range in the
  u16 sized data elements. Normally these are converted back to linear power or signal
  levels."

So SonarView's visible behaviour — a bottom-tracked blank nadir, uniform seabed across
the swath, black shadows, true-scale rows — is consistent with: a bottom tracker, a
smooth range gain, a linear-power transfer, and a geometric row scale. None of it
requires erasing data on our side; the same appearance is obtained by the mapping in §9.

## 7. Geometry of the AI pictures

Detector inputs are waterfall-domain, boat-relative pictures (project rule CM-10):
world-frame mosaic crops distort near turns and mix revisits. Within that domain two
choices remain. Along-track: one row per ping keeps every sample verbatim but makes an
object's apparent length depend on speed and ping rate; resampling to square pixels
("speed correction", Chavez 1986) makes objects the same shape at any speed — the
representation Xu et al. (2023) evaluate as canonical — at the cost of replicating or
skipping pings. Across-track: the native slant-bin pitch keeps every bin verbatim; the
water column is kept in the picture and darkened by the mapping so tiles stay continuous
(no NaN holes, no invented samples). The project chose square pixels with nearest-ping
selection, recorded per row so the choice is reversible (`seabed.row_geometry`).

## 8. Measurements on our logs

Commands: the per-ping profiles and fits below come from `load_svlog` over the named
files (see `blueboat_gcs/core/svlog.py`); the forensics tool
(`python3 -m blueboat_gcs.analysis.svlog_forensics`) reproduces the acquisition
parameters and the bottom-sample numbers.

| Log | Range / bins / pitch | Tracked bottom (p10–p90) | Ringing at r = 0 → 1 m | Water column (1 m → bottom) | Seabed onset (FBR) | Far-range seabed | Along-track m/ping (aspect) |
|---|---|---|---|---|---|---|---|
| Simulation session 2026-09-04 (5474 pings) | 15 m (32 m after a mid-run change) / 600 / 25 mm | 2.9–3.9 m | 50–53 → 31 dB | 24–36 dB | 60–66 dB at 3.5–4.3 m | 30–45 dB at 10–11 m | 26 mm (1.0) |
| `diffDepthCompensation.svlog` (field, SonarView-recorded) | 20 m / 600 / 33 mm | 6.9–10.9 m | 44 → 32 dB | 27 dB at 2 m → 18 dB at 8 m | 42 dB at 7.7 m | 19 dB at 20 m | 34 mm (1.0) |
| `fullHarbourCleanExample.svlog` (field) | 95.6 m / 1200 / 80 mm | 4.2–9.1 m | 49 → 36 dB | 31 dB at 2 m → 18 dB at 8 m | 60 dB at 7.4 m | 1–2 dB at 80–95 m (noise floor) | 122 mm (1.5) |

Derived quantities (real logs; the sim log's range change makes a single fit
ill-posed, but its per-ping profiles show the same ordering):

* **Range falloff of the seabed median:** log-only fits of −64 dB/decade
  (`fullHarbourCleanExample`) and steeper on the short log where the far edge reaches
  the beam-pattern roll-off; a `k·log10 r + b·r` fit gives `k ≈ −57`, `b ≈ −0.08` dB/m on
  the 95 m log. The physical two-way spreading alone is −40 dB/decade; the remainder is
  the grazing-angle and beam-pattern term that the empirical curve has to absorb.
* **Water column vs seabed onset:** 15–25 dB below on the field logs, 25–35 dB below in
  the simulation. The water column is dark in the *data*; our display made it bright.
* **Seabed residual about the smooth curve** (`diffDepthCompensation`): p5 −14.7 dB,
  p50 0, p95 +15.3 dB, standard deviation 9.2 dB; in linear power the 90th percentile
  is 16× the median. (`fullHarbourCleanExample`: p5 −20, p95 +13.)
* **Shadow share:** samples more than 12 dB below the smooth curve inside the seabed
  class are 5.4 % (`diffDepthCompensation`), 13.4 % (`fullHarbourCleanExample`), and about
  14 % in the simulation (walls and objects). Their level is at the per-ping floor.

Root cause of the observed defects, traced in `core/contrast.py` (2026-09-03 version):
the per-column reference was a **mean** over "seabed" samples that included the shadows,
and the low window handle was the **5th percentile** of the same distribution. With 5–14 %
of shadow samples 15–40 dB down, that handle landed ~30 dB below the seabed level, so the
water column (only 15–35 dB down) rendered at 70–90 % brightness with the ringing
gradient on top — the "weird nadir" — and shadows rendered as grey noise. Columns
straddling the tracked bottom were seabed on some pings and water column on others, so
their references were biased (the dark band at the seabed onset), and any wall or
shadow at a fixed range biased its column's mean (vertical bands, lifted far rim).

## 9. Design derived from the above (implemented in `core/display_model.py`)

1. **One model per window, shared by the waterfall, the AI pictures and the mosaic**,
   estimated live in a warm-up then frozen, or fitted in one pass over a replayed log.
2. **Normalisation** `e = db + TL(r) − A_side(r/h)` with `TL(r) = k·log10 r + 2αr`
   (defaults `k = 40`, `α = 0.1` dB/m — the physical part, §2) and `A_side(x)` an
   empirical curve in normalised slant range `x = r/h` (the grazing-angle part, §2),
   per side because the two units carry independent gains. Per log-spaced bin of `x`
   the **mode** of the level histogram over seabed samples (`r ≥ max(h, ringing)`) is
   read (immune to shadows, walls and targets while plain seabed holds the plurality;
   a median already shifts by half a texture deviation at 30 % contamination). The
   modes are then tied to physics: a straight line in `log10 x` is found by consensus
   (each bin proposes a line of Lambert slope through itself; the proposal with the most
   bins within `curve_tolerance_db` wins — the seabed ridge, even when the whole far
   range lies in a wall's shadow on every ping, because the noise floor's TL-compensated
   level *rises* with range and cannot share one line with the seabed), its slope is
   refined on the inliers and clamped to `[−40, 0]` dB/decade, and only inlier bins
   refine the shape around it; any other bin takes the line exactly. Median-filtered
   across bins, interpolated, held at `A(1)` for `x < 1`. `x = r/h` makes it
   altitude-invariant and range-setting-invariant (no rebuild on a range change).
3. **Water column and ringing** darken by physics: for `r < h` the extrapolated `TL`
   predicts a seabed far brighter than anything in the water column, so `e` falls tens
   of dB below the seabed level and the transfer sends it to black. Nothing is masked,
   nothing erased; the raw float dB is kept (§1, §5).
4. **Transfer** `p = 10^(γ·(e − hi)/10)`, a power law with exponent `γ` (operator
   "Contrast"; `γ = 1` is SonarView's linear power, `γ = 0.5` amplitude), `hi` a robust
   high percentile of `e` over seabed samples, no low handle (§5); above a soft `knee`
   (0.7) highlights are compressed, `u = knee + (1−knee)(1 − e^{−(p−knee)/(1−knee)})`,
   so a wall face keeps its texture for a few dB past `hi` instead of clipping. Shadows
   go black because they are 15–40 dB below `hi`, not because a threshold was set.
   Defaults calibrated on the 2026-09-04 simulation log against SonarView's screenshot
   of the 2026-09-03 log: `γ = 0.7`, `hi` = p95 (median seabed brightness matched, ~4 %
   of seabed pixels saturated), `knee = 0.7`, `curve_tolerance_db = 6`.
5. **Invertibility**: the JSON of every picture carries the model (`k`, `α`, bin
   centres, the two curves, `hi`, `γ`, `knee`) and the `.npz` the curves and the raw dB;
   `p = u` below the knee else `knee − (1−knee)·ln(1 − (u−knee)/(1−knee))`,
   `e = hi + (10/γ)·log10 p`, `db = e − TL(r) + A(x)`.
6. **Display geometry**: the on-screen waterfall is drawn at true scale (along-track
   metres per ping over the bin pitch, §1); the AI pictures are resampled to square
   pixels by nearest-ping selection with the ping index recorded per row (§7).
7. **Not done, by decision**: no despeckling in the feed (§3); no histogram equalisation
   in the feed (§5); no bottom-tracked mask (§1, §6).

## 10. References

Peer-reviewed and book sources (DOIs verified through Crossref, 2026-09-05):

1. Abraham, D. A., & Lyons, A. P. (2002). Novel physical interpretations of K-distributed reverberation. *IEEE Journal of Oceanic Engineering*, 27(4), 800–813. https://doi.org/10.1109/JOE.2002.804324
2. Al-Rawi, M., Elmgren, F., Frasheri, M., Çürüklü, B., Yuan, X., Martínez, J.-F., Bastos, J., Rodriguez, J., & Pinto, M. (2017). Algorithms for the detection of first bottom returns and objects in the water column in sidescan sonar images. *OCEANS 2017 – Aberdeen*. https://doi.org/10.1109/OCEANSE.2017.8084587
3. Blondel, P. (2009). *The Handbook of Sidescan Sonar*. Springer Praxis. https://doi.org/10.1007/978-3-540-49886-5 — chapter "Sidescan sonar data processing": https://doi.org/10.1007/978-3-540-49886-5_4
4. Buades, A., Coll, B., & Morel, J.-M. (2005). A non-local algorithm for image denoising. *CVPR 2005*. https://doi.org/10.1109/CVPR.2005.38
5. Burguera, A., & Oliver, G. (2016). High-resolution underwater mapping using side-scan sonar. *PLOS ONE*, 11(1), e0146396. https://doi.org/10.1371/journal.pone.0146396
6. Capus, C. G., Banks, A. C., Coiras, E., Tena Ruiz, I., Smith, C. J., & Petillot, Y. R. (2008). Data correction for visualisation and classification of sidescan SONAR imagery. *IET Radar, Sonar & Navigation*, 2(3), 155–169. https://doi.org/10.1049/iet-rsn:20070032
7. Cervenka, P., & de Moustier, C. (1993). Sidescan sonar image processing techniques. *IEEE Journal of Oceanic Engineering*, 18(2), 108–122. https://doi.org/10.1109/48.219531
8. Chang, Y.-C., Hsu, S.-K., & Tsai, C.-H. (2010). Sidescan sonar image processing: correcting brightness variation and patching gaps. *Journal of Marine Science and Technology*, 18(6), 785–789. (No DOI registered; https://jmstt.ntou.edu.tw/journal/vol18/iss6/1/)
9. Chavez, P. S., Jr. (1986). Processing techniques for digital sonar images from GLORIA. *Photogrammetric Engineering and Remote Sensing*, 52(8), 1133–1145. (No DOI registered; USGS publication 70014528)
10. Dabov, K., Foi, A., Katkovnik, V., & Egiazarian, K. (2007). Image denoising by sparse 3-D transform-domain collaborative filtering. *IEEE Transactions on Image Processing*, 16(8), 2080–2095. https://doi.org/10.1109/TIP.2007.901238
11. Francois, R. E., & Garrison, G. R. (1982). Sound absorption based on ocean measurements. Part II: Boric acid contribution and equation for total absorption. *Journal of the Acoustical Society of America*, 72(6), 1879–1890. https://doi.org/10.1121/1.388673
12. Frost, V. S., Stiles, J. A., Shanmugan, K. S., & Holtzman, J. C. (1982). A model for radar images and its application to adaptive digital filtering of multiplicative noise. *IEEE Transactions on Pattern Analysis and Machine Intelligence*, PAMI-4(2), 157–166. https://doi.org/10.1109/TPAMI.1982.4767223
13. Grabek, J., & Cyganek, B. (2019). Speckle noise filtering in side-scan sonar images based on the Tucker tensor decomposition. *Sensors*, 19(13), 2903. https://doi.org/10.3390/s19132903
14. Hellequin, L., Boucher, J.-M., & Lurton, X. (2003). Processing of high-frequency multibeam echo sounder data for seafloor characterization. *IEEE Journal of Oceanic Engineering*, 28(1), 78–89. https://doi.org/10.1109/JOE.2002.808205
15. Johnson, H. P., & Helferty, M. (1990). The geological interpretation of side-scan sonar. *Reviews of Geophysics*, 28(4), 357–380. https://doi.org/10.1029/RG028i004p00357
16. Kuan, D. T., Sawchuk, A. A., Strand, T. C., & Chavel, P. (1985). Adaptive noise smoothing filter for images with signal-dependent noise. *IEEE Transactions on Pattern Analysis and Machine Intelligence*, PAMI-7(2), 165–177. https://doi.org/10.1109/TPAMI.1985.4767641
17. Lee, J.-S. (1980). Digital image enhancement and noise filtering by use of local statistics. *IEEE Transactions on Pattern Analysis and Machine Intelligence*, PAMI-2(2), 165–168. https://doi.org/10.1109/TPAMI.1980.4766994
18. Lee, J.-S. (1981). Speckle analysis and smoothing of synthetic aperture radar images. *Computer Graphics and Image Processing*, 17(1), 24–32. https://doi.org/10.1016/S0146-664X(81)80005-6
19. Lurton, X. (2010). *An Introduction to Underwater Acoustics: Principles and Applications* (2nd ed.). Springer Praxis. https://doi.org/10.1007/978-3-642-13835-5 — chapter "Sonar signal processing – principles and performance": https://doi.org/10.1007/978-3-642-13835-5_6
20. Perona, P., & Malik, J. (1990). Scale-space and edge detection using anisotropic diffusion. *IEEE Transactions on Pattern Analysis and Machine Intelligence*, 12(7), 629–639. https://doi.org/10.1109/34.56205
21. Pizer, S. M., Amburn, E. P., Austin, J. D., Cromartie, R., Geselowitz, A., Greer, T., ter Haar Romeny, B., Zimmerman, J. B., & Zuiderveld, K. (1987). Adaptive histogram equalization and its variations. *Computer Vision, Graphics, and Image Processing*, 39(3), 355–368. https://doi.org/10.1016/S0734-189X(87)80186-X
22. Reed, S., Petillot, Y., & Bell, J. (2003). An automatic approach to the detection and extraction of mine features in sidescan sonar. *IEEE Journal of Oceanic Engineering*, 28(1), 90–105. https://doi.org/10.1109/JOE.2002.808199
23. Sethuraman, A. V., Sheppard, A., Bagoren, O., Pinnow, C., Anderson, J., Havens, T. C., & Skinner, K. A. (2025). Machine learning for shipwreck segmentation from side scan sonar imagery: Dataset and benchmark. *The International Journal of Robotics Research*. https://doi.org/10.1177/02783649241266853
24. Xu, W., Ling, L., Xie, Y., Zhang, J., & Folkesson, J. (2023). Evaluation of a canonical image representation for sidescan sonar. *OCEANS 2023 – Limerick*. https://doi.org/10.1109/OCEANSLimerick52467.2023.10244293
25. Yan, J., Meng, J., & Zhao, J. (2019). Real-time bottom tracking using side scan sonar data through one-dimensional convolutional neural networks. *Remote Sensing*, 12(1), 37. https://doi.org/10.3390/rs12010037
26. Yan, J., Meng, J., & Zhao, J. (2021). Bottom detection from backscatter data of conventional side scan sonars through 1D-UNet. *Remote Sensing*, 13(5), 1024. https://doi.org/10.3390/rs13051024
27. Ye, X., Yang, H., Li, C., Jia, Y., & Li, P. (2019). A gray scale correction method for side-scan sonar images based on Retinex. *Remote Sensing*, 11(11), 1281. https://doi.org/10.3390/rs11111281
28. Yu, Y., Acton, S. T. (2002). Speckle reducing anisotropic diffusion. *IEEE Transactions on Image Processing*, 11(11), 1260–1270. https://doi.org/10.1109/TIP.2002.804276
29. Yu, F., et al. (2020). Bottom detection method of side-scan sonar image for AUV missions. *Complexity*, 2020, 8890410. https://doi.org/10.1155/2020/8890410
30. Zhao, J., Yan, J., Zhang, H., & Meng, J. (2017). A new radiometric correction method for side-scan sonar images in consideration of seabed sediment variation. *Remote Sensing*, 9(6), 575. https://doi.org/10.3390/rs9060575
31. Zheng, G., Zhang, H., Li, Y., & Zhao, J. (2021). A universal automatic bottom tracking method of side scan sonar data based on semantic segmentation. *Remote Sensing*, 13(10), 1945. https://doi.org/10.3390/rs13101945
32. Zhou, P., Chen, J., Tang, P., Gan, J., & Zhang, H. (2024). A multi-scale fusion strategy for side scan sonar image correction to improve low contrast and noise interference. *Remote Sensing*, 16(10), 1752. https://doi.org/10.3390/rs16101752

Vendor documentation (web, fetched 2026-09-05):

* Cerulean Sonar, *SonarView — Omniscan 450 display controls*, https://docs.ceruleansonar.com/c/sonarview/device-specific-controls/omniscan-450/display-controls
* Cerulean Sonar, *SonarView — Omniscan 450 device controls*, https://docs.ceruleansonar.com/c/sonarview/device-specific-controls/omniscan-450/device-controls
* Cerulean Sonar, *Omniscan 450 application programming interface*, https://docs.ceruleansonar.com/c/omniscan-450/application-programming-interface
* Cerulean Sonar, *Omniscan 450 specifications*, https://docs.ceruleansonar.com/c/omniscan-450/specifications

Project documents: `docs/SONARVIEW_SVLOG_ANALYSIS.md` (acquisition measurements on the
field corpus, §5.1 nadir table), `docs/HANDOVER.md`, and the simulator's
`docs/sonar_model.md` (BlueBoat-SSS-Sim) for the modelled range response.
