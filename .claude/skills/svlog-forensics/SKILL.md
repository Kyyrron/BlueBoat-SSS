---
name: svlog-forensics
description: Forensic analysis of a recorded .svlog side-scan sonar log — packet census, session segmentation, per-channel acquisition parameters, PRI and ping-number gap statistics, src-vs-channel_number consistency, FBR altitude distribution, and a before/after waterfall pair. Use whenever a .svlog needs to be characterised, compared against another log, or checked for mis-tagging, ping loss, a mid-file settings change or a session gap.
---

# `.svlog` forensics

Run the analyser. Do not re-derive any of this by hand — that is the workflow this
skill exists to replace.

```bash
cd BlueBoat-SSS/blueboat_sss          # the parent of the blueboat_gcs package
python3 -m blueboat_gcs.analysis.svlog_forensics --out ~/svlog_reports LOG.svlog
```

- Several files, or a directory (searched recursively): pass them all, and add
  `--compare` for a cross-file table.
- `--no-images` skips the waterfall pair and is roughly 4× faster. Use it when only
  the numbers are wanted.
- Needs numpy + opencv only. **No ROS, no display**, so it runs against any log
  anywhere.

Per input it writes `<out>/<stem>/report.md` plus `waterfall_slant.png` and
`waterfall_ground.png`. Reports are deterministic apart from one `*Generated:*` line,
and the PNGs are byte-stable.

## Rules

**Never write into the log's own directory tree.** Recorded `.svlog` files are primary
field data (`CLAUDE.md` NC #6 / root CM-7). The tool refuses an `--out` that resolves
anywhere inside a tree holding `.svlog` files; if it refuses, pick a different `--out`
rather than working around it.

**Never route a ping by the `src` byte.** Side identity comes from the packet:
`channel_number` (byte 34), falling back to the sign of `transducer_heading_deg`
(byte 52). `src` is a *metric* here, not an input — see below.

**Report the mode, and say so.** Several field logs change range mid-survey. Any
single-value summary of such a file is quoting the modal value; the report flags it and
the comparison table footnotes it.

## Reading the output

| Number | What it means | Field reference |
|---|---|---|
| **wrong-`src` %** | Frames whose `src` tag disagrees with the packet's own side. Non-zero means SonarView, and any consumer routing by the tag, renders this log mirrored or swapped. | 0 % on clean logs; 10.6 / 19.8 / 29.5 % on three of ours; **~50 % (uncorrelated)** on the two `channel_number = 255` logs |
| **missing ping numbers**, per channel per segment | Real acquisition loss upstream of the recorder. Always read per segment — the counter restarts at a session boundary. | 7.7 % port / 1.8 % starboard on the 80 m sea trial; 0 on the SonarView and Cerulean references |
| **counter offset** | Constant difference between the two devices' independent ping counters. Any consumer assembling dual-channel rows must normalise by it first. | 0 on ten corpus logs, −1 on four, +60 on two; confidence ≥ 0.94 on every two-sided log |
| **FBR p10 / p50 / p90** | Raw per-ping bottom detections. A wide spread means slant-range correction is unreliable on this log and `Depth comp. = off` is the safer display choice. | 4.67 / 6.54 / 15.23 m on the 80 m log — the spread that makes `off` better there |
| **bottom at sample `k/N`** | How close the bottom return sits to the transmit ringing. Below ~50 the FBR detector cannot find it at all. | 49/600 at 80 m (undetectable), 269/600 at 20 m, 137/1200 at 25.4 m |
| **sessions / gaps** | Multiple id-10 headers mean a segmented file. | the Cerulean demo is 2 sessions with a 397.8 s gap inside a 611.5 s span |
| **`gain_index` transition %** | Whether auto-gain is a real contributor to banding — the transition *rate*, not the number of distinct values. | 0.6–0.9 % on our logs, i.e. not a contributor |

`timestamp_ms` runs backwards in file order on most logs because the writer batches by
channel; the report states the count. All timing is computed on stamps sorted per
channel, so that is expected, not a defect.

## Where the numbers came from

`blueboat_gcs/docs/SONARVIEW_SVLOG_ANALYSIS.md` holds the full analysis the reference
column reproduces. The three files behind `CLAUDE.md`'s *Measured acquisition settings*
table are:

| Column | File |
|---|---|
| our sea trial (80 m) | `ShiraishiJima/MainReflectionEvidence+misspingsWaterfall/2026-07-23-11-45-00.svlog` |
| SonarView (20 m) | `ShiraishiJima/diffDepthCompensation.svlog` |
| Cerulean demo (25.4 m) | `harbor_scan_combined.svlog` |

`tests/test_svlog_forensics.py` pins those numbers, so a regression in the analyser
fails the suite rather than producing a quietly wrong report.

## When a file will not analyse

The tool exits 2 with a message rather than half-reporting. Empty file, no `BR` framing
at all, or framed packets with no decodable `OS_MONO_PROFILE` are all refusals. A file
that is merely *truncated* still analyses, with a warning naming the trailing byte
count.
