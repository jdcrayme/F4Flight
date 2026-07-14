# f4flight tools

Standalone command-line utilities that ship with the library. None are
linked into f4flight itself; each is a thin `main()` over the public API.

## dat2json — convert Falcon 4 `.dat` to f4flight JSON

```bash
dat2json <input.dat> <output.json>
```

Parses a BMS/Falcon 4 `.dat` aircraft file and writes the equivalent JSON.
Captures every key/value pair from the `.dat` AuxAeroData section verbatim
into `rawAuxAeroData`, plus the `aeropt`/`engopt` option flags and the
`# Title` / `# Author` / `# Revision` header comments.

Exit codes: `0` OK, `2` parse error, `4` AFM-skip (BMS Advanced Flight Model
files use a different schema and are not supported).

## dat_validate — validate a JSON aircraft file

```bash
dat_validate <input.json>                    # print full report
dat_validate <input.json> --quiet            # only print on failure
dat_validate <input.json> --strict           # treat warnings as errors
```

Loads a JSON and runs `AircraftConfig::validate()`, which checks aero table
dimensions, engine table dimensions, geometry sanity, AOA/beta limits, G
limits, NaN/Inf in critical fields, and gear point validity. Reports every
problem found in one pass (does not short-circuit on the first error).

Exit codes: `0` passed, `1` validation failed, `2` could not load file.

## json_diff — field-by-field diff of two JSON aircraft files

```bash
json_diff <old.json> <new.json>                          # full diff
json_diff <old.json> <new.json> --summary                # counts only
json_diff <old.json> <new.json> --threshold 1e-9         # numeric tolerance
```

Loads both JSONs into `AircraftConfig` structs and compares every field:
geometry, aux, aero tables, engine tables, roll command, limiters, gear
points, `rawAuxAeroData` (as a map diff), `aeroOptions`/`engineOptions`
vectors, and source metadata. The comparison is structural, so whitespace
and key-order differences in the JSON source are ignored.

Exit codes: `0` identical, `1` differ, `2` could not load one or both files.

## trace2html — interactive HTML flight-path report

```bash
trace2html <trace.json> -o <report.html>                 # single trace
trace2html traces/*.json -o report.html                  # multi-trace dashboard
trace2html trace.json -o report.html --open              # generate + open
```

Converts one or more trace JSON files (produced by the scenario runner with
`--trace-dir`, or by any tool that writes the trace format) into a single
self-contained `.html` file with an interactive flight-path viewer.

The report is a single-page app with two views:

- **Index dashboard** — a grid of cards, one per trace, each with a mini
  top-down flight-path thumbnail, aircraft/scenario name, phase count,
  duration, and a PASS/FAIL/MIXED badge. Filter by text or status.
- **Detail view** (click a card) — a large top-down flight path beside a
  three-panel time series (altitude, speed, G-load vs time), with:
  - A time scrubber + play/pause (0.25×–4×) to step through the flight.
  - A "color by" selector: phase / pass-fail / altitude / speed / G-load /
    AI mode. Gradient modes draw per-segment coloring; phase mode draws
    one color per phase.
  - Phase chips along the top — click to jump the playhead to that phase.
  - A live frame readout (time, altitude, VCAS, G, bank, pitch, heading,
    throttle, AI mode, phase) that updates as you scrub.

All trace data is embedded inline as compact JSON inside the HTML — no web
server, no external files, no network. The file is shareable and opens via
`file://` in any browser. Traces are downsampled to ≤3000 frames each
(preserving phase boundaries) so even 10-minute scenarios render quickly.

`--open` launches the default browser (`xdg-open` / `open` / `start`).

Exit codes: `0` OK, `1` usage error, `2` could not read a trace, `3` write error.

## trace2svg — static 2-panel SVG

```bash
trace2svg <trace.json> <output.svg>          # one SVG with all phases
trace2svg <trace.json> <output.svg> --split  # one SVG per phase
```

Produces two panels per SVG: a top-down ground track and a side profile
(altitude + speed vs time). Use this when you need a static image for
embedding in docs, PRs, or slide decks. For interactive inspection prefer
`trace2html`.

## Recording traces from the scenario runner

The scenario runner (`f4flight_flight_scenarios` / `f4flight_digi_scenarios`)
can record a trace per scenario and emit an HTML report in one step:

```bash
# Run all scenarios for the F-16 and open a report in the browser:
f4flight_digi_scenarios tests/fixtures/aircraft/fighters/f16bk50.json \
    --html report.html --open

# Run one scenario, write the raw trace JSON too:
f4flight_digi_scenarios f16bk50.json --scenario ai_basic \
    --trace-dir traces/ --html report.html

# Just open a report (writes to ./f4flight_report.html):
f4flight_digi_scenarios f16bk50.json --open
```

Traces are **never** written unless `--trace-dir` or `--html` is given, so
normal `ctest` runs stay clean. The `--html` report embeds all traces from
the run inline; `--trace-dir` additionally writes per-scenario JSON files
that you can re-process later with `trace2html` or `trace2svg`.

## Building

The tools build as part of the normal cmake build:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
# tools are at build/dat2json, build/dat_validate, build/json_diff,
#          build/trace2html, build/trace2svg
```

For a quick build without cmake (g++ only), see
`scripts/build_dat2json.sh` in the project root.
