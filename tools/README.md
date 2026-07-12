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

## Building

The tools build as part of the normal cmake build:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
# tools are at build/dat2json, build/dat_validate, build/json_diff
```

For a quick build without cmake (g++ only), see
`scripts/build_dat2json.sh` in the project root.
