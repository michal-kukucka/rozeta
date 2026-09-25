# Monitors

`rozeta/monitors.hpp` holds conditions that cannot be decided from one tick. Each monitor answers two
questions — *is this true?* and *is it newly worth saying?* — and nothing more. None of them commands
motion: the caller decides whether a condition stops the robot, warns the operator, or is only logged.

The split with the application is deliberate. The timing and the once-per-episode bookkeeping, which every
robot gets wrong the same way, live here. The thresholds that come from a rulebook (one metre, one minute),
where an area comes from, and the words an operator reads belong to the application.

| type | answers | reports |
|------|---------|---------|
| `monitors::GeoRect` | how far inside a latitude/longitude rectangle a position is, signed | — |
| `monitors::BoundaryWatch` | inside, near the edge, or outside | leaving once; each approach within the warning margin once |
| `monitors::HeldCondition` | has a condition held continuously for `hold_s`? | once per episode; a condition that clears and returns has happened twice |
| `monitors::StallWatch` | is a robot that is trying to drive still inside a circle after `seconds`? | once per stall; not driving clears it |

## GeoRect and BoundaryWatch

`GeoRect::marginM` is positive inside and negative outside, and outside a corner it is the diagonal
distance to that corner. A signed margin rather than a bare in/out, because the useful question while
driving is how much room is left: stopping a few metres inside ends nothing, while being stopped at the
edge may end the attempt.

`BoundaryWatch` built without an area treats every position as inside with a margin of 0 — having no
geofence is not an offence. With one, `check()` returns a `BoundaryVerdict` whose `report` flag is set on
the first tick outside and on the first tick of each approach inside the warning margin. The approach
warning re-arms only once the robot is back beyond the margin, so a path that runs along the edge does not
fill a log.

## HeldCondition

A single sample beyond a line is noise, not a robot on the grass. `HeldCondition::update(now, active)`
returns true once, on the first update at which `active` has held for at least `hold_s`; clearing resets
both the timer and the report.

## StallWatch

`StallWatch` anchors a circle of `radius_m` where the robot was when it last left the previous one
(great-circle metres, `geodesy::haversineDistance`). It reports once when a robot that is driving has not
left the circle for `seconds`. Not driving clears everything: waiting on purpose is not being stuck.

## C ABI

`rozeta_geo_rect_valid`, `rozeta_geo_rect_margin_m`, `rozeta_boundary_watch_create/check/closest/destroy`,
`rozeta_held_condition_create/update/state/reset/destroy` and
`rozeta_stall_watch_create/update/state/reset/destroy` expose the same objects to ctypes and other FFI
callers. See `docs/api-reference.md`.
