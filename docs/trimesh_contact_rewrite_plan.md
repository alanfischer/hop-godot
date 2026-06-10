# Trimesh contact rewrite — plan & findings

Status: **IMPLEMENTED** (offline-validated; pending in-game playtest). The capsule
query paths in `HopTrimeshTraceable::trace_solid` now do real capsule-segment-vs-
triangle closest-point contact as designed below. Non-capsule shapes (box/sphere,
e.g. area queries) keep the older winding-agnostic support-plane approximation.

Offline validation (harness `/tmp/htest`, real ww_2fort BSP collision):
- **Basement replay** (`replay.cpp`): recovery now pushes **+0.901 up**
  (game previously saw `recover.y = -1.810`, i.e. shoved down through the floor).
- **Ramp climb** (`ramp2fort.cpp -20 14.2 50.4`): reaches the top (`x≈-27.0,
  y≈19.8`), crossing the seam at `x≈-24.3` smoothly — no fall-through/stick.
- **Map sweep** (`bstress.cpp`): **0 void-falls** across 3880 walks at 12/24/40/
  **80** m/s (the distance-based conservative-advancement cast is tunnel-proof,
  so no `seam_tol_` hack is needed on the capsule path; stuck count ~ unchanged
  vs the prior baseline and dominated by genuine walls at the sweep boundary).
- **Unit test** (`tests/test_trimesh_traceable.cpp`): rewritten to pin the new
  one-sided/closest-point invariants (front-face recovery, mixed-winding surface,
  deep-sink-pushes-up regression, two-sided swept cast, resting-allows-horizontal-
  motion, capsule-gap-fit, ramp climb) — all pass via ctest.

All 6 dylib variants rebuilt via `./build_hop.sh`. In-game playtest still TODO
(strafe the blue basement resupply door; re-walk ramps across maps).

---

Original plan & diagnosis below (kept for reference).

The committed branch `fix/trimesh-winding-agnostic-collision`
made player-vs-trimesh "much better" (ramps mostly walkable) but two classes of
bug remain, and they can't be fixed by patching the current approximation. This
doc captures the full diagnosis so the rewrite can start cold.

## The two remaining bugs

1. **Basement fall-through (blue_resupply_3 doorway, ~godot (-21.46, -14.15, 87.56)).**
   While **strafing**, the player sinks ~0.9 m into a flat floor, then the
   depenetration (recovery) shoves them **down** through it. Captured live via the
   `WW_LOG_FALL` logger:
   `recover=(-0.331, -1.810, 0)` — i.e. a 1.81 m **downward** push.
2. **Stuck/▒fall on some ledges** (a different, unlocated spot).

Godot's own physics handles this same map/mesh fine — so the geometry is fine and
this is a hop-side contact bug.

## Root cause (validated)

hop's trimesh contact (`hop_trimesh_traceable.h`, `trace_solid`) is an
**approximation**: it expands each triangle into an **infinite plane** (offset by
the shape's `support()` extent) and tests the **body center** against that plane,
pushing along the triangle's face normal. This breaks in two compounding ways:

- **Direction flips when the body sinks.** The committed code orients the normal
  "toward the body center." Once the center crosses below a floor's plane (sunk
  more than ~0), "toward the body" points **down**, so depenetration pushes the
  player *through* the floor. That is the basement `-1.81`. (Reverting orient to a
  plain stored normal fixes the *direction* — see below.)
- **Mixed winding → one-sided holes.** The BSP collision triangles are **not**
  consistently wound. On the z≈50.4 ramp, adjacent walkable faces alternate
  up/down (`hop stored normal` is `(+.64,+.77,0)` on some tris, `(-.64,-.77,0)` on
  neighbours — verified). A **flat floor** like the basement is wound correctly
  (all `(0,+1,0)`). So:
  - **One-sided** collision (push out only the front face) is correct for the
    basement, but on the ramp it leaves **holes** (the down-wound halves aren't
    solid) and the body sinks through them.
  - **Two-sided** collision (the committed orient-to-body) covers the ramp (no
    holes) but flips the depenetration direction wrong in the basement.

These two requirements are contradictory **for the center+plane approximation**.
Every patch (one-sided, two-sided, min-depth vs deepest, gravity/up bias, winding
correction at source) trades the ramp for the basement or vice-versa. The
approximation is the ceiling.

### What is actually correct (and what Godot does)

Godot's capsule-vs-`ConcavePolygonShape3D` is **one-sided** (it sets no
`backface_collision`, default false) and uses **real capsule-segment-vs-triangle
geometry** (closest point between the capsule spine and the *finite* triangle).
Two properties make it robust where hop isn't:

- **Depenetration pushes out along the FRONT face normal**, by the true
  penetration, for any capsule overlapping a triangle's front — *regardless of how
  deep the center is*. It never flips. (Confirmed: a "one-sided recovery along the
  stored normal" gives the correct **+0.90 up** at the exact basement state, and it
  also climbs the ramp.)
- **Accurate swept contact keeps the body from sinking in the first place**, so the
  recovery only ever sees shallow penetration. hop's approximate cast is what lets
  the player sink ~0.9 m, after which even a correct recovery is in the ambiguous
  "deeply straddling a one-sided face" regime.

So the fix is **not** winding correction and **not** a gravity hack. It's to
replace the center+plane approximation with **proper capsule-segment-vs-triangle
closest-point contact** in both query paths.

## Target design

Rewrite the two query paths in `HopTrimeshTraceable::trace_solid`
(`extern/hop-godot/src/hop_trimesh_traceable.h`) to do real shape-vs-triangle
contact. The player is a **capsule** (segment spine + radius); handle that
exactly. Keep a reasonable fallback for box/sphere shapes (most callers are the
capsule player; boxes/spheres can keep the support-plane path or get their own
closest-point later).

### Primitives to add (Ericson, *Real-Time Collision Detection*)
- `closest_pt_point_triangle(p, a,b,c)` — already effectively present inside
  `point_near_triangle`; factor it out to return the closest point + barycentric
  region (face / edge / vertex).
- `closest_pt_segment_segment(p1,q1, p2,q2)` — for capsule-spine vs triangle-edge.
- `closest_seg_triangle(p,q, a,b,c) -> {dist2, cs (on segment), ct (on triangle)}`
  — min over: each spine endpoint clamped to the triangle, and the spine vs each of
  the 3 edges; plus the spine-crosses-triangle (dist 0) case.

### Recovery (zero-direction / static overlap)
For each candidate triangle (BVH `query_aabb`):
1. Compute `closest_seg_triangle` between the capsule spine and the triangle →
   `d`, `cs`, `ct`.
2. If `d >= radius` → not touching, skip.
3. Penetration `depth = radius - d`.
4. **One-sided front gate** (Godot rule): only accept if the capsule is on the
   **front** of the face — i.e. `dot(faceNormal, cs - ct) >= 0` (capsule axis on the
   +normal side) OR, for the degenerate spine-crosses case (`d≈0`), accept and use
   the **face normal** as the push direction. Reject back-face contacts (this is
   what excludes the inverted down-twin on mixed-winding surfaces).
5. Contact normal: `faceNormal` for a face contact; `(cs-ct)/d` for an edge/vertex
   contact (Voronoi region), still gated to front.
6. Accumulate **minimum-translation** depenetration (nearest surface first; track
   best/`min depth`). Output `result.time=0, result.depth=depth, result.normal,
   result.point`.

This is validated in spirit: "one-sided along the stored face normal + min depth"
already produced `+0.90` up at the basement. The closest-point version generalizes
it and removes the center-flip.

### Swept cast
The cast must **stop the capsule at the surface from either side it approaches**
(so the ramp has no holes) while reporting a normal that opposes motion. Options,
simplest-first:
- **Conservative advancement / TOI**: step the capsule along the motion; at each
  step do the static `closest_seg_triangle`; find the first `t` where `d` reaches
  `radius`. Report that TOI + the contact normal. This is robust and not winding-
  sensitive (it's distance-based, two-sided by nature), so it covers the mixed-
  winding ramp without holes.
- Keep it cheap: the existing swept AABB BVH query already bounds the candidate
  triangles; CA only needs a few iterations for the per-frame motion sizes here.

Because the cast now keeps the body on the surface, the recovery only ever sees
shallow penetration (the clean case).

### Then revert the workarounds
Once the above works, delete from `hop_trimesh_traceable.h`:
- the orient-to-body normal flip (recovery **and** cast),
- the up-bias / gravity experiments (never committed, but make sure none linger),
- keep `point_near_triangle` (its closest-point math is reused).
No `goldsrc-godot` winding change is needed. No gravity assumption.

## Validation (must pass before committing)

Offline harness in `/tmp/htest` (rebuildable — see below). The reproduction is
faithful: it loads the **real ww_2fort BSP collision** and runs the **real**
`HopTrimeshTraceable` through a port of `_body_test_motion` + `move_and_slide`.

1. **Basement replay** (`replay.cpp`): capsule at `(-21.45783, -14.14961, 87.55874)`
   → recovery must push **up** (`recover.y > 0`, ~+0.9), never down.
2. **Ramp climb** (`ramp2fort.cpp -20.0 14.2 50.4`): must reach the top (`x≈-27`,
   `y≈19.7`) without falling through — especially across the **seam at x≈-24.3**.
3. **Map sweep** (`bstress.cpp` / `stress2.cpp`): 0 void-falls across thousands of
   walks (incl. basement region, diagonals, up to ~40 m/s).
4. **Unit test**: `extern/hop-godot/tests/test_trimesh_traceable.cpp` (ctest) still
   passes; extend it with a capsule-resting-sunk case (center on/just below the
   floor plane → recovery normal up, small depth) and a mixed-winding floor.
5. **In-game**: rebuild all variants (`./build_hop.sh`), play ww_2fort, strafe the
   blue basement resupply door — no fall-through; re-walk ramps.

## Reproduction harness (how to rebuild `/tmp/htest`)

Everything is offline; no Godot run needed.

- **Extract the real collision mesh** (replicates `goldsrc_bsp.cpp
  build_hull_collision` exactly): compile a small tool against the goldsrc BSP
  parser, which has **zero godot deps**:
  `clang++ -std=c++17 -I extern/goldsrc-godot/src/parsers extract.cpp \
   extern/goldsrc-godot/src/parsers/bsp_parser.cpp`.
  Parse `maps/ww_2fort.bsp`, for worldspawn faces (`model_index==0`, skip `!`/`*`/
  sky): fan-triangulate, transform `goldsrc_to_godot(x,y,z)=(-x*.025, z*.025,
  y*.025)`, emit triangles. hop builds them as `{i0,i2,i1}` (the winding reversal in
  `hop_shape_data.cpp`). Dump to `2fort_tris.txt` (`N` then `N*3` `x y z` lines).
  (Use ALL `model_index` for the full collision incl. brush entities → the basement
  spot needs those; `2fort_tris_all.txt`.)
- **Drive the real trimesh**: the test `.cpp`s include the real
  `hop_trimesh_traceable.h` and need a 1-line stub `hop_conversions.h` (`#pragma
  once`) on the include path so it compiles against hop core only
  (`-I extern/hop-godot/extern/hop/include`). They construct a `hop::simulator`, a
  static solid carrying `HopTrimeshTraceable<float>`, and a capsule player solid
  (spine `(0,-0.5,0)..(0,0.5,0)`, r=0.4 = the real player), then port
  `_body_test_motion` (recovery loop + swept cast) + a simple `move_and_slide`.
  Up = +Y, gravity `(0,-9.81,0)`. `hop_scalar = float`.

Key constants: player capsule r=0.4 h=1.8 (CapsuleShape3D); map scale 0.025
(GoldSrc units→m); coords up to ~94 m. The blue basement door is BSP submodel `*98`
at godot x[-25.6,-22.4] y[-14.15,-11.2] z≈86.9.

## Memory pointers

See `memory/project_trimesh_ramp_penetration.md` for the running log of findings.
Related: `project_hop_godot_build_variants.md` (rebuild ALL dylib variants via
`./build_hop.sh`).
