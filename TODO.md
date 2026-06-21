## Bugs

### Still getting caught in a lot of trimesh geometry

### Hitscan weapons hit the attacking player

## Known fixed-point (fixed16) limitations

### rounded × large convex_solid can still lose contact
sphere/capsule × box now use an analytic closest point (clamp / segment-vs-box)
that is exact at any box size, but sphere/capsule × convex_solid still go through
GJK. GJK reconstructs the small contact vector as a weighted average of the hull's
far vertices (v = Σ wᵢ·yᵢ), which fixed16 loses to cancellation on a *large*
convex hull — the same failure the box pairs had before the analytic path.
`gjk_fit_scale` keeps the simplex products from overflowing but does not cure the
reconstruction cancellation. Convex_solids are usually small brush geometry, so
this rarely bites; a proper fix needs either an analytic closest-point-on-convex
(GJK-class — no trivial clamp like a box has) or a cancellation-free GJK
reconstruction. Suspected sibling, unverified: `for_each_convex_solid_vertex`'s
Cramer-rule triple products may also overflow fixed16 for a very large convex_solid.

### world-coordinate range
Q16.16 saturates at ±32768, and any squared length overflows once a coordinate
passes ~180 units (180² ≈ 32400). Far-from-origin play and large levels are
inherently risky under fixed16 — keep simulated coordinates within a few hundred
units of the origin (or move to fixed32 for larger worlds).
