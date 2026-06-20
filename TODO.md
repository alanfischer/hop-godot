## Bugs

### Still getting caught in a lot of trimesh geometry

### Hitscan weapons hit the attacking player

### fixed16: a sphere sliding across a box top tunnels through it

hop core narrowphase precision bug (not rotation-specific). A finite-mass
sphere given a horizontal velocity while resting on an `aa_box` *top face*
loses the contact and falls through under `fixed16` — reproduces with zero
friction and no angular velocity, so it is independent of the Phase 6 carry.
The same setup holds fine in `float`, and a sphere sliding on a *sphere*
floor holds in `fixed16`. Surfaced while testing kinematic angular carry
(`test_angular_carry` uses a sphere platform to dodge it). Likely in the
sphere×box GJK/sweep path losing the top-face contact as the contact point
slides; needs a fixed-point audit of that narrowphase.
