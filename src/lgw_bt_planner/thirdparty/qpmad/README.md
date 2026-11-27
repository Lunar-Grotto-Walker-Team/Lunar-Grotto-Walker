# qpmad (vendored)

Header-only QP solver, Apache-2.0. Vendored so the container needs no
network and no extra build dependency (only Eigen, already required).

- Upstream: https://github.com/asherikov/qpmad
- Commit: 11602ec9f87a62a97ba8318ebd81da6646f3c631
- License: Apache-2.0 (see LICENSE); NOTICE retained.
- `include/qpmad/config.h` is the generated header for an all-options-OFF
  build (tracing off, Householder off, pedantic license off).

Do not edit the headers; re-vendor from upstream when upgrading.
