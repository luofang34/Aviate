#!/usr/bin/env python3
"""Generate aviate-geomag/src/coefficients.rs from a WMM.COF file.

Usage (inside a uv venv; stdlib only):
    python3 scripts/generate_wmm_coefficients.py WMM.COF > aviate-geomag/src/coefficients.rs

WMM.COF is the coefficient file of the official World Magnetic Model
release (https://www.ncei.noaa.gov/products/world-magnetic-model). The
generated module records the file header and sha256, so the table can be
traced to the release it came from.
"""

import hashlib
import sys

src = sys.argv[1]
raw = open(src, "rb").read()
sha = hashlib.sha256(raw).hexdigest()
lines=raw.decode().splitlines()
header=lines[0].split()
rows=[]
for l in lines[1:]:
    p=l.split()
    if len(p)!=6: break
    rows.append(p)
out=[]
out.append("//! WMM2025 Gauss coefficients, as published.")
out.append("//!")
out.append("//! Generated from the NOAA NCEI `WMM.COF` file of the WMM2025")
out.append("//! release (header `%s`), sha256" % ' '.join(lines[0].split()))
out.append("//! `%s`." % sha)
out.append("//! Source: https://www.ncei.noaa.gov/products/world-magnetic-model")
out.append("//! (WMM2025COF.zip). The World Magnetic Model is a product of the US")
out.append("//! National Geospatial-Intelligence Agency and the UK Defence Geographic")
out.append("//! Centre, produced by NOAA NCEI and the British Geological Survey; it is")
out.append("//! in the public domain.")
out.append("")
out.append("/// Model epoch, decimal year.")
out.append("pub(crate) const EPOCH: f64 = %s;" % header[0])
out.append("")
out.append("/// Maximum spherical-harmonic degree.")
out.append("pub(crate) const MAX_DEGREE: usize = 12;")
out.append("")
out.append("/// `(n, m, g, h, g_dot, h_dot)`: Schmidt semi-normalized coefficients in")
out.append("/// nT and their secular variation in nT/year.")
out.append("#[rustfmt::skip]")
out.append("pub(crate) const COEFFICIENTS: [(usize, usize, f64, f64, f64, f64); %d] = [" % len(rows))
for n,m,g,h,gd,hd in rows:
    out.append("    (%s, %s, %s, %s, %s, %s)," % (n,m,g,h,gd,hd))
out.append("];")
print("\n".join(out))
