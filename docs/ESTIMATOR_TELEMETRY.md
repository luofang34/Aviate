# Estimator telemetry contract

A telemetry frame timestamp establishes transport recency only. A recent frame
with finite attitude, position, or velocity values is not evidence that the
estimator considers those values usable. Consumers must evaluate estimator
status independently of frame freshness.

`ATTITUDE_QUATERNION` and `LOCAL_POSITION_NED` continue to carry the latest
numeric estimate even when it is degraded or unusable. This preserves
observability while the status frames determine whether each dimension may be
used.

## Status frames

On the cycle-formatter path — the path GCS links use — Aviate emits both
status frames at least at `estimator_status_hz`, and immediately before every
attitude or position snapshot, so each numeric frame has status with the same
timestamp. The raw `TelemetryBackend::send_state` API bypasses this pairing
and is not suitable for consumers applying the fail-closed rules below:

- Standard MAVLink `ESTIMATOR_STATUS` (message 230) provides a conservative
  projection for common MAVLink consumers.
- `AVIATE_ESTIMATOR_STATUS` (message 20000) carries `EstimateQuality` and the
  complete per-dimension validity as `AVIATE_STATE_VALID_FLAGS` without loss.
  Its canonical definition is `aviate-link/message_definitions/aviate.xml`;
  the Rust side maps each internal flag onto the wire bitmask explicitly and
  never emits internal bit representations directly. The message does not
  duplicate the standard-flag projection; consumers needing standard
  semantics read `ESTIMATOR_STATUS`.

The standard projection is deliberately narrower than Aviate's state model:

| Aviate validity | Standard flag projection |
|---|---|
| `ATTITUDE` | `ESTIMATOR_ATTITUDE` |
| `VELOCITY` | `ESTIMATOR_VELOCITY_HORIZ`, `ESTIMATOR_VELOCITY_VERT` |
| `POSITION` | `ESTIMATOR_POS_HORIZ_REL` |
| `ANGULAR_RATE` | none; read the Aviate message |

Local vertical-position validity has no exact standard flag and remains only
in the Aviate bitmap. The standard absolute-height and above-ground flags are
not substitutes for local NED vertical position.

Only `Good` estimates set standard validity flags because the standard enum
describes those outputs as good. `Degraded` and `Unusable` clear every standard
validity flag rather than overstating their quality. The Aviate message still
carries the raw validity bitmap for diagnosis. Consumers must not use any
dimension while quality is `Unusable`; for `Good` or `Degraded`, they may use
only dimensions whose Aviate validity bits are set.

Innovation ratios and accuracy fields in standard `ESTIMATOR_STATUS` are NaN
until Aviate exports those values. Consumers must not infer estimator quality
from unavailable ratio or accuracy fields.

The queue admits a same-timestamp status and numeric group together or drops the
whole group under backpressure. Consumers must match status and numeric frames
by timestamp and must not carry validity forward to a different timestamp.

## Consumer fail-closed rules

`AVIATE_ESTIMATOR_STATUS` is the authorization source for using numeric
telemetry. A consumer must treat every dimension as `Unusable` when any of
the following holds:

- no status frame exists for the timestamp of a numeric frame,
- the newest status frame is older than the newest numeric frame,
- the `quality` value is unknown to the consumer,
- the status frame fails to parse.

Bits in `valid_flags` outside the `AVIATE_STATE_VALID_FLAGS` definition
never authorize anything: a consumer masks to the defined bits and
ignores the rest, so a newer sender cannot accidentally authorize a
dimension an older consumer knows under a different bit.

A newer status supersedes all older telemetry immediately. On receiving
`Unusable`, or a status with a validity bit cleared, the consumer
revokes authorization for the affected dimensions across every
previously received numeric frame — it does not wait for a numeric
frame with a matching timestamp. Aviate emits the status pair at
`estimator_status_hz` even when no numeric stream is due in that cycle,
so revocation never depends on a coincident numeric emission.

Standard `ESTIMATOR_STATUS` remains a conservative projection for common
consumers and must not be used as the lossless authorization source.

## Heading reference frame

Once per heartbeat, Aviate sends `AVIATE_GEOMAGNETIC_REFERENCE`
(message 20001). It states the declination that the estimator adds to
the magnetometer heading, and the source of that declination:

- `AVIATE_GEOMAGNETIC_SOURCE_MODEL`: a geomagnetic model at the
  vehicle's position and date.
- `AVIATE_GEOMAGNETIC_SOURCE_CONFIGURED`: a configured value for the
  operating area.
- `AVIATE_GEOMAGNETIC_SOURCE_NONE`: no declination. The declination
  field is NaN.

With a model or a configured declination, the heading in
`ATTITUDE_QUATERNION` is referenced to true north. With source NONE, it
is magnetic. A consumer that shows a magnetic heading subtracts the
declination from the true heading.

## Global position frames

At the `position_hz` rate, the cycle formatter also sends two standard
frames:

- `GLOBAL_POSITION_INT` (message 33) gives the local estimate as a
  global position. Aviate sends it only when the vehicle has a declared
  geodetic origin, which is the point that the local NED frame is
  measured from. `lat`, `lon` and `alt` are the local NED position
  projected on the WGS-84 tangent plane at the origin
  (`world_core::LocalTangentPlane`). The origin's sea-level altitude is
  taken as its ellipsoidal height, with no geoid separation applied.
  `relative_alt` is the height above the local origin. This frame is part of the estimate group: it has the same
  timestamp as `LOCAL_POSITION_NED`, and the status rules above apply to
  it in the same way.
- `GPS_RAW_INT` (message 24) gives the last fix that the GNSS receiver
  reported. It is a sensor report, not an estimate, so it is not part of
  the estimate group. The receiver reading has one height and no geoid
  separation, so Aviate sends that height as `alt` and as
  `alt_ellipsoid`. DOP is not part of the reading, so `eph` and `epv`
  are `UINT16_MAX`. The 1-sigma accuracies are in `h_acc` and `v_acc`.

## Wire contract stability

The Aviate dialect is a private dialect in the sense of the MAVLink
dialect guidance: message id 20000 is self-assigned and no upstream range
has been requested. Until Aviate declares the dialect stable, the message
layout may change between releases and each change updates the golden
vectors generated from `aviate.xml`. Publishing the dialect requires
requesting a dedicated message-id range upstream first.
