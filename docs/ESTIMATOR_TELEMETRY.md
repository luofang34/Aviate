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

Aviate emits both status frames at least at `estimator_status_hz`. It also emits
them immediately before every attitude or position snapshot, so each numeric
frame has status with the same timestamp:

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

Standard `ESTIMATOR_STATUS` remains a conservative projection for common
consumers and must not be used as the lossless authorization source.

## Wire contract stability

The Aviate dialect is a private dialect in the sense of the MAVLink
dialect guidance: message id 20000 is self-assigned and no upstream range
has been requested. Until Aviate declares the dialect stable, the message
layout may change between releases and each change updates the golden
vectors generated from `aviate.xml`. Publishing the dialect requires
requesting a dedicated message-id range upstream first.
