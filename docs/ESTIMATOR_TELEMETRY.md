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

Aviate emits both status frames at `estimator_status_hz`:

- Standard MAVLink `ESTIMATOR_STATUS` (message 230) provides a conservative
  projection for common MAVLink consumers.
- `AVIATE_ESTIMATOR_STATUS` (message 20000) carries `EstimateQuality` and the
  complete `StateValidFlags` bitmap without loss. Its canonical definition is
  `aviate-link/message_definitions/aviate.xml`.

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
