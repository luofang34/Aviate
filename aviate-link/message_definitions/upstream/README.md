# Vendored upstream MAVLink definitions

`common.xml`, `standard.xml`, and `minimal.xml` are vendored verbatim
from mavlink/mavlink commit f9cb1f9e482977446c3dd953a16368d4834c6aa5
(`message_definitions/v1.0/`). Vendoring keeps dialect generation
reproducible and offline: `aviate.xml`'s `<include>common.xml</include>`
always resolves to exactly these files.

The definitions and the generator are pinned separately. The codec
implements extension fields (SET_ATTITUDE_TARGET `thrust_body`,
SYS_STATUS `onboard_control_sensors_*_extended`) that exist upstream but
not in the definitions snapshot bundled with any released pymavlink, so
the definitions come from the upstream repo while
`scripts/generate_mavlink_dialect.sh` pins the generator tool version.

Do not edit these files. To move to a newer upstream, replace all three
files from one mavlink/mavlink commit, record that commit hash here,
rerun the generator with `--check`, and regenerate the golden vectors in
the codec tests in the same commit.

MAVLink message definition XML files are released under the MIT license
(see https://mavlink.io/en/#license).
