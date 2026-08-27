//! Single-consumer loading for a verified X-Plane runtime binding.

use std::io::Read as _;
use std::path::{Path, PathBuf};

use aviate_config::airframe_preset::ContentDigest;
use aviate_config::xplane_runtime::XPlaneRuntimeHandshake;

use super::CliError;

pub(crate) struct ClaimedRuntimeHandshake {
    pub(crate) handshake: XPlaneRuntimeHandshake,
    pub(crate) content_digest: ContentDigest,
}

pub(super) fn claim(
    path: &Path,
    selected_bridge: std::net::SocketAddr,
) -> Result<ClaimedRuntimeHandshake, CliError> {
    let claim_directory = claim_directory(path)?;
    std::fs::create_dir(&claim_directory).map_err(|source| CliError::ClaimRuntimeBinding {
        path: path.to_owned(),
        source,
    })?;
    let result = claim_while_locked(path, selected_bridge);
    std::fs::remove_dir(&claim_directory).ok();
    result
}

fn claim_while_locked(
    path: &Path,
    selected_bridge: std::net::SocketAddr,
) -> Result<ClaimedRuntimeHandshake, CliError> {
    let link_metadata =
        std::fs::symlink_metadata(path).map_err(|source| CliError::ReadArtifact {
            kind: "runtime handshake",
            path: path.to_owned(),
            source,
        })?;
    if link_metadata.file_type().is_symlink() || !link_metadata.is_file() {
        return Err(CliError::InvalidCombination(
            "--runtime-handshake must name a regular file",
        ));
    }
    let mut file = open_binding(path).map_err(|source| CliError::ReadArtifact {
        kind: "runtime handshake",
        path: path.to_owned(),
        source,
    })?;
    let metadata = file.metadata().map_err(|source| CliError::ReadArtifact {
        kind: "runtime handshake",
        path: path.to_owned(),
        source,
    })?;
    validate_same_file(path, &link_metadata, &metadata)?;
    validate_private_permissions(path, &metadata)?;
    std::fs::remove_file(path).map_err(|source| CliError::ConsumeRuntimeBinding {
        path: path.to_owned(),
        source,
    })?;
    let mut text = String::new();
    file.read_to_string(&mut text)
        .map_err(|source| CliError::ReadArtifact {
            kind: "claimed runtime handshake",
            path: path.to_owned(),
            source,
        })?;
    let handshake =
        XPlaneRuntimeHandshake::from_toml_str(&text).map_err(CliError::InvalidRuntimeBinding)?;
    let declared = handshake
        .bridge_endpoint
        .parse::<std::net::SocketAddr>()
        .map_err(|_| CliError::RuntimeBridgeMismatch {
            declared: handshake.bridge_endpoint.clone(),
            selected: selected_bridge,
        })?;
    if declared != selected_bridge {
        return Err(CliError::RuntimeBridgeMismatch {
            declared: handshake.bridge_endpoint.clone(),
            selected: selected_bridge,
        });
    }
    Ok(ClaimedRuntimeHandshake {
        handshake,
        content_digest: XPlaneRuntimeHandshake::content_digest(&text),
    })
}

#[cfg(unix)]
fn open_binding(path: &Path) -> std::io::Result<std::fs::File> {
    use std::os::unix::fs::OpenOptionsExt as _;

    std::fs::OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NOFOLLOW)
        .open(path)
}

#[cfg(not(unix))]
fn open_binding(path: &Path) -> std::io::Result<std::fs::File> {
    std::fs::File::open(path)
}

#[cfg(unix)]
fn validate_same_file(
    path: &Path,
    path_metadata: &std::fs::Metadata,
    file_metadata: &std::fs::Metadata,
) -> Result<(), CliError> {
    use std::os::unix::fs::MetadataExt as _;

    if path_metadata.dev() == file_metadata.dev() && path_metadata.ino() == file_metadata.ino() {
        Ok(())
    } else {
        Err(CliError::RuntimeBindingChanged(path.to_owned()))
    }
}

#[cfg(not(unix))]
fn validate_same_file(
    path: &Path,
    path_metadata: &std::fs::Metadata,
    file_metadata: &std::fs::Metadata,
) -> Result<(), CliError> {
    if path_metadata.file_type() == file_metadata.file_type()
        && path_metadata.len() == file_metadata.len()
    {
        Ok(())
    } else {
        Err(CliError::RuntimeBindingChanged(path.to_owned()))
    }
}

fn claim_directory(path: &Path) -> Result<PathBuf, CliError> {
    let parent = path.parent().unwrap_or_else(|| Path::new("."));
    let name = path.file_name().ok_or(CliError::InvalidCombination(
        "invalid --runtime-handshake path",
    ))?;
    Ok(parent.join(format!(".{}.claim", name.to_string_lossy())))
}

#[cfg(unix)]
fn validate_private_permissions(path: &Path, metadata: &std::fs::Metadata) -> Result<(), CliError> {
    use std::os::unix::fs::PermissionsExt as _;

    let mode = metadata.permissions().mode() & 0o777;
    if mode & 0o077 == 0 {
        Ok(())
    } else {
        Err(CliError::InsecureRuntimeBinding {
            path: path.to_owned(),
            mode,
        })
    }
}

#[cfg(not(unix))]
fn validate_private_permissions(
    _path: &Path,
    _metadata: &std::fs::Metadata,
) -> Result<(), CliError> {
    Ok(())
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used)]

    use super::*;

    const BINDING: &str = "schema_version = 1\nverifier_id = \"pilotage-xplane-trial-v1\"\nsession_binding_digest = \"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\"\nbridge_endpoint = \"127.0.0.1:4560\"\nbridge_protocol = \"mavlink-hil-tcp-v1\"\nbridge_build_digest = \"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb\"\nbridge_config_digest = \"cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc\"\nsimulator_id = \"xplane-12\"\naircraft_id = \"xplane12-laminar-alia250\"\naircraft_file_digest = \"dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd\"\nsample_rate_hz = 100\nmotor_count = 4\nlane_order = [0, 2, 1, 3]\n";

    #[test]
    fn a_private_runtime_binding_has_one_consumer() {
        let nonce = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .expect("system clock")
            .as_nanos();
        let directory = std::env::temp_dir().join(format!("aviate-binding-{nonce}"));
        std::fs::create_dir(&directory).expect("test directory");
        let path = directory.join("binding.toml");
        std::fs::write(&path, BINDING).expect("binding file");
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt as _;
            std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o600))
                .expect("private permissions");
        }
        let claimed = claim(&path, "127.0.0.1:4560".parse().expect("bridge address"))
            .expect("claimed binding");
        assert_eq!(claimed.handshake.sample_rate_hz, 100);
        assert!(!path.exists());
        assert!(claim(&path, "127.0.0.1:4560".parse().expect("bridge address")).is_err());
        std::fs::remove_dir(directory).ok();
    }
}
