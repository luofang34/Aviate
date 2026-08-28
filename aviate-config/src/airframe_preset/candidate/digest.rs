//! Content identity for candidate artifacts.

use core::fmt;

use sha2::{Digest, Sha256};

use super::CandidateError;

/// SHA-256 identity for one exact artifact.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ContentDigest([u8; 32]);

impl ContentDigest {
    /// Calculate the identity of an artifact.
    #[must_use]
    pub fn calculate(bytes: &[u8]) -> Self {
        Self(Sha256::digest(bytes).into())
    }

    /// Parse a lowercase or uppercase hexadecimal identity.
    ///
    /// # Errors
    ///
    /// Returns an error when the text is not one SHA-256 value.
    pub fn from_hex(text: &str) -> Result<Self, CandidateError> {
        if text.len() != 64 || !text.as_bytes().iter().all(u8::is_ascii_hexdigit) {
            return Err(CandidateError::InvalidDigest { field: "digest" });
        }
        let mut bytes = [0_u8; 32];
        for (index, byte) in bytes.iter_mut().enumerate() {
            let offset = index * 2;
            *byte =
                (hex_value(text.as_bytes()[offset]) << 4) | hex_value(text.as_bytes()[offset + 1]);
        }
        Ok(Self(bytes))
    }

    /// Return the raw digest bytes.
    #[must_use]
    pub fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }
}

impl fmt::Display for ContentDigest {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        for byte in self.0 {
            write!(formatter, "{byte:02x}")?;
        }
        Ok(())
    }
}

fn hex_value(byte: u8) -> u8 {
    match byte {
        b'0'..=b'9' => byte - b'0',
        b'a'..=b'f' => byte - b'a' + 10,
        b'A'..=b'F' => byte - b'A' + 10,
        _ => 0,
    }
}

pub(super) fn parse_digest(
    text: &str,
    field: &'static str,
) -> Result<ContentDigest, CandidateError> {
    ContentDigest::from_hex(text).map_err(|_| CandidateError::InvalidDigest { field })
}
