pub type Scalar = f32;

#[derive(Copy, Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct Meters(pub Scalar);
#[derive(Copy, Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct MetersPerSecond(pub Scalar);
#[derive(Copy, Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct MetersPerSecondSquared(pub Scalar);
#[derive(Copy, Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct RadiansPerSecond(pub Scalar);
#[derive(Copy, Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct Radians(pub Scalar);
#[derive(Copy, Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct Seconds(pub Scalar);
#[derive(Copy, Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct Normalized(pub Scalar);      // [0.0, 1.0]
#[derive(Copy, Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct NormalizedSigned(pub Scalar); // [-1.0, 1.0]
#[derive(Copy, Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct Pascals(pub Scalar);
#[derive(Copy, Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct Celsius(pub Scalar);
#[derive(Copy, Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct Degrees(pub Scalar);
#[derive(Copy, Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct Microtesla(pub Scalar);
#[derive(Copy, Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct Kilograms(pub Scalar);
#[derive(Copy, Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct KilogramMeterSquared(pub Scalar);

pub trait Validated {
    fn is_valid(&self) -> bool;
    fn sanitize_or_default(&self, default: Self) -> Self;
}

impl Validated for Scalar {
    fn is_valid(&self) -> bool { self.is_finite() }
    fn sanitize_or_default(&self, default: Self) -> Self {
        if self.is_finite() { *self } else { default }
    }
}

// Macro to implement Validated for newtypes
macro_rules! impl_validated {
    ($($t:ty),*) => {
        $(
            impl Validated for $t {
                fn is_valid(&self) -> bool { self.0.is_finite() }
                fn sanitize_or_default(&self, default: Self) -> Self {
                    if self.0.is_finite() { *self } else { default }
                }
            }
            
            // Allow adding scalar to newtype (if needed, mostly we want newtype algebra)
            // For now, we keep it minimal.
        )*
    }
}

impl_validated!(
    Meters, MetersPerSecond, MetersPerSecondSquared, 
    RadiansPerSecond, Radians, Seconds, 
    Normalized, NormalizedSigned, 
    Pascals, Celsius, Degrees, Microtesla, 
    Kilograms, KilogramMeterSquared
);

pub trait FloatExt {
    fn sqrt(self) -> Self;
    fn sin(self) -> Self;
    fn cos(self) -> Self;
    fn asin(self) -> Self;
    fn powf(self, exp: Self) -> Self;
    fn atan2(self, other: Self) -> Self;
}

impl FloatExt for Scalar {
    fn sqrt(self) -> Self { libm::sqrtf(self) }
    fn sin(self) -> Self { libm::sinf(self) }
    fn cos(self) -> Self { libm::cosf(self) }
    fn asin(self) -> Self { libm::asinf(self) }
    fn powf(self, exp: Self) -> Self { libm::powf(self, exp) }
    fn atan2(self, other: Self) -> Self { libm::atan2f(self, other) }
}

macro_rules! impl_arithmetic {
    ($($t:ty),*) => {
        $(
            impl core::ops::Add for $t {
                type Output = Self;
                fn add(self, rhs: Self) -> Self {
                    Self(self.0 + rhs.0)
                }
            }
            impl core::ops::Sub for $t {
                type Output = Self;
                fn sub(self, rhs: Self) -> Self {
                    Self(self.0 - rhs.0)
                }
            }
            impl core::ops::Mul<Scalar> for $t {
                type Output = Self;
                fn mul(self, rhs: Scalar) -> Self {
                    Self(self.0 * rhs)
                }
            }
            impl core::ops::Div<Scalar> for $t {
                type Output = Self;
                fn div(self, rhs: Scalar) -> Self {
                    Self(self.0 / rhs)
                }
            }
             impl core::ops::Neg for $t {
                type Output = Self;
                fn neg(self) -> Self {
                    Self(-self.0)
                }
            }
        )*
    }
}

impl_arithmetic!(
    Meters, MetersPerSecond, MetersPerSecondSquared,
    RadiansPerSecond, Radians, Seconds,
    Normalized, NormalizedSigned,
    Pascals, Celsius, Degrees, Microtesla,
    Kilograms, KilogramMeterSquared
);
