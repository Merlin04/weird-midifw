macro_rules! create_trait {
    ($struct_name:ident < $($generics:ident),+ >, $trait_name:ident, {
        $(fn $method_name:ident($($param_name:ident: $param_type:ty),*) -> $ret_type:ty;)*
    }) => {
        pub trait $trait_name {
            $(fn $method_name(&self, $($param_name: $param_type),*) -> $ret_type;)*
        }

        impl<$($generics),+> $trait_name for $struct_name<$($generics),+> {
            $(
                fn $method_name(&self, $($param_name: $param_type),*) -> $ret_type {
                    self.$method_name($($param_name),*)
                }
            )*
        }
    };
}

use core::ops::{Add, Div, Mul, Sub};

pub(crate) use create_trait;
use rtic_monotonics::systick::fugit::Instant;

pub type SystickInstant = Instant<u32, 1, 10000>;

pub trait RangeMappable {
    fn map_range(self, from_range: (Self, Self), to_range: (Self, Self)) -> Self where Self: Sized;
    fn map_range_array<const N: usize>(self, points: [(Self, Self); N]) -> Self where Self: Sized;
}

impl<T> RangeMappable for T
    where T: Add<T, Output=T> +
    Sub<T, Output=T> +
    Mul<T, Output=T> +
    Div<T, Output=T> +
    PartialOrd<T> + 
    Copy
{
    fn map_range(self, from_range: (Self, Self), to_range: (Self, Self)) -> Self {
        to_range.0 + (self - from_range.0) * (to_range.1 - to_range.0) / (from_range.1 - from_range.0)
    }

    fn map_range_array<const N: usize>(self, points: [(Self, Self); N]) -> Self {
        if self < points[0].0 {
            return points[0].1
        }
        for w in points.windows(2) {
            let (x1, y1) = w[0];
            let (x2, y2) = w[1];
            if self >= x1 && self < x2 {
                return self.map_range((x1, x2), (y1, y2))
            }
        }
        return points.last().unwrap().1
    }
}