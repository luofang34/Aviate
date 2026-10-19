//! Calendar time as the decimal year the model takes.

const SECONDS_PER_DAY: i64 = 86_400;

/// Days from 1970-01-01 to January 1 of `year` (proleptic Gregorian).
fn days_to_year_start(year: i64) -> i64 {
    // Days from civil, for month 1 day 1 (H. Hinnant's algorithm).
    let y = year - 1;
    let era = y.div_euclid(400);
    let yoe = y.rem_euclid(400);
    let doy = 306; // March-based day of year of January 1.
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe - 719_468
}

fn is_leap(year: i64) -> bool {
    (year % 4 == 0 && year % 100 != 0) || year % 400 == 0
}

/// The decimal year of a UTC instant given as seconds since the Unix
/// epoch: the year plus the elapsed fraction of it, leap years counted.
pub fn decimal_year_from_unix_seconds(unix_seconds: i64) -> f64 {
    let days = unix_seconds.div_euclid(SECONDS_PER_DAY);
    // A first guess from the mean year length, corrected by at most one.
    let mut year = 1970 + (days as f64 / 365.2425) as i64;
    while days_to_year_start(year) > days {
        year -= 1;
    }
    while days_to_year_start(year + 1) <= days {
        year += 1;
    }
    let year_days = if is_leap(year) { 366 } else { 365 };
    let into_year = unix_seconds - days_to_year_start(year) * SECONDS_PER_DAY;
    year as f64 + into_year as f64 / (year_days * SECONDS_PER_DAY) as f64
}

#[cfg(test)]
mod tests;
