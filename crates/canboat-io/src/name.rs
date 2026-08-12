// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! The 64-bit ISO 11783-5 NAME — a device's globally-unique bus identity.

/// A device's 64-bit ISO 11783-5 NAME: its globally-unique identity on the
/// bus, and the tiebreaker for address-claim arbitration (lowest NAME wins).
///
/// Build one with [`Name::new`] plus the setters, then [`Name::to_u64`] for
/// the [`AddressClaim`](crate::address_claim::AddressClaim) or a 60928 Address
/// Claim payload. Defaults to the Marine industry group and
/// arbitrary-address-capable — the common case for an NMEA 2000 node.
///
/// ```
/// use canboat_io::name::Name;
/// let name = Name::new(381 /* B&G */, 0x1234)
///     .device_function(140) // Ownship Attitude
///     .device_class(60);    // Navigation
/// let claim_id: u64 = name.to_u64();
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Name {
    unique_number: u32,
    manufacturer_code: u16,
    device_instance: u8,
    device_function: u8,
    device_class: u8,
    system_instance: u8,
    industry_group: u8,
    arbitrary_address_capable: bool,
}

impl Name {
    /// A NAME for an 11-bit `manufacturer_code` and a 21-bit `unique_number`
    /// (both masked to width), defaulting to industry group 4 (Marine) and
    /// arbitrary-address-capable. Set the device function/class and instances
    /// as the node requires.
    pub fn new(manufacturer_code: u16, unique_number: u32) -> Self {
        Self {
            unique_number,
            manufacturer_code,
            device_instance: 0,
            device_function: 0,
            device_class: 0,
            system_instance: 0,
            industry_group: 4, // Marine
            arbitrary_address_capable: true,
        }
    }

    /// Device Function (8-bit ISO field), e.g. 130 = PC Gateway.
    pub fn device_function(mut self, v: u8) -> Self {
        self.device_function = v;
        self
    }

    /// Device Class (7-bit ISO field), e.g. 25 = Inter/Intranetwork Device.
    pub fn device_class(mut self, v: u8) -> Self {
        self.device_class = v;
        self
    }

    /// Device Instance (8-bit: ECU instance lower 3 bits + function instance
    /// upper 5), for distinguishing identical devices on one bus.
    pub fn device_instance(mut self, v: u8) -> Self {
        self.device_instance = v;
        self
    }

    /// System Instance (4-bit ISO field).
    pub fn system_instance(mut self, v: u8) -> Self {
        self.system_instance = v & 0x0f;
        self
    }

    /// Industry Group (3-bit ISO field); defaults to 4 (Marine).
    pub fn industry_group(mut self, v: u8) -> Self {
        self.industry_group = v & 0x07;
        self
    }

    /// The arbitrary-address-capable bit: when set, a node that loses a claim
    /// moves to another free address rather than going silent. Defaults to
    /// `true`.
    pub fn arbitrary_address_capable(mut self, v: bool) -> Self {
        self.arbitrary_address_capable = v;
        self
    }

    /// Whether this NAME is arbitrary-address-capable (bit 63) — the value to
    /// pass alongside [`to_u64`](Self::to_u64) to an address claimer.
    pub fn is_arbitrary_address_capable(&self) -> bool {
        self.arbitrary_address_capable
    }

    /// Pack into the 64-bit ISO 11783-5 NAME.
    pub fn to_u64(&self) -> u64 {
        let unique = (self.unique_number & 0x1f_ffff) as u64;
        let mfr = (self.manufacturer_code & 0x7ff) as u64;
        let di = self.device_instance as u64;
        unique
            | (mfr << 21)
            | ((di & 0x07) << 32)
            | (((di >> 3) & 0x1f) << 35)
            | ((self.device_function as u64 & 0xff) << 40)
            | ((self.device_class as u64 & 0x7f) << 49)
            | ((self.system_instance as u64 & 0x0f) << 56)
            | ((self.industry_group as u64 & 0x07) << 60)
            | ((self.arbitrary_address_capable as u64) << 63)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Lock the packing to the two hand-rolled `build_name` bodies this
    // builder replaces (canboat-io socketcan gateway + the motion quirk), so
    // a device's on-bus NAME — and thus its claim arbitration — never shifts.

    #[test]
    fn matches_socketcan_gateway_packing() {
        let (unique, manufacturer, system_instance) = (0x0012_3456_u32, 275_u16, 3_u8);
        let expected = {
            let unique = (unique & 0x1fffff) as u64;
            let manufacturer = manufacturer as u64 & 0x7ff;
            let device_instance: u64 = 0;
            let device_function: u64 = 130;
            let device_class: u64 = 25;
            let system_instance = system_instance as u64 & 0x0f;
            let industry_group: u64 = 4;
            let arbitrary: u64 = 1;
            unique
                | (manufacturer << 21)
                | ((device_instance & 0x07) << 32)
                | ((device_instance >> 3 & 0x1f) << 35)
                | ((device_function & 0xff) << 40)
                | ((device_class & 0x7f) << 49)
                | ((system_instance & 0x0f) << 56)
                | ((industry_group & 0x07) << 60)
                | (arbitrary << 63)
        };
        let got = Name::new(manufacturer, unique)
            .device_function(130)
            .device_class(25)
            .system_instance(system_instance)
            .to_u64();
        assert_eq!(got, expected);
    }

    #[test]
    fn matches_motion_quirk_packing() {
        let unique = 0x0001_abcd_u32 & 0x1f_ffff;
        let expected = {
            const MFG_BANDG: u64 = 381;
            const DEVICE_FUNCTION: u64 = 140;
            const DEVICE_CLASS: u64 = 60;
            const INDUSTRY_MARINE: u64 = 4;
            let arbitrary: u64 = 1;
            (unique as u64)
                | (MFG_BANDG << 21)
                | (DEVICE_FUNCTION << 40)
                | (DEVICE_CLASS << 49)
                | (INDUSTRY_MARINE << 60)
                | (arbitrary << 63)
        };
        let got = Name::new(381, unique)
            .device_function(140)
            .device_class(60)
            .to_u64();
        assert_eq!(got, expected);
    }
}
