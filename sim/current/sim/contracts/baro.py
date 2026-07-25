"""Bar30/AP_Baro pressure contract.

This module is the canonical active-runtime owner for the pressure/depth
formulas shared by live MuJoCo, controller replay, and initial-state tools. The
formulas model the ArduSub 4.1.2 JSON-SITL path where JSON `position.z` is
converted by AP_Baro_SITL into water-barometer pressure.
"""

from __future__ import annotations

from dataclasses import dataclass


AP_BARO_FRONTEND_PA_PER_M = 9800.0
AP_BARO_SITL_SSL_AIR_PRESSURE_PA = 101325.01576
AP_BARO_SITL_WATER_DENSITY_KG_M3 = 1024.0
STANDARD_GRAVITY_M_S2 = 9.80665


@dataclass(frozen=True)
class BaroPressureLaw:
    """Map real/physical Bar30 pressure into JSON depth AP_Baro_SITL expects."""

    real_ground_pressure_pa: float
    sitl_ground_pressure_pa: float
    specific_gravity: float = 1.0
    frontend_pa_per_m: float = AP_BARO_FRONTEND_PA_PER_M
    sitl_ssl_air_pressure_pa: float = AP_BARO_SITL_SSL_AIR_PRESSURE_PA
    sitl_water_density_kg_m3: float = AP_BARO_SITL_WATER_DENSITY_KG_M3
    gravity_m_s2: float = STANDARD_GRAVITY_M_S2

    def pressure_from_depth_m(
        self,
        depth_m: float,
        *,
        surface_pressure_pa: float,
        water_density_kg_m3: float | None = None,
        gravity_m_s2: float | None = None,
    ) -> float:
        """Absolute Bar30 pressure for positive-down physical depth."""
        rho = float(self.sitl_water_density_kg_m3 if water_density_kg_m3 is None else water_density_kg_m3)
        gravity = float(self.gravity_m_s2 if gravity_m_s2 is None else gravity_m_s2)
        return float(surface_pressure_pa) + rho * gravity * max(0.0, float(depth_m))

    def frontend_altitude_m_from_pressure(self, pressure_pa: float) -> float:
        """AP_Baro water frontend altitude, negative when pressure is above ground."""
        denom = max(1.0e-9, self.frontend_pa_per_m * self.specific_gravity)
        return float((self.real_ground_pressure_pa - float(pressure_pa)) / denom)

    def frontend_depth_m_from_pressure(self, pressure_pa: float) -> float:
        """Positive-down AP_Baro water frontend depth from absolute pressure."""
        return float(max(0.0, -self.frontend_altitude_m_from_pressure(pressure_pa)))

    def sitl_depth_m_for_pressure(self, pressure_pa: float) -> float:
        """JSON depth that makes AP_Baro_SITL emit this absolute pressure."""
        denom = max(1.0e-9, self.sitl_water_density_kg_m3 * self.gravity_m_s2)
        return float(max(0.0, (float(pressure_pa) - self.sitl_ssl_air_pressure_pa) / denom))

    def sitl_depth_m_for_frontend_altitude(self, target_alt_m: float) -> float:
        """JSON depth that makes AP_Baro frontend produce target_alt_m."""
        sitl_pressure_pa = self.sitl_ground_pressure_pa - (
            float(target_alt_m) * self.frontend_pa_per_m * self.specific_gravity
        )
        return self.sitl_depth_m_for_pressure(sitl_pressure_pa)

    def sitl_depth_m_for_frontend_match(self, pressure_pa: float) -> float:
        """JSON depth that recreates the real AP_Baro water-frontend altitude."""
        target_alt_m = self.frontend_altitude_m_from_pressure(pressure_pa)
        return self.sitl_depth_m_for_frontend_altitude(target_alt_m)


def surface_pressure_for_depth_sample(
    *,
    pressure_pa: float,
    depth_m: float,
    water_density_kg_m3: float = AP_BARO_SITL_WATER_DENSITY_KG_M3,
    gravity_m_s2: float = STANDARD_GRAVITY_M_S2,
) -> float:
    """Surface pressure datum that makes a depth sample reproduce pressure_pa."""
    return float(pressure_pa) - float(water_density_kg_m3) * float(gravity_m_s2) * max(0.0, float(depth_m))


__all__ = [
    "AP_BARO_FRONTEND_PA_PER_M",
    "AP_BARO_SITL_SSL_AIR_PRESSURE_PA",
    "AP_BARO_SITL_WATER_DENSITY_KG_M3",
    "STANDARD_GRAVITY_M_S2",
    "BaroPressureLaw",
    "surface_pressure_for_depth_sample",
]
