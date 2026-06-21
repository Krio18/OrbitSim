#pragma once

namespace Orbit {

    // =========================================================================
    // Coordinate frame convention (heliocentric inertial, right-handed)
    //   X : direction de l'équinoxe vernal (ICRF)
    //   Y : 90° est dans le plan de l'écliptique
    //   Z : pôle nord de l'écliptique (« haut »)
    //   Rotation positive : règle de la main droite autour de Z
    //                       (sens antihoraire vu du pôle nord)
    // Units : SI strict — mètres, kilogrammes, secondes, radians
    // =========================================================================

    namespace Constants {

        // ---------------------------------------------------------------------
        // Constantes physiques fondamentales
        // ---------------------------------------------------------------------
        inline constexpr double G  = 6.67430e-11; // constante gravitationnelle  [m³·kg⁻¹·s⁻²]
        inline constexpr double g0 = 9.80665;      // gravité standard (exacte)  [m·s⁻²]

        // ---------------------------------------------------------------------
        // Paramètres gravitationnels standard μ = GM
        // Chargés directement : μ est connu bien plus précisément que G ou M seuls.
        // ---------------------------------------------------------------------
        inline constexpr double MU_SUN     = 1.32712440018e20; // [m³·s⁻²]
        inline constexpr double MU_EARTH   = 3.986004418e14;   // [m³·s⁻²]
        inline constexpr double MU_MOON    = 4.9048695e12;     // [m³·s⁻²]
        inline constexpr double MU_MARS    = 4.282837e13;      // [m³·s⁻²]
        inline constexpr double MU_VENUS   = 3.24858592e14;    // [m³·s⁻²]
        inline constexpr double MU_JUPITER = 1.26686534e17;    // [m³·s⁻²]
        inline constexpr double MU_SATURN  = 3.7931187e16;     // [m³·s⁻²]

        // ---------------------------------------------------------------------
        // Constantes mathématiques
        // ---------------------------------------------------------------------
        inline constexpr double PI     = 3.14159265358979323846;
        inline constexpr double TWO_PI = 6.28318530717958647692;

        // ---------------------------------------------------------------------
        // Conversions d'unités
        // ---------------------------------------------------------------------
        inline constexpr double AU         = 1.495978707e11;          // 1 UA → mètres
        inline constexpr double DEG_TO_RAD = PI / 180.0;
        inline constexpr double RAD_TO_DEG = 180.0 / PI;

    } // namespace Constants
} // namespace Orbit
