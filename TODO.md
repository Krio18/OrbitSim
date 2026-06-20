# OrbitSim - Development Roadmap

**Architecture:** Deterministic fixed-timestep physics core · Hybrid gravity (n-body vessel + bodies on rails) · Double precision (float64) · SI units
**Current Version:** 0.1.0-alpha
**Last Updated:** 2026-06-02

> Sim aérospatiale de type Kerbal : système solaire avec gravité réelle, où les orbites *émergent* de l'intégration des forces, plus descente/atterrissage propulsé. Conçue comme deuxième projet alternant avec le game engine — réutilise volontairement les mêmes patterns (ServiceLocator, fixed timestep, manager lifecycle) pour que passer d'un projet à l'autre soit naturel.

---

## Table of Contents
1. [Architecture Overview](#architecture-overview)
2. [Phase 0: Core Simulation Architecture](#phase-0-core-simulation-architecture)
3. [Phase 1: First Orbit Milestone (2D)](#phase-1-first-orbit-milestone-2d)
4. [Phase 2: N-body Gravity & Multi-body](#phase-2-n-body-gravity--multi-body)
5. [Phase 3: Celestial Bodies & Solar System](#phase-3-celestial-bodies--solar-system)
6. [Phase 4: Coordinate Frames & Precision](#phase-4-coordinate-frames--precision)
7. [Phase 5: The Vessel (6DOF Rigid Body)](#phase-5-the-vessel-6dof-rigid-body)
8. [Phase 6: Atmospheres (Multi-layer)](#phase-6-atmospheres-multi-layer)
9. [Phase 7: Aerodynamics](#phase-7-aerodynamics)
10. [Phase 8: Guidance, Navigation & Control](#phase-8-guidance-navigation--control)
11. [Phase 9: Orbital Mechanics & Trajectory Prediction](#phase-9-orbital-mechanics--trajectory-prediction)
12. [Phase 10: Time Warp](#phase-10-time-warp)
13. [Phase 11: Rendering & Visualization](#phase-11-rendering--visualization)
14. [Phase 12: Sandbox & Editor Tools](#phase-12-sandbox--editor-tools)
15. [Validation & Testing Strategy](#validation--testing-strategy)
16. [Technical Decisions](#technical-decisions)
17. [Priority Order (Next Steps)](#priority-order-next-steps)
18. [Build System](#build-system)
19. [Phase 13: GPU Particle Field (CUDA)](#phase-13-gpu-particle-field-cuda)
20. [Notes & Best Practices](#notes--best-practices)

---

## Architecture Overview

### Design Principles
- **Deterministic fixed-timestep core** - La physique avance par pas fixes, totalement découplée du framerate de rendu. Mêmes entrées → mêmes sorties, toujours.
- **Double precision everywhere** - Tout l'état physique en `float64`. Les distances du système solaire (~10¹¹ m) tuent la `float32`.
- **Symplectic integration** - Conservation de l'énergie sur le long terme : les orbites restent fermées au lieu de spiraler.
- **Hybrid gravity model** - Les corps célestes suivent des orbites képlériennes analytiques (sur rails), le vaisseau ressent la somme des gravités de tous les corps. On obtient les orbites émergentes + les points de Lagrange tout en gardant un système solaire stable.
- **Data-driven bodies** - Masses, rayons, éléments orbitaux et compositions atmosphériques chargés depuis des fichiers, pas codés en dur.
- **Sim core ≠ rendering** - Le moteur physique ne connaît rien du rendu. On peut le tester sans fenêtre.

### Target Feature Set
- Système solaire complet basé sur des paramètres réels (masses, μ, éléments orbitaux)
- Gravité qui s'applique en continu : le vaisseau tombe toujours, sauf si la vitesse latérale crée une orbite
- Atmosphères multi-couches dépendant de la composition de chaque corps
- Vaisseau 6DOF avec masse variable, étages, poussée orientable
- Pilote automatique : maintien d'attitude, descente propulsée, atterrissage en douceur
- Vue carte avec prédiction de trajectoire (coniques raccordées) et nœuds de manœuvre
- Time warp via propagation analytique

### Simulation Step Order (Critical!)
**Chaque pas de temps fixe `dt` exécute exactement cette séquence :**

1. **Bodies layer** - Avancer les corps célestes sur leurs rails (positions analytiques à `t`)
2. **Force layer** (sur le vaisseau)
   - Gravité : somme de `−μᵢ · r̂ / rᵢ²` sur tous les corps
   - Aéro : traînée/portance depuis l'atmosphère du corps dominant
   - Poussée : depuis les moteurs actifs + commandes du contrôleur
3. **Integration layer** - Intégrer position/vitesse (translation) et quaternion/ω (rotation) via intégrateur symplectique
4. **Mass layer** - Décrémenter la masse de carburant selon le débit massique
5. **Events layer** - Transitions de sphère d'influence (SOI), collisions sol, séparation d'étages
6. **Interpolation** - Interpoler l'état pour le rendu (découplé)

---

## Phase 0: Core Simulation Architecture

**Goal:** Poser le squelette de simulation dont tout le reste dépend — boucle, temps, état, intégrateur

### 0.1 State Representation
**Purpose:** Définir ce qui décrit complètement le système à l'instant `t`

- [x] Créer `src/Core/State.hpp`
- [x] Translation state : `dvec3 position`, `dvec3 velocity` (double précision)
- [x] Rotation state (ajouté en Phase 5) : `dquat orientation`, `dvec3 angularVelocity`
- [x] `double mass` (variable — diminue quand le carburant brûle)
- [x] Séparer clairement l'état (intégré) des dérivées (forces/couples calculés chaque pas)
- [x] Structure `Derivative { dvec3 dPosition; dvec3 dVelocity; }` pour les intégrateurs multi-étapes

**Why this matters:** L'intégrateur ne fait qu'avancer ce vecteur d'état dans le temps. Tout ce qui n'est pas dedans n'existe pas pour la physique.

### 0.2 Fixed Timestep Loop
**Purpose:** Faire tourner la physique à pas constant, indépendamment du framerate

- [x] Créer `src/Core/SimulationLoop.hpp` et `.cpp`
- [x] Pattern accumulateur : accumuler le temps réel, avancer la physique par chunks de `dt` fixe
- [x] `dt` physique fixe configurable (défaut 1/60 s, plus petit près des corps)
- [x] Cap du nombre de sous-pas par frame (éviter la spirale de la mort si le rendu lag)
- [x] Calculer un facteur d'interpolation `alpha` pour le rendu entre deux états physiques
- [x] Stocker l'état précédent + courant pour l'interpolation

> **Note technique :** Le `dt` fixe est non négociable pour le déterminisme. Un `dt` variable rend la simulation non reproductible et fait dériver les orbites différemment selon le framerate.
> Si un `dt` adaptatif (plus petit près des corps) est un jour souhaité, le critère de switch doit être basé exclusivement sur l'état de la simulation (ex. altitude, distance au corps dominant) — jamais sur le framerate ou un timer mural. Un critère non déterministe réintroduit la non-reproductibilité que le fixed timestep cherche à éliminer.

**Why this matters:** C'est le cœur de toute sim. Découpler physique et rendu donne déterminisme + stabilité.

### 0.3 TimeManager
**Purpose:** Source de temps unifiée, avec support du time warp

- [x] Créer `src/Core/TimeManager.hpp` et `.cpp`
- [x] Mesurer le temps réel entre deux frames avec `std::chrono::steady_clock` → `getUnscaledDeltaTime()` (c'est cette valeur que `SimulationLoop::update()` attend en paramètre — pas la version scalée)
- [x] Temps de simulation cumulé (secondes depuis t0)
- [x] `fixedDeltaTime` : pas physique constant
- [x] `_timeScale` : modificateur de vitesse général (0.0 = pause, 0.5 = ralenti cinématique, 1.0 = normal) — affecte `getDeltaTime()` et `_simTime`
- [x] `_warpLevel` : accélération orbitale KSP (1×, 10×, 100×, 1000×) — multiplie uniquement `_simTime`, jamais passé à `SimulationLoop`
- [x] Temps réel (mur) vs temps simulé
- [x] Calendrier optionnel : convertir secondes ↔ date (jours/années) pour l'affichage
- [x] Exposer `getSimTime()`, `getFixedDt()`, `getWarpLevel()`, `getDeltaTime()`
- [x] Machine d'état `enum class State { Stopped, Running, Paused }` avec cycle de vie `start()` / `pause()` / `resume()` / `stop()`
- [x] `setFixedDt()` verrouillé en état `Stopped` uniquement (protège le déterminisme)
- [x] `_elapsedTime` avance en `Running` et `Paused`, pas en `Stopped`
- [x] Calendrier grégorien exact (algorithme Howard Hinnant, `constexpr`)

**Why this matters:** Observer une orbite demande d'accélérer le temps. Le warp interagit avec l'intégration (voir Phase 10).

### 0.4 Numerical Integrators
**Purpose:** Avancer l'état d'un pas — le choix décide de la stabilité des orbites

- [ ] Créer `src/Core/Integrator.hpp` (interface)
- [ ] Implémenter **Semi-implicit Euler** (référence pédagogique — montre la dérive)
- [ ] Implémenter **Velocity Verlet** (symplectique — workhorse pour les orbites)
- [ ] Implémenter **Leapfrog** (symplectique, équivalent, parfois plus pratique)
- [ ] Implémenter **RK4** (non symplectique — utile pour comparer, NE PAS utiliser pour orbites longues)
- [ ] Interface commune : `step(State&, forceFunc, dt)`
- [ ] Permettre de switcher d'intégrateur via config (pour comparer la dérive)

> **Note technique :** RK4 ne conserve PAS l'énergie : une orbite censée être stable spirale lentement. Velocity Verlet / leapfrog sont symplectiques → l'énergie oscille autour d'une valeur fixe sans dériver. C'est *la* clé d'orbites fermées indéfiniment.

> **Note technique — Verlet et forces dépendant de la vitesse :** Le Velocity Verlet classique suppose `a = f(position)` uniquement. Dès que la traînée (Phase 7) ou la poussée dépend de la vitesse ou de la masse courante, la mise à jour `v += ½(a₀ + a₁)dt` devient implicite : `a₁` dépend de `v₁` qui n'est pas encore connu. Trois options : (1) itérer pour résoudre l'implicite (coûteux), (2) traiter le terme de traînée en semi-implicite — `F_drag ∝ −k·v`, ce qui se linéarise exactement — , (3) accepter une dégradation d'ordre pour ces termes. En pratique, l'intégrateur reste pleinement symplectique en orbite balistique (là où la précision à long terme compte), et approximatif en atmosphère/poussée (là où l'incertitude du modèle aéro domine de toute façon). **Décision à prendre consciemment** — ne pas la découvrir quand le test Tsiolkovsky ne matche pas.

**Why this matters:** Mauvais intégrateur = orbites qui s'écrasent ou s'échappent toutes seules après quelques tours. Tout repose là-dessus.

### 0.5 Constants, Units & Logger
**Purpose:** Conventions cohérentes et outils de debug dès le départ

- [ ] Créer `src/Core/Constants.hpp` : `G`, `g0 = 9.80665`, conversions
- [ ] **SI partout** : mètres, kilogrammes, secondes, radians
- [ ] Utiliser `μ = GM` (paramètre gravitationnel standard) plutôt que `G·M` séparément
- [ ] Logger avec niveaux (Info/Warning/Error)
- [ ] Macro d'assertion `SIM_ASSERT`
- [ ] Documenter les conventions de repère (axe « haut », sens de rotation positif)

> **Note technique :** Pour les vrais corps, `μ` est connu bien plus précisément que `G` ou `M` séparément. Charger `μ` directement évite d'accumuler l'imprécision de `G`.

**Why this matters:** Les bugs d'unités et de repère sont les plus pénibles. Les fixer comme conventions évite des heures de debug.

### Points d'intégration
- **Prérequis :** aucun — c'est la fondation de tout le reste
- **Fournit à :** toutes les phases — `State`, `SimulationLoop`, `TimeManager`, `Integrator`, `Constants` sont le squelette commun sur lequel chaque phase s'appuie
- **Dans la boucle :** *définit* la boucle elle-même et les 6 étapes du Simulation Step Order

### 0.x Intégration dans l'application
**Purpose:** Brancher tous les composants Phase 0 dans `main.cpp` et valider la boucle de bout en bout

- [ ] Instancier `TimeManager` et `SimulationLoop` dans `main()`
- [ ] Boucle principale : appeler `timeManager.update()` puis `simulationLoop.update(timeManager.getUnscaledDeltaTime())`
- [ ] Récupérer `alpha` et l'afficher (vérification visuelle que la boucle tourne)
- [ ] Afficher `FPS`, `simTime`, `calendar` via les getters `TimeManager`
- [ ] Instancier un `State` de test et vérifier les getters/setters
- [ ] Instancier chaque intégrateur (Phase 0.4) et appeler `step()` une fois avec un `State` factice
- [ ] Vérifier que la boucle tourne sans spirale de mort (cap 8 sous-pas actif)
- [ ] **Success criteria :** La simulation tourne, le calendrier avance, alpha ∈ [0, 1]

---

## Phase 1: First Orbit Milestone (2D)

**Goal:** Obtenir une orbite stable qui émerge de la gravité — le plus vite possible, en 2D, pour valider le cœur

### 1.1 Single Body + Test Particle
**Purpose:** Le cas le plus simple : un corps massif fixe, une particule sans masse

- [ ] Corps massif fixe à l'origine avec `μ`
- [ ] Particule ponctuelle avec `position` + `velocity`
- [ ] Force gravitationnelle : `a = −μ · r̂ / r²` (accélération, indépendante de la masse de la particule)
- [ ] Boucle de simulation appliquant la gravité chaque pas

### 1.2 Integrator Comparison
**Purpose:** Voir de ses yeux pourquoi l'intégrateur compte

- [ ] Lancer la particule avec une vitesse latérale donnant une orbite circulaire (`v = √(μ/r)`)
- [ ] Avec **Euler semi-implicite** : observer la dérive (l'orbite change de forme)
- [ ] Avec **Velocity Verlet** : observer une orbite stable et fermée sur des centaines de tours
- [ ] **Success criteria :** L'orbite reste fermée et l'énergie spécifique `ε = v²/2 − μ/r` reste constante (à la tolérance numérique près)

### 1.3 Conservation Checks
**Purpose:** Vérifier que la physique est juste, pas juste « jolie »

- [ ] Logger l'énergie spécifique orbitale à chaque tour
- [ ] Logger le moment cinétique spécifique `h = r × v`
- [ ] Vérifier la période contre `T = 2π√(a³/μ)`
- [ ] Tracer la dérive d'énergie en fonction du temps pour chaque intégrateur

### 1.4 Minimal 2D Rendering
**Purpose:** Voir l'orbite

- [ ] Rendu 2D basique (le corps, la particule, une traînée des N dernières positions)
- [ ] Mapping monde → écran avec zoom/pan
- [ ] **Success criteria :** Lâcher la particule à la bonne vitesse et la voir boucler une orbite fermée stable sans rien scripter

> **Note technique :** À ce stade, ne rends pas en `float64` directement — convertis l'état physique double précision en coordonnées écran. Le rendu lui-même peut rester simple.

**Why this matters:** Si ça marche, le cœur du moteur est validé. Tout le reste n'ajoute que des termes au modèle de forces.

### Points d'intégration
- **Prérequis :** Phase 0 complète (State, SimulationLoop, Integrator, TimeManager)
- **Fournit à :** Phase 2 (GravitySystem single-body → base à généraliser en n-corps) · valide que le cœur est correct avant d'empiler les phases suivantes
- **Dans la boucle :** Force layer étape 2 (gravité d'un corps fixe) + Integration layer étape 3 (Verlet sur position/vitesse)

### 1.x Intégration dans l'application
**Purpose:** Brancher le premier corps + particule dans la boucle et valider l'orbite

- [ ] Instancier un corps massif (μ fixe) et une particule (`State`) dans `main.cpp`
- [ ] Appeler `GravitySystem` dans la Force layer (étape 2) de `SimulationLoop`
- [ ] Logger l'énergie spécifique `ε` et le moment cinétique `h` à chaque tour
- [ ] Lancer la même simulation avec Euler puis Velocity Verlet et comparer la dérive sur 100 tours
- [ ] **Success criteria :** Orbite fermée et stable avec Velocity Verlet, dérive visible avec Euler

---

## Phase 2: N-body Gravity & Multi-body

**Goal:** Généraliser à plusieurs corps qui s'attirent, avec le modèle de gravité hybride

### 2.1 Gravity Accumulation
**Purpose:** Sommer les contributions de tous les corps sur le vaisseau

- [ ] Créer `src/Physics/GravitySystem.hpp` et `.cpp`
- [ ] Pour le vaisseau : sommer `aᵢ = −μᵢ · (r − rᵢ).normalized() / |r − rᵢ|²` sur tous les corps
- [ ] Paramètre de softening optionnel pour éviter la singularité quand `r → 0`
- [ ] Direct summation (O(n²)) : suffisant pour < ~100 corps (un système solaire)
- [ ] *(Optionnel, plus tard)* Barnes-Hut O(n log n) si milliers de corps (probablement inutile ici)

### 2.2 Hybrid Model Decision
**Purpose:** Décider qui attire qui

- [ ] **Corps célestes** : sur rails képlériens analytiques (Phase 3), PAS de gravité mutuelle simulée
- [ ] **Vaisseau** : ressent la somme des gravités de tous les corps (vrai n-corps pour lui seul)
- [ ] Permet les points de Lagrange et les perturbations réelles côté vaisseau
- [ ] Garde le système solaire parfaitement stable côté corps

> **Note technique :** Simuler les planètes elles-mêmes en n-corps complet les fait dériver sur le long terme et déstabilise le système. Les rails analytiques garantissent un système solaire fidèle et reproductible. C'est le compromis que font Orbiter et (en gros) KSP.

### 2.3 Collision Detection
**Purpose:** Détecter l'impact sur la surface d'un corps

- [ ] Distance vaisseau ↔ centre du corps < rayon → collision/atterrissage
- [ ] Émettre un événement de collision (vitesse d'impact, corps touché)
- [ ] Distinguer atterrissage doux (Phase 8) vs crash (vitesse trop élevée)

**Why this matters:** Sans collision, le vaisseau traverse les planètes. C'est aussi la base de l'atterrissage.

### Points d'intégration
- **Prérequis :** Phase 0 (boucle + intégrateur) · Phase 1 (GravitySystem validé)
- **Fournit à :** Phase 5 (événement collision → base de l'atterrissage propulsé) · Phase 10 (transitions SOI → le warp on-rails doit les détecter et re-raccorder la conique) · Phase 9 (hiérarchie SOI utilisée pour les coniques raccordées)
- **Dans la boucle :** Force layer étape 2 (gravity accumulation sur N corps) · Events layer étape 5 (collision sol, transitions SOI)

### 2.x Intégration dans l'application
**Purpose:** Généraliser la gravité à N corps et valider la détection de collision

- [ ] Remplacer le corps unique par une liste de N corps dans `GravitySystem`
- [ ] Sommer les contributions gravitationnelles de tous les corps sur la particule chaque pas
- [ ] Brancher la détection de collision dans l'Events layer (étape 5)
- [ ] Logger les transitions de SOI détectées
- [ ] **Success criteria :** Particule attirée par plusieurs corps simultanément, collision sol détectée

---

## Phase 3: Celestial Bodies & Solar System

**Goal:** Construire un système solaire à partir de vrais paramètres, avec corps sur rails

### 3.1 CelestialBody Data
**Purpose:** Décrire un corps avec des paramètres physiques réels

- [ ] Créer `src/Bodies/CelestialBody.hpp`
- [ ] `μ` (paramètre gravitationnel standard), `radius`, `mass`
- [ ] Période de rotation sidérale + inclinaison de l'axe (obliquité)
- [ ] Corps parent (hiérarchie : Soleil → planètes → lunes)
- [ ] Référence vers le modèle d'atmosphère (Phase 6), nullable
- [ ] Rayon de sphère d'influence (calculé, voir 3.3)

### 3.2 Keplerian Propagation (On Rails)
**Purpose:** Calculer la position d'un corps à n'importe quel instant, analytiquement

- [ ] Stocker les éléments orbitaux : demi-grand axe `a`, excentricité `e`, inclinaison `i`, longitude du nœud ascendant `Ω`, argument du périapse `ω`, anomalie moyenne à l'époque `M₀`
- [ ] Anomalie moyenne à `t` : `M = M₀ + n·(t − t₀)`, avec `n = √(μ/a³)`
- [ ] Résoudre l'équation de Kepler `M = E − e·sin(E)` pour l'anomalie excentrique `E` (Newton-Raphson)
- [ ] Anomalie vraie `ν` depuis `E`
- [ ] Convertir `(a, e, ν, i, Ω, ω)` → position + vitesse dans le repère parent
- [ ] Cache de la position par frame (recalcul seulement si le temps change)

> **Note technique :** L'itération de Newton sur l'équation de Kepler converge en ~3-5 itérations pour `e < 0.9`. Démarrer avec `E₀ = M` (ou `E₀ = π` pour les fortes excentricités).
> **Note technique — cas hyperbolique :** Pour `e > 1` (trajectoire hyperbolique — typique d'un vaisseau entrant dans la SOI d'une lune), l'équation de Kepler devient `M = e·sinh(H) − H` (anomalie hyperbolique `H`), résolue par Newton-Raphson sur `H₀ = M/e`. Prévoir aussi le cas limite parabolique `e = 1` (`M = D + D³/3`, `D = tan(ν/2)`). Sans ça, la prédiction de flyby plante exactement au moment le plus intéressant (voir 9.2).

### 3.3 Sphere of Influence (SOI)
**Purpose:** Déterminer quel corps domine à une position donnée

- [ ] Rayon de SOI (Laplace) : `r_SOI ≈ a · (m_corps / m_parent)^(2/5)`
- [ ] Fonction « quel corps domine cette position ? » (parcours de la hiérarchie)
- [ ] Détecter les transitions de SOI pour le vaisseau (événement)

> **Note technique :** La SOI sert surtout à la prédiction de trajectoire (coniques raccordées, Phase 9) et au time warp (Phase 10). En vol normal n-corps, le vaisseau ressent tout le monde de toute façon.

### 3.4 Data-driven Loading
**Purpose:** Charger le système solaire depuis un fichier

- [ ] Format JSON : un fichier décrivant tous les corps et leur hiérarchie
- [ ] Charger les vrais paramètres (Soleil, planètes, lunes) — masses, rayons, μ, éléments orbitaux
- [ ] Validation : hiérarchie cohérente, pas de référence parent manquante
- [ ] Permettre des systèmes fictifs (style Kerbol) en changeant juste le fichier

### 3.5 Body Rotation
**Purpose:** Faire tourner les corps sur eux-mêmes

- [ ] Angle de rotation à `t` depuis la période sidérale
- [ ] Repère tournant lié au corps (pour la position au sol, l'atmosphère qui tourne avec)

**Why this matters:** La rotation du corps crée le « vent » atmosphérique (Phase 7) et détermine les positions au sol pour l'atterrissage.

### Points d'intégration
- **Prérequis :** Phase 0 (TimeManager pour `t`) · Phase 2 (gravity accumulation consomme les positions des corps à chaque pas)
- **Fournit à :** Phase 2 (positions + μ des corps à chaque pas) · Phase 6 (modèle d'atmosphère + rotation attachés au corps) · Phase 9 (μ + éléments orbitaux pour les coniques et le solveur Kepler) · Phase 10 (rails képlériens = base de la propagation analytique en warp)
- **Dans la boucle :** **Bodies layer — étape 1** (propagation képlérienne analytique, avant tout calcul de force du pas courant)

### 3.x Intégration dans l'application
**Purpose:** Charger le système solaire depuis JSON et faire tourner les corps sur leurs rails

- [ ] Charger le fichier JSON des corps et instancier les `CelestialBody`
- [ ] Appeler la propagation képlérienne (Bodies layer — étape 1) dans `SimulationLoop`
- [ ] Vérifier qu'un corps à `t = T + période` retrouve sa position initiale
- [ ] Brancher la hiérarchie SOI et tester la détection de transition
- [ ] **Success criteria :** Terre et Lune suivent leurs orbites analytiques sur plusieurs périodes sans dérive

---

## Phase 4: Coordinate Frames & Precision

**Goal:** Gérer les repères et la précision flottante — le piège silencieux des sims spatiales

### 4.1 Reference Frames
**Purpose:** Transformer proprement entre les différents repères

- [ ] Créer `src/Frames/FrameManager.hpp`
- [ ] Repère inertiel héliocentrique (référence principale)
- [ ] Repère lié à un corps (centré sur la planète, non tournant)
- [ ] Repère tournant lié au corps (tourne avec la planète)
- [ ] Repère local du vaisseau (body frame)
- [ ] Transformations position + vitesse entre repères (attention aux termes de Coriolis/centrifuge dans les repères tournants)

### 4.2 Floating Origin / Origin Rebasing
**Purpose:** Garder la précision de rendu malgré les distances énormes

- [ ] Recentrer le repère de rendu sur le vaisseau actif (ou le corps dominant)
- [ ] Physique en `float64` absolu, rendu en `float32` relatif à l'origine flottante
- [ ] Rebaser quand le vaisseau s'éloigne trop de l'origine courante

> **Note technique :** 1 UA ≈ 1,5×10¹¹ m. En `float32`, la résolution à cette distance dépasse le kilomètre → ton vaisseau tremble et se téléporte. La physique reste en double, et on ne convertit en simple qu'après avoir soustrait l'origine flottante.

### 4.3 Scaled Space (Distant Bodies)
**Purpose:** Afficher les corps lointains sans casser la précision

- [ ] Rendre les corps très éloignés à une échelle réduite dans un « espace mis à l'échelle »
- [ ] Basculer entre espace local (proche) et espace mis à l'échelle (lointain)

**Why this matters:** C'est exactement là que KSP s'est cassé les dents. Régler ça tôt évite une réécriture douloureuse plus tard.

### Points d'intégration
- **Prérequis :** Phase 0 (State en float64) · Phase 3 (corps de référence pour définir les repères parent/tournants)
- **Fournit à :** Phase 7 (vitesse relative à l'atmosphère tournante, via repère lié au corps) · Phase 11 (floating origin → conversion float64→float32 avant tout vertex shader) · Phase 13 (coordonnées locales fp32 pour les particules GPU, déjà en précision locale)
- **Dans la boucle :** transformations à la demande — pas d'étape dédiée dans la boucle principale, mais l'origine flottante doit être rebased **avant** la conversion vers le rendu (étape 6)

### 4.x Intégration dans l'application
**Purpose:** Brancher les repères et l'origine flottante avant le rendu

- [ ] Instancier `FrameManager` et le relier aux `CelestialBody`
- [ ] Convertir l'état physique (float64 héliocentrique) en coordonnées locales (float32) avant toute passe de rendu
- [ ] Implémenter le rebasing de l'origine flottante quand le vaisseau dépasse le seuil de précision
- [ ] Vérifier les transformations aller-retour (identitaires à la tolérance numérique près)
- [ ] **Success criteria :** Position de rendu stable sans tremblement à 1 UA

---

## Phase 5: The Vessel (6DOF Rigid Body)

**Goal:** Passer du point matériel à un vrai corps rigide avec orientation, masse variable et poussée

### 5.1 Rigid Body State
**Purpose:** Ajouter la rotation à l'état

> **Rappel Phase 0.1 :** `State` ne contenait que position, vitesse et masse — la rotation était volontairement absente pour valider le cœur orbital sans complexité inutile. C'est ici qu'on l'ajoute, avec sa dérivée dans `Derivative` (`dQuat`, `dAngularVelocity`), car la dérivée d'un quaternion est `q̇ = ½·ω·q` — non triviale à intégrer.

- [ ] Étendre `State` : `dquat orientation` (quaternion unité = pas de rotation), `dvec3 angularVelocity` (rad/s dans le body frame)
- [ ] Tenseur d'inertie + centre de masse
- [ ] Intégration de l'orientation : `q̇ = ½ · ω · q` (ω en quaternion pur), renormaliser `q` chaque pas
- [ ] Accumulation des couples → `α = I⁻¹ · (τ − ω × (I·ω))`

> **Note technique :** Renormalise le quaternion à chaque pas : l'intégration numérique le fait dériver de la norme unité, ce qui déforme les rotations.

### 5.2 Mass Properties
**Purpose:** Gérer une masse qui change quand le carburant brûle

- [ ] Masse à sec, masse de carburant, masse totale
- [ ] Recalculer le centre de masse et l'inertie quand le carburant diminue
- [ ] Exposer la masse courante au système de gravité

### 5.3 Propulsion
**Purpose:** Modéliser les moteurs

- [ ] Créer `src/Vessel/Engine.hpp`
- [ ] Poussée (N), impulsion spécifique `Isp` (s)
- [ ] Débit massique : `ṁ = F / (Isp · g0)`
- [ ] Manette des gaz (throttle 0-1)
- [ ] Poussée appliquée dans le body frame, transformée en repère monde avant intégration
- [ ] Orientation de la poussée (gimbal) : vecteur de poussée orientable de quelques degrés

### 5.4 Staging
**Purpose:** Larguer les étages

- [ ] Représenter le vaisseau comme une pile d'étages
- [ ] Séparation : retirer un étage, recalculer masse/inertie/CoM
- [ ] Événement de staging

### 5.5 RCS / Attitude Thrusters
**Purpose:** Contrôler l'orientation finement

- [ ] Petits propulseurs produisant des couples
- [ ] Consommation de carburant RCS séparée

> **Note technique :** Vérifie ton modèle avec l'équation de Tsiolkovsky : `Δv = Isp·g0·ln(m₀/m₁)`. Le Δv obtenu en intégrant une poussée constante dans le vide doit matcher cette formule.

**Why this matters:** C'est le passage du « caillou en orbite » à une vraie fusée pilotable.

### Points d'intégration
- **Prérequis :** Phase 0 (State étendu avec orientation + angularVelocity) · Phase 2 (gravité reçue en n-corps) · Phase 3 (corps dominant pour l'orientation locale et le frame body)
- **Fournit à :** Phase 6/7 (section transversale, altitude, vitesse du vaisseau) · Phase 8 (moteurs + RCS disponibles à commander) · Phase 9 (masse courante pour les calculs Δv)
- **Dans la boucle :** Force layer étape 2 (poussée en body frame → world) · Integration layer étape 3 (quaternion + ω) · **Mass layer — étape 4** (décrément carburant après intégration)

### 5.x Intégration dans l'application
**Purpose:** Transformer la particule en vaisseau pilotable avec orientation et poussée

- [ ] Étendre `State` et `Derivative` avec `orientation` et `angularVelocity`
- [ ] Brancher l'intégration du quaternion (`q̇ = ½·ω·q`) dans Integration layer (étape 3)
- [ ] Brancher le décrément de carburant dans Mass layer (étape 4)
- [ ] Appliquer la poussée en body frame → world dans la Force layer (étape 2)
- [ ] Vérifier Tsiolkovsky : Δv mesuré en simulation == `Isp · g0 · ln(m0/m1)`
- [ ] **Success criteria :** Vaisseau peut changer d'orbite via une manœuvre de poussée

---

## Phase 6: Atmospheres (Multi-layer)

**Goal:** Modéliser des atmosphères dont le profil dépend de la composition

### 6.1 Atmosphere Model
**Purpose:** Donner densité, pression, température en fonction de l'altitude

- [ ] Créer `src/Atmosphere/AtmosphereModel.hpp`
- [ ] Densité via formule barométrique : `ρ(h) = ρ₀ · exp(−h / H)`
- [ ] Hauteur d'échelle `H = R·T / (M·g)` (R = constante des gaz, M = masse molaire, T = température)
- [ ] Vitesse du son `c = √(γ·R·T / M)` (pour le nombre de Mach)
- [ ] Altitude de fin d'atmosphère (limite type Kármán)

### 6.2 Multi-layer Profile
**Purpose:** Couches successives avec propriétés distinctes

- [ ] Définir des couches (troposphère, stratosphère, ...) avec gradient de température (lapse rate) propre
- [ ] Continuité de pression aux frontières de couches
- [ ] Composition → masse molaire moyenne → hauteur d'échelle de la couche

> **Note technique :** La composition entre par la masse molaire `M` : elle change `H`, donc tout le profil de densité, donc tout le freinage et le profil de rentrée. Une atmosphère de CO₂ (Mars/Vénus) se comporte très différemment d'une d'azote.

### 6.3 Data-driven Atmospheres
**Purpose:** Configurer chaque corps depuis le fichier de données

- [ ] Charger les couches + composition + température de surface depuis le JSON des corps
- [ ] Corps sans atmosphère = modèle nul (densité 0 partout)

**Why this matters:** L'atmosphère alimente toute l'aérodynamique. Multi-couches + composition = des planètes qui se « sentent » vraiment différentes.

### Points d'intégration
- **Prérequis :** Phase 3 (rayon + rotation sidérale du corps pour l'altitude et la vitesse du vent) · Phase 4 (altitude = |r − r_corps| − rayon, calculée dans le repère du corps)
- **Fournit à :** Phase 7 (ρ(h), P(h), T(h), c(h) à chaque pas de la Force layer) · Phase 8 (densité locale utilisée dans le calcul de suicide burn et de drag en descente)
- **Dans la boucle :** consultée dans la **Force layer — étape 2** par AeroSystem · service passif, pas d'étape propre dans la boucle principale

### 6.x Intégration dans l'application
**Purpose:** Associer un profil atmosphérique à chaque corps et le rendre disponible à la Force layer

- [ ] Associer un `AtmosphereModel` à chaque `CelestialBody` lors du chargement JSON
- [ ] Appeler `atmo.getDensity(altitude)` dans la Force layer avant `AeroSystem`
- [ ] Vérifier le profil : `ρ(0) == ρ₀` des données, `ρ` nul au-delà de l'altitude limite
- [ ] Tester la continuité de pression aux frontières de couches
- [ ] **Success criteria :** Densité atmosphérique correcte à chaque altitude, corps sans atmosphère retourne 0

---

## Phase 7: Aerodynamics

**Goal:** Appliquer les forces aérodynamiques au vaisseau

### 7.1 Drag
**Purpose:** La force de traînée

- [ ] Créer `src/Physics/AeroSystem.hpp`
- [ ] Traînée : `F_d = ½ · ρ · v_rel² · C_d · A`, opposée à la vitesse relative
- [ ] Densité `ρ` depuis l'atmosphère du corps dominant (Phase 6)
- [ ] Vitesse *relative à l'atmosphère* : soustraire la vitesse de l'atmosphère qui tourne avec le corps
- [ ] Coefficient `C_d` simple d'abord (sphère/cylindre), raffiner ensuite

### 7.2 Dynamic Pressure & Limits
**Purpose:** Suivre les contraintes physiques

- [ ] Pression dynamique `Q = ½ · ρ · v_rel²`
- [ ] Limite structurelle optionnelle (destruction si Q trop élevée — « max Q »)

### 7.3 Lift (Optional)
**Purpose:** Pour les corps portants / ailes

- [ ] Portance perpendiculaire à la vitesse relative, dépend de l'angle d'attaque
- [ ] Modèle simple coefficient `C_l(α)`

### 7.4 Reentry Heating (Advanced, Optional)
**Purpose:** Échauffement à la rentrée

- [ ] Flux thermique approximatif `∝ ρ · v_rel³`
- [ ] Accumulation de chaleur, seuil de destruction
- [ ] Effets visuels (Phase 11)

**Why this matters:** L'aéro transforme l'ascension et la rentrée en vrais défis. Le « max Q » et le freinage atmosphérique deviennent tangibles.

### Points d'intégration
- **Prérequis :** Phase 5 (vitesse + section transversale du vaisseau) · Phase 6 (ρ(h)) · Phase 4 (repère tournant → vitesse relative à l'atmosphère, pas la vitesse inertielle)
- **Fournit à :** Phase 8 (pression dynamique Q pour les limites structurelles et les calculs GNC) · la traînée crée un écart entre trajectoire réelle et prédiction conique de Phase 9 — écart attendu et normal
- **Dans la boucle :** **Force layer — étape 2** (F_drag accumulé avec gravité et poussée) · attention : force dépendant de la vitesse → traiter en semi-implicite (voir note technique 0.4)

### 7.x Intégration dans l'application
**Purpose:** Brancher la traînée dans la Force layer et valider contre la vitesse terminale analytique

- [ ] Brancher `AeroSystem` dans la Force layer (étape 2) après la gravité
- [ ] Calculer la vitesse relative à l'atmosphère tournante via `FrameManager` (Phase 4)
- [ ] Logger `F_drag`, `Q`, et le nombre de Mach à chaque pas
- [ ] Comparer la vitesse terminale simulée à la valeur analytique `v_t = √(2mg / (ρ·Cd·A))`
- [ ] **Success criteria :** Satellite en orbite basse freine et rentre en atmosphère de façon réaliste

---

## Phase 8: Guidance, Navigation & Control

**Goal:** Piloter automatiquement — culminant sur la descente/atterrissage propulsé

### 8.1 Attitude Control (PID)
**Purpose:** Pointer et maintenir le vaisseau dans une direction

- [ ] Créer `src/Control/AttitudeController.hpp`
- [ ] Contrôleur PID par axe pour annuler l'erreur d'orientation
- [ ] Sortie → commandes de gimbal + RCS
- [ ] Anti-emballement (integral windup clamp)

### 8.2 Autopilot Modes
**Purpose:** Modes de maintien d'attitude utiles

- [ ] Hold prograde / retrograde (le long de la vitesse)
- [ ] Hold radial in / out
- [ ] Hold normal / anti-normal
- [ ] Pointer vers une cible (autre vaisseau/corps)

### 8.3 Powered Descent / Landing
**Purpose:** Le problème Falcon 9 — poser le vaisseau en douceur

- [ ] Orienter retrograde par rapport à la surface
- [ ] Calculer l'altitude de début de combustion (suicide burn / hoverslam) à partir de vitesse, poussée dispo, gravité locale
- [ ] Contrôle de la manette pour annuler la vitesse verticale ≈ au contact du sol
- [ ] Annuler la vitesse horizontale (vitesse relative à la surface tournante)
- [ ] **Success criteria :** Le vaisseau se pose tout seul à < ~2 m/s sans intervention

> **Note technique :** Le « suicide burn » : combustion la plus tardive possible qui annule la vitesse pile au sol, pour minimiser le carburant. Calcule l'altitude de départ avec la décélération nette `(F/m − g)` et la vitesse courante. Garde une marge de sécurité au début.

### 8.4 Ascent Guidance (Optional)
**Purpose:** Monter en orbite automatiquement

- [ ] Gravity turn : inclinaison progressive selon l'altitude/vitesse
- [ ] Coupure moteur à l'apoapse cible, puis circularisation

**Why this matters:** C'est le payoff de la fusée : décoller, se mettre en orbite, et se reposer — le tout en émergeant de la physique.

### Points d'intégration
- **Prérequis :** Phase 5 (moteurs + RCS + état 6DOF complet) · Phase 6/7 (ρ et Q pour les calculs de descente et de drag) · Phase 9 (éléments orbitaux pour les modes d'attitude prograde/rétrograde/normal)
- **Fournit à :** Phase 5 (commandes throttle, gimbal, δ-RCS appliquées à chaque pas physique)
- **Dans la boucle :** le GNC peut tourner à une fréquence inférieure à la physique (ex. 10 Hz vs 60 Hz fixe) · ses commandes sont lues par la **Force layer — étape 2** au pas physique suivant

### 8.x Intégration dans l'application
**Purpose:** Brancher les contrôleurs GNC et valider l'atterrissage autonome

- [ ] Brancher `AttitudeController` : lit l'orientation courante, émet commandes gimbal/RCS à chaque cycle GNC
- [ ] Activer le mode prograde et vérifier la convergence de l'erreur d'orientation
- [ ] Implémenter la descente propulsée et valider vitesse d'impact < 2 m/s
- [ ] **Success criteria :** Atterrissage autonome reproductible sans intervention manuelle

---

## Phase 9: Orbital Mechanics & Trajectory Prediction

**Goal:** Calculer et afficher les orbites et trajectoires futures (la « vue carte »)

### 9.1 State ↔ Orbital Elements
**Purpose:** Convertir dans les deux sens

- [ ] Depuis `(r, v, μ)` → éléments orbitaux (a, e, i, Ω, ω, ν)
- [ ] Depuis éléments orbitaux → `(r, v)`
- [ ] Calculer apoapse, périapse, période, énergie spécifique

### 9.2 Trajectory Prediction
**Purpose:** Prédire le futur sans simuler pas à pas

- [ ] Propager la conique analytiquement pour tracer l'orbite future
- [ ] Coniques raccordées : prédire à travers les transitions de SOI
- [ ] **Gérer les trois types de coniques :** ellipse (`e < 1`), parabole (`e = 1`), hyperbole (`e > 1`) — un vaisseau entrant dans la SOI d'une lune est presque toujours en trajectoire hyperbolique relative à elle
- [ ] Vérifier avec l'équation vis-viva : `v² = μ·(2/r − 1/a)`

### 9.3 Maneuver Nodes
**Purpose:** Planifier des manœuvres

- [ ] Placer un nœud : Δv (prograde/normal/radial) à un instant donné
- [ ] Prédire l'orbite résultante
- [ ] Calcul d'approche la plus proche / interception avec une cible

> **Note technique :** La prédiction utilise des coniques analytiques (rapide, déterministe), pendant que le vol réel utilise l'intégration n-corps. Léger écart attendu près des points de Lagrange — c'est normal et acceptable.

**Why this matters:** Sans prédiction d'orbite, impossible de planifier des manœuvres. C'est ce qui rend une sim orbitale jouable.

### Points d'intégration
- **Prérequis :** Phase 3 (μ + hiérarchie SOI) · Phase 4 (repères pour les conversions état↔éléments) · Phase 3.2 étendu (solveur Kepler hyperbolique — e > 1 fréquent à l'entrée d'une SOI de lune, voir note 3.2)
- **Fournit à :** Phase 10 (éléments orbitaux → propagation analytique on-rails) · Phase 11 (éléments → lignes d'orbite à afficher) · Phase 8 (prograde/rétrograde/normal déduits de v et h)
- **Dans la boucle :** **hors boucle principale** — calcul à la demande (prédiction, vue carte, nœuds de manœuvre) · ne doit jamais bloquer un pas physique

### 9.x Intégration dans l'application
**Purpose:** Exposer les éléments orbitaux et la prédiction de trajectoire hors boucle physique

- [ ] Appeler la conversion `State → éléments orbitaux` depuis l'état courant entre deux pas physiques
- [ ] Propager la conique analytiquement et fournir les points au rendu (Phase 11)
- [ ] Placer un nœud de manœuvre et vérifier l'orbite résultante prédite
- [ ] Vérifier vis-viva à chaque point propagé : `v² = μ·(2/r − 1/a)`
- [ ] **Success criteria :** Prédiction correcte, nœuds de manœuvre fonctionnels, sans jamais bloquer un pas physique

---

## Phase 10: Time Warp

**Goal:** Accélérer le temps sans casser l'intégration numérique

### 10.1 The Problem
- [ ] Documenter : un grand `dt` fait diverger l'intégration n-corps
- [ ] Définir des niveaux de warp (1×, 10×, 100×, 1000×, 10000×, ...)

### 10.2 On-Rails Propagation
**Purpose:** Basculer le vaisseau en analytique pendant le warp

- [ ] En warp élevé : convertir l'état du vaisseau en éléments orbitaux et le propager analytiquement (conique deux-corps autour du corps dominant)
- [ ] Gérer les transitions de SOI pendant le warp (re-raccorder la conique)
- [ ] Revenir en intégration n-corps dès que le joueur reprend la main (physics warp bas)
- [ ] **Collision analytique pendant le warp :** la détection pas-à-pas (2.3) ne suffit plus — à 10000×, le vaisseau peut « traverser » un corps entre deux évaluations. Si le périapse de la conique courante est inférieur au rayon du corps, calculer l'instant exact d'impact (intersection conique/sphère) et arrêter le warp à cet instant.

### 10.3 Physics Warp (Low Levels)
**Purpose:** Garder la physique active aux warps faibles

- [ ] Aux warps faibles (≤ 4×), continuer l'intégration n-corps avec sous-pas
- [ ] Interdire le warp élevé en atmosphère ou sous poussée

> **Note technique :** C'est la deuxième raison du modèle hybride : la propagation analytique pendant le warp n'est possible que parce qu'on a déjà les éléments orbitaux et la structure deux-corps/SOI.

**Why this matters:** Une orbite peut durer des heures voire des années. Sans warp, impossible d'observer le système.

### Points d'intégration
- **Prérequis :** Phase 9 (éléments orbitaux du vaisseau pour la propagation) · Phase 3 (rails des corps pour les transitions SOI en warp) · Phase 2.3 étendu (collision analytique : si périapse < rayon du corps → stopper le warp à l'instant exact, voir note 10.2)
- **Fournit à :** rien en aval — feature utilisateur finale
- **Dans la boucle :** en warp élevé, **remplace les étapes 2–3** (force + intégration n-corps) par la propagation analytique de Phase 9 · en physics warp bas (≤ 4×), conserve les étapes 1–5 avec sous-pas multiples

### 10.x Intégration dans l'application
**Purpose:** Implémenter le switch warp dans SimulationLoop et valider la cohérence à la reprise

- [ ] Implémenter le switch : remplacer étapes 2–3 par propagation analytique au-dessus d'un seuil de warp
- [ ] Interdire le warp élevé en atmosphère ou sous poussée active
- [ ] Tester une transition SOI complète pendant le warp (re-raccordement de la conique)
- [ ] Valider la collision analytique : périapse < rayon → arrêt du warp à l'instant exact calculé
- [ ] **Success criteria :** Orbite propagée à 1000× restituée fidèlement à la reprise en n-corps, état cohérent

---

## Phase 11: Rendering & Visualization

**Goal:** Visualiser le système, les trajectoires et le vol

### 11.1 Core Rendering
**Purpose:** Afficher corps, vaisseau, traînées

- [ ] 2D d'abord (validation), puis 3D
- [ ] Rendu des corps (sphères texturées, à l'échelle ou en scaled space)
- [ ] Rendu du vaisseau
- [ ] Intégration de l'origine flottante (Phase 4)

### 11.2 Orbit & Trajectory Lines
**Purpose:** Tracer les orbites prédites

- [ ] Lignes d'orbite depuis les éléments orbitaux (Phase 9)
- [ ] Marqueurs apoapse/périapse, nœuds de manœuvre
- [ ] Traînée historique du vaisseau

### 11.3 Map View vs Flight View
**Purpose:** Deux modes d'affichage

- [ ] Vue carte : système entier, orbites, planification
- [ ] Vue vol : proche du vaisseau, atmosphère, sol
- [ ] Caméra orbitale autour du vaisseau / corps

### 11.4 HUD
**Purpose:** Télémétrie

- [ ] Altitude, vitesse (surface + orbitale), apoapse/périapse, période
- [ ] Carburant, Δv restant, manette des gaz
- [ ] Pression dynamique Q, nombre de Mach

### 11.5 Atmospheric & Reentry Effects (Optional)
- [ ] Dégradé atmosphérique (ciel), effet de rentrée (plasma) lié au flux thermique

> **Note technique :** À terme, le rendu pourrait s'appuyer sur VoxelEngine — mais ses besoins (double précision, origine flottante, échelles astronomiques) diffèrent d'un moteur de jeu classique. Recommandation : rendu standalone simple d'abord, intégration éventuelle plus tard.

**Why this matters:** En tant que graphics programmer, c'est ton terrain — mais garde le rendu découplé de la physique.

### Points d'intégration
- **Prérequis :** Phase 4 (floating origin → conversion float64→float32 avant tout vertex) · Phase 9 (éléments orbitaux → lignes d'orbite) · Phase 0 (state interpolé via `alpha` pour lisser le rendu entre deux pas physiques)
- **Fournit à :** rien en aval — sortie finale vers l'utilisateur
- **Dans la boucle :** **Interpolation layer — étape 6** : lit les deux états `t₋₁` et `t`, interpole avec `alpha`, puis envoie au GPU · la boucle de rendu tourne à son propre framerate, totalement découplée

### 11.x Intégration dans l'application
**Purpose:** Initialiser la fenêtre et brancher le rendu découplé sur la boucle physique

- [ ] Initialiser la fenêtre SDL2/bgfx et la boucle de rendu à son propre framerate
- [ ] Lire `alpha` depuis `SimulationLoop`, interpoler `State` pour lisser le rendu entre deux pas physiques
- [ ] Brancher l'origine flottante (Phase 4) avant toute conversion float64→float32
- [ ] Tracer les lignes d'orbite depuis les éléments orbitaux (Phase 9)
- [ ] Afficher le HUD (altitude, vitesse, carburant, apoapse/périapse)
- [ ] **Success criteria :** Vaisseau visible, orbite tracée, HUD mis à jour — rendu entièrement découplé de la physique

---

## Phase 12: Sandbox & Editor Tools

**Goal:** Outils pour expérimenter et déboguer

### 12.1 Vessel Builder (Optional)
- [ ] Assembler un vaisseau depuis des pièces (étages, moteurs, réservoirs)
- [ ] Calcul automatique masse/Δv/TWR (rapport poussée/poids)

### 12.2 Debug Visualization
- [ ] Vecteurs de force (gravité, poussée, traînée) en temps réel
- [ ] Affichage des sphères d'influence
- [ ] Graphe de dérive d'énergie/moment cinétique
- [ ] Inspecteur d'état (position, vitesse, éléments orbitaux live)

### 12.3 Scenario System
- [ ] Charger des scénarios (en orbite, sur la rampe, en approche d'atterrissage)
- [ ] Sauvegarde/chargement de l'état complet (déterministe)

**Why this matters:** Itérer vite sur les modes de vol sans tout relancer à zéro.

### Points d'intégration
- **Prérequis :** Phase 0 (déterminisme — save/load = snapshot de l'état canonique, reproductible au bit près) · Phase 9 + 11 (vue carte + inspecteur d'état pour les outils de debug)
- **Fournit à :** rien en aval physique — outils éditeur uniquement
- **Dans la boucle :** **hors boucle principale** — les outils lisent l'état entre les pas et n'écrivent que via les commandes normales (throttle, chargement de scénario) ; jamais d'écriture directe dans l'état canonique

### 12.x Intégration dans l'application
**Purpose:** Brancher les outils de debug et le système de scénarios sur l'état canonique

- [ ] Brancher Dear ImGui pour l'inspecteur d'état live (position, vitesse, éléments orbitaux)
- [ ] Afficher les vecteurs de force (gravité, poussée, traînée) en temps réel via ImGui
- [ ] Implémenter save/load de l'état complet et vérifier la reproductibilité bit-à-bit
- [ ] Charger un scénario depuis fichier et rejouer depuis cet état
- [ ] **Success criteria :** Scénario sauvegardé, rechargé et rejoué de façon bit-identique

---

## Validation & Testing Strategy

**Goal:** Garantir que la physique est correcte, pas juste plausible

### Analytical Ground Truths
**Purpose:** Comparer la sim à des solutions exactes connues

- [ ] **Chute libre sans air** → parabole analytique exacte
- [ ] **Orbite circulaire** → reste circulaire, rayon constant
- [ ] **Période orbitale** → matche `T = 2π√(a³/μ)`
- [ ] **Énergie spécifique** `ε = v²/2 − μ/r = −μ/(2a)` → constante
- [ ] **Moment cinétique** `h = r × v` → constant
- [ ] **Vis-viva** : `v² = μ·(2/r − 1/a)` vérifiée à tout point de l'orbite
- [ ] **Deux-corps** → matche la conique analytique
- [ ] **Vitesse terminale** (avec traînée) → matche la valeur analytique
- [ ] **Δv en poussée** → matche Tsiolkovsky `Δv = Isp·g0·ln(m₀/m₁)`

> **Note technique :** Une sim qui affiche de jolies trajectoires peut être totalement fausse. Ces vérités-terrain sont le seul moyen de savoir si ta physique est juste ou si ton rendu montre juste de belles erreurs.

### Unit Tests
- [ ] Ajouter un framework de test (gtest)
- [ ] `tests/Integrators/` — dérive d'énergie par intégrateur sur orbite circulaire
- [ ] `tests/Kepler/` — solveur de l'équation de Kepler (convergence, cas limites e→1)
- [ ] `tests/Frames/` — transformations aller-retour (identité)
- [ ] `tests/Atmosphere/` — profil de densité, continuité aux couches
- [ ] `tests/Orbital/` — conversions état ↔ éléments orbitaux (aller-retour)

### Integration & Determinism Tests
- [ ] Orbite complète : énergie/moment conservés sur N tours
- [ ] Descente complète : atterrissage doux reproductible
- [ ] **Déterminisme** : mêmes entrées → état final identique au bit près (fixed timestep)

### Performance Tests
- [ ] Coût de la gravité n-corps avec N corps
- [ ] Throughput de la propagation analytique pendant le warp

**Why this matters:** Refactorer une sim sans ces tests, c'est piloter à l'aveugle. La dérive numérique est silencieuse.

---

## Technical Decisions

### Numerical Integration
- **Choix : Velocity Verlet / Leapfrog (symplectique)**
  - Conserve l'énergie sur le long terme → orbites fermées stables
  - RK4 disponible pour comparaison mais PAS pour les orbites longues (dérive)
  - Euler semi-implicite gardé comme référence pédagogique

**Pourquoi :** Le symplectique est non négociable pour une sim orbitale. C'est la décision la plus structurante.

### Gravity Model
- **Choix : Hybride — corps sur rails képlériens, vaisseau en n-corps sommé**
  - Système solaire stable et fidèle (rails analytiques)
  - Orbites émergentes, points de Lagrange, perturbations côté vaisseau
  - Permet le time warp via propagation analytique

**Pourquoi :** Le n-corps complet sur les planètes dérive et déstabilise le système. C'est le compromis d'Orbiter/KSP.

### Precision
- **Choix : `float64` pour toute la physique, origine flottante pour le rendu**
  - `float32` insuffisant aux distances astronomiques (tremblement)
  - Conversion en simple seulement après soustraction de l'origine lgèbre linéaire, tenseur d'inertie)
- **Recommandation :** GLM pour rester cohérent avec VoxelEngine et limiter la charge mentale en alternant les projets

### Relationship to VoxelEngine
- Projet **standalone** au départ (besoins de précision/échelle trop différents d'un moteur de jeu)
- Réutilise les **patterns** (ServiceLocator, fixed timestep, manager lifecycle, structure de dépôt) pour que passer d'un projet à l'autre soit fluide
- Intégration du rendu via VoxelEngine envisageable plus tard, pas une priorité

---

## Priority Order (Next Steps)

**Suis cet ordre exact pour démarrer :**

### Immediate Priority (v0.1.0 — Sim Core)
1. **State representation** — le vecteur d'état (Phase 0.1)
2. **Fixed timestep loop** — la boucle accumulateur (Phase 0.2)
3. **TimeManager** — temps fixe + base du warp (Phase 0.3)
4. **Integrators** — Euler + Velocity Verlet (Phase 0.4)
5. **Constants & units** — SI, μ, logger (Phase 0.5)

### Short-term Priority (v0.1.0 — First Orbit)
6. **Single body + particle** — gravité d'un corps (Phase 1.1)
7. **Integrator comparison** — voir Euler dériver, Verlet tenir (Phase 1.2)
8. **Conservation checks** — énergie, moment, période (Phase 1.3)
9. **Minimal 2D rendering** — *l'orbite à l'écran* (motivation !) (Phase 1.4)

### Medium-term Priority (v0.2.0 — Solar System)
10. **Gravity accumulation** — somme n-corps sur le vaisseau (Phase 2.1)
11. **Hybrid model** — décision rails vs n-corps (Phase 2.2)
12. **Collision detection** — surface des corps (Phase 2.3)
13. **CelestialBody + Keplerian rails** — corps analytiques (Phase 3.1-3.2)
14. **SOI + data-driven loading** — système solaire réel depuis JSON (Phase 3.3-3.4)
15. **Coordinate frames + floating origin** — la précision (Phase 4)

### Long-term Priority (v0.3.0 — The Vessel)
16. **Rigid body 6DOF** — orientation, inertie (Phase 5.1)
17. **Mass properties + propulsion** — masse variable, moteurs (Phase 5.2-5.3)
18. **Staging + RCS** — étages, attitude (Phase 5.4-5.5)
19. **Validation: Tsiolkovsky, vis-viva** — tests analytiques (Validation)

### Feature Complete (v0.4.0 — Atmosphere & Flight)
20. **Atmosphere model (multi-layer)** — densité, composition (Phase 6)
21. **Aerodynamics** — traînée, Q, vitesse relative (Phase 7)
22. **Attitude control (PID)** — maintien d'orientation (Phase 8.1-8.2)
23. **Powered descent / landing** — *l'atterrissage autonome* (Phase 8.3)

### Map & Time (v0.5.0)
24. **State ↔ orbital elements** — conversions (Phase 9.1)
25. **Trajectory prediction** — coniques raccordées (Phase 9.2)
26. **Maneuver nodes** — planification (Phase 9.3)
27. **Time warp** — propagation analytique (Phase 10)

### Polish & Tools (v0.6.0+)
28. **Rendering: map/flight view, HUD, orbit lines** (Phase 11)
29. **Ascent guidance** — mise en orbite auto (Phase 8.4)
30. **Sandbox, debug viz, scenarios** (Phase 12)
31. **Reentry heating + effects** (Phase 7.4 / 11.5)

### GPU Particle Field (v0.6.0+, optionnelle — hors chemin critique)
> Prérequis : cœur validé (Phase 1-2) + origine flottante (Phase 4) + rendu de base (Phase 11.1). Branchement : Phase 2 côté physique, Phase 11 côté rendu.
32. **Scope & isolation** — frontière déterminisme, `GPUParticleSystem` (Phase 13.1)
33. **Buffers GPU + interop** — SoA float32, VBO partagé (Phase 13.2)
34. **Kernel passif** — champ qui orbite les corps sur rails, le « week-end » (Phase 13.3)
35. **Spawning data-driven** — ceinture, anneaux, depuis JSON des corps (Phase 13.5)
36. **Rendu interop** — point sprites sans round-trip CPU (Phase 13.7)
37. **Validation déterminisme** — test « GPU on/off = état identique » (Phase 13.8)
38. **Kernel mutuel n-body** — tuiles shared memory, la vitrine (Phase 13.4)

---

## Build System

### Dependencies to Add Progressively

**Phase 0-1 (v0.1.0 — Core & First Orbit):**
- [ ] GLM (avec `dvec3`/`dquat` double précision) — maths vecteurs/quaternions
- [ ] Un framework de rendu 2D simple (SDL2, ou réutiliser SDL2+bgfx du game engine)

**Phase 3 (v0.2.0 — Solar System):**
- [ ] nlohmann-json — chargement data-driven des corps et atmosphères

**Testing (dès v0.1.0) :**
- [ ] Google Test (gtest) — tests unitaires (commencer tôt avec les conservation checks)

**Phase 11 (v0.6.0 — Rendering 3D) :**
- [ ] bgfx (+ bx, bimg) — rendu 3D multiplateforme (cohérent avec VoxelEngine)
- [ ] Dear ImGui — HUD debug et inspecteurs

**Phase 13 (v0.6.0+ — GPU Particle Field, optionnelle) :**
- [ ] CUDA Toolkit (nvcc, runtime) — kernels GPU
- [ ] Interop CUDA ↔ rendu : OpenGL (`cudaGraphicsGLRegisterBuffer`) ou équivalent bgfx
- [ ] *(Optionnel)* Thrust — réductions, tri, stream compaction sans réécrire les primitives

**Optional / Advanced :**
- [ ] Eigen — si besoin d'algèbre linéaire avancée (tenseurs d'inertie complexes)
- [ ] Tracy — profiling

---

## Phase 13: GPU Particle Field (CUDA)

**Goal:** Ajouter un champ de particules massif (ceinture d'astéroïdes, anneaux planétaires, nuage de débris) simulé sur GPU — couche purement visuelle et vitrine technique, séparée du cœur déterministe. Optionnelle et parallèle au chemin principal : à attaquer une fois le cœur CPU validé (Phase 1-2), pour disposer du n-corps CPU comme référence de correction.

### 13.1 Scope & Separation from Core
**Purpose:** Définir ce que le GPU simule, et surtout ce qu'il ne simule PAS

- [ ] Champ de particules = couche visuelle/expérimentale, HORS de la vérité canonique
- [ ] Couplage à sens unique : les particules ressentent les corps, mais n'influencent jamais le vaisseau ni les corps
- [ ] Aucune garantie de déterminisme bit-à-bit sur cette couche (assumé explicitement)
- [ ] Manager dédié `GPUParticleSystem`, lifecycle séparé (même pattern que les autres managers)
- [ ] Le cœur CPU float64 reste la seule référence ; activer/désactiver le GPU ne change rien à l'état canonique

> **Note technique :** C'est la raison d'être de l'isolation. Le float GPU n'est pas déterministe gratuitement (ordre de réduction variable, atomics, fast-math, résultats dépendants du hardware). Mélanger ça au cœur saboterait la reproductibilité bit-à-bit. On confine donc le non-déterminisme à une couche qui le tolère — parce qu'elle est purement cosmétique.

**Why this matters:** Tout repose sur cette frontière. Tant que le GPU n'écrit jamais dans l'état canonique, on garde le cœur déterministe ET on s'offre une vitrine GPU.

### 13.2 Particle State & GPU Buffers
**Purpose:** Représenter les particules en mémoire device

- [ ] Structure of Arrays (SoA) : positions et vitesses en buffers séparés (accès coalescés)
- [ ] `float32`, en coordonnées relatives à l'origine flottante (Phase 4) — PAS `float64`
- [ ] Allocation device ; buffers ping-pong (lecture/écriture séparées) ou intégration in-place selon l'intégrateur
- [ ] Interop CUDA ↔ rendu (OpenGL/bgfx) : buffer partagé (VBO) pour éviter le round-trip device→host→device

> **Note technique :** fp32 suffit ici parce que les particules sont locales, exprimées relativement à l'origine flottante (la précision astronomique est déjà absorbée par le rebasing). C'est aussi un choix de débit : sur les cartes grand public, le fp64 est fortement bridé par rapport au fp32.

### 13.3 Gravity Kernel — Passive Particles
**Purpose:** Le cas simple et trivialement parallèle — les particules ressentent les corps, pas entre elles

- [ ] Kernel : un thread = une particule ; sommer `aᵢ = −μᵢ · (r − rᵢ).normalized() / |r − rᵢ|²` sur les corps sur rails
- [ ] Positions/μ des corps passés en mémoire constante (`__constant__`) — petit nombre, lus en broadcast par tous les threads
- [ ] Intégration symplectique par particule (velocity Verlet) dans le kernel, même schéma que le cœur
- [ ] Complexité O(n) en particules — embarrassingly parallel

> **Note technique :** `__constant__` memory est idéale ici : ~50 corps lus par ~1M threads, c'est exactement le motif broadcast pour lequel le cache constant est optimisé. Charger les corps via mémoire globale gaspillerait de la bande passante.

**Why this matters:** 80 % de l'effet visuel (anneaux, ceinture qui orbite) vient de ce kernel seul. C'est le « week-end » — fais-le marcher avant 13.4.

### 13.4 Gravity Kernel — Mutual N-body (Optional)
**Purpose:** Les particules s'attirent entre elles — le vrai morceau CUDA

- [ ] Tiling avec mémoire partagée (`__shared__`) : charger des blocs de particules en shared, chaque thread accumule contre la tuile courante
- [ ] Réduit drastiquement les accès mémoire globale (le goulot du n-body naïf)
- [ ] O(n²) mais transformé de memory-bound en compute-bound par les tuiles ; Barnes-Hut O(n log n) pour passer à l'échelle supérieure
- [ ] Softening obligatoire (réutiliser le paramètre de Phase 2.1) pour éviter les singularités

> **Note technique :** Le kernel naïf est limité par la bande passante mémoire. En chargeant une tuile de p particules en shared et en la réutilisant pour tout le bloc, on amortit chaque lecture globale sur p threads et le kernel redevient compute-bound — là où le GPU brille.

**Why this matters:** C'est le morceau qui impressionne vraiment en entretien. Mais il dépend d'un cœur validé et d'un kernel passif (13.3) qui marche.

### 13.5 Field Sourcing & Spawning
**Purpose:** Générer le champ depuis des paramètres

- [ ] Ceinture : distribution sur un anneau (a_min..a_max), excentricités/inclinaisons faibles
- [ ] Anneaux planétaires : autour d'un corps, dans son plan équatorial (lié à l'obliquité, Phase 3.5)
- [ ] Vitesses initiales = vitesse circulaire locale `v = √(μ/r)` + dispersion contrôlée
- [ ] Data-driven : décrire les champs (corps hôte, bornes, densité, dispersion) dans le JSON des corps (Phase 3.4)

> **Note technique :** Initialise chaque particule à sa vitesse orbitale locale. À vitesse nulle, tout le champ tombe sur le corps au premier pas ; trop vite, tout s'éjecte. La dispersion donne l'épaisseur réaliste de l'anneau/ceinture.

### 13.6 Lifecycle & Collision Handling (Optional)
**Purpose:** Gérer la disparition des particules

- [ ] Particule sous le rayon d'un corps → despawn (marquée morte)
- [ ] Cull des particules trop éloignées de la zone d'intérêt
- [ ] Pas de transition SOI fine — inutile pour une couche visuelle
- [ ] Stream compaction si recyclage des particules mortes nécessaire (garder une densité constante)

> **Note technique :** Un simple test de rayon + un cull de distance suffisent. La compaction n'est utile que si tu veux respawner activement.

### 13.7 Rendering Integration
**Purpose:** Afficher le champ sans jamais quitter le GPU

- [ ] Point sprites / instanced rendering directement depuis le buffer GPU (interop CUDA↔GL, zéro copie)
- [ ] Intégration origine flottante (Phase 4) : soustraire l'origine dans le vertex shader
- [ ] LOD : réduire la densité affichée pour les champs lointains / en scaled space (Phase 4.3)

> **Note technique :** Tout l'intérêt de l'interop CUDA↔GL : les positions sont calculées par le kernel et lues par le rendu sans jamais transiter par le CPU. La particule naît, vit et s'affiche sans quitter la VRAM.

### 13.8 Validation & Determinism Boundary
**Purpose:** Vérifier la couche GPU sans entamer les garanties du cœur

- [ ] Comparer le kernel passif GPU vs la gravité CPU sur un petit jeu de particules : trajectoires identiques à la tolérance fp32 près
- [ ] **Test de non-régression critique :** l'état du cœur doit rester bit-identique, que le champ GPU soit actif ou non
- [ ] Conservation d'énergie du champ mutuel (avec softening — dérive attendue plus forte qu'en CPU fp64, c'est normal)
- [ ] Throughput : nombre de particules à 60 Hz pour le kernel passif et le kernel mutuel

> **Note technique :** Le test le plus important de toute la phase n'est pas sur le GPU mais sur le cœur : prouver que « GPU on » et « GPU off » donnent le MÊME état final canonique au bit près. Tant que ce test passe, la frontière de déterminisme tient et la couche GPU est « gratuite » du point de vue de la correction.

**Why this matters:** Une vitrine GPU qui casserait silencieusement le déterminisme serait un piège. Ce test garantit que la vitrine reste additive, jamais intrusive.

### Points d'intégration
- **Prérequis :** Phase 3 (positions + μ des corps → mémoire constante GPU `__constant__`) · Phase 4 (floating origin → coordonnées locales fp32 déjà en précision locale pour les particules) · Phase 11.1 (rendu de base en place → interop CUDA↔GL pour le buffer partagé)
- **Fournit à :** Phase 11 (buffer GPU → rendu direct par point sprites, zéro round-trip CPU)
- **Dans la boucle :** `GPUParticleSystem::update()` lancé **après Bodies layer — étape 1** (positions de corps à jour) et avant le rendu · couplage à sens unique — jamais d'écriture dans l'état canonique (voir note 13.1)

### 13.x Intégration dans l'application
**Purpose:** Brancher GPUParticleSystem dans la boucle et valider la frontière de déterminisme

- [ ] Instancier `GPUParticleSystem` et l'appeler après Bodies layer (étape 1)
- [ ] Passer positions + μ des corps en mémoire constante GPU (`__constant__`)
- [ ] Brancher l'interop CUDA↔GL pour le rendu direct depuis le buffer GPU
- [ ] Exécuter le test non-régression : état canonique bit-identique avec GPU on et GPU off
- [ ] **Success criteria :** Ceinture/anneaux affichés à 60 Hz, déterminisme du cœur intact

---

## Notes & Best Practices

### Development Workflow
- **Commence en 2D** — valide tout le cœur (boucle, intégrateur, conservation) avant la 3D
- **Valide contre l'analytique en continu** — chaque nouvelle force a une vérité-terrain
- **Commits atomiques** — un commit par feature
- **Tests tôt** — les conservation checks dès la Phase 1
- **Profile, mais pas prématurément** — la direct summation suffit pour un système solaire

### Simulation Guidelines
- **Fixed timestep sacré** — jamais de `dt` variable dans la physique (casse le déterminisme)
- **Symplectique pour les orbites** — RK4 seulement pour comparer
- **Double précision physique, simple précision rendu** — toujours via origine flottante
- **Corps sur rails, vaisseau en n-corps** — ne jamais simuler les planètes en n-corps complet
- **Sépare sim et rendu** — le cœur physique doit tourner et se tester sans fenêtre
- **`μ` directement** — plus précis que `G·M`

### Debugging Tips
- **Trace l'énergie et le moment cinétique** — leur dérive révèle les bugs d'intégration
- **Visualise les vecteurs de force** — gravité/poussée/traînée en temps réel
- **Compare deux intégrateurs côte à côte** — pour isoler erreur d'intégration vs erreur de modèle
- **Scénarios reproductibles** — sauvegarde l'état au moment d'un bug (déterminisme aide)

### Conceptual Reminders
- **Une orbite = chute permanente + vitesse latérale suffisante pour rater le sol**
- **Trop lent → s'écrase · bonne vitesse → orbite · trop rapide → s'échappe**
- **Le modèle de forces est le seul morceau qui change selon le sous-domaine** — le reste (boucle, intégrateur, état, repères) est universel

### Documentation Resources
- **GLM**: https://github.com/g-truc/glm
- **bgfx**: https://bkaradzic.github.io/bgfx/
- **Orbital mechanics (vis-viva, Kepler)**: tout cours de mécanique spatiale ; voir aussi les notes d'implémentation d'Orbiter
- **Symplectic integrators**: chercher « velocity Verlet » et « leapfrog energy conservation »

---

**Roadmap complète. Commence par la Phase 0, vise une orbite 2D stable le plus vite possible (Phase 1.4), puis empile les couches. Bonne construction !**