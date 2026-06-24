# ProNav Interception Simulator

An interactive, single‑file simulator of a **proportional‑navigation (PN) guided interceptor** chasing a **manually‑flown evading drone**, in a 2‑D plane. It models the full guidance loop — seeker → estimator → autopilot → airframe → aerodynamics — with enough fidelity to build real intuition about *why* intercepts succeed or fail, while staying light enough to run in a browser tab from a single HTML file.

**Live:** https://gitmrlu.github.io/pronav-sim/
**Run locally:** just open `index.html` in any browser (no build, no server, no dependencies).

---

## Table of contents

- [What you're looking at](#what-youre-looking-at)
- [How to use it](#how-to-use-it)
- [The core idea: proportional navigation](#the-core-idea-proportional-navigation)
- [Guidance laws](#guidance-laws)
- [Parameter reference](#parameter-reference)
- [The seeker & estimation chain](#the-seeker--estimation-chain)
- [Outputs: charts, readouts, replay, CSV](#outputs-charts-readouts-replay-csv)
- [Modeled under the hood (things you don't directly see)](#modeled-under-the-hood-things-you-dont-directly-see)
- [Deliberate simplifications & fidelity limits](#deliberate-simplifications--fidelity-limits)
- [Glossary](#glossary)

---

## What you're looking at

The screen has four regions:

| Region | What it shows |
|---|---|
| **Top‑down engagement** (center) | Bird's‑eye view of the plane. The **interceptor** (cyan dart, with a motor flame while boosting) chases the **target** (yellow diamond). Trails, the line‑of‑sight (LOS), and velocity vectors are drawn. A small HUD top‑right shows run speed (`▸ 1.0×`) and the sim clock. |
| **Interceptor seeker FOV** (center‑right) | What the missile's seeker "sees": a field‑of‑view cone with the target as a blip offset from boresight, plus a `LOCKED` / `ACQUIRING TRACK` / `LOCK LOST` status. |
| **Time‑series telemetry** (right column) | Live strip charts: acceleration (G), LOS rate (raw vs. Kalman vs. true), range & closing velocity, and seeker bearing offset. |
| **Readout strip + results** (bottom) | Live numeric readouts (with bar gauges showing % of each limit), then a results panel + replay scrubber after the run. |

---

## How to use it

1. **Adjust parameters** in the left panel (everything has a sensible default; tweaks re‑arm the scenario live).
2. Click **Arm / Reset**.
3. **Click anywhere on the engagement view to launch.** Then **move the mouse over that view to fly the drone** — no need to hold the button. The drone chases your cursor; how far the cursor is from the drone sets how hard it maneuvers (capped by its max‑G).
4. Watch it play out. At **1× run speed it's real‑time (very fast)** — drop the run‑speed slider (presets `0.05× … 1×`) to get wall‑clock time to fly the drone manually. The physics are identical at any run speed; only playback is slowed.
5. After the run, read the **results**, scrub the **replay** timeline, or **Export CSV**.

The drone is *holonomic* — it can thrust in any direction up to its max‑G: turn, speed up, brake, even reverse and stop. (It's a multirotor, not a rocket.)

---

## The core idea: proportional navigation

Proportional navigation is the guidance law used by virtually all real homing missiles. The principle is deceptively simple:

> **Turn at a rate proportional to the rotation rate of the line of sight to the target.**

The **line of sight (LOS)** is the imaginary line from the interceptor to the target. Its angle is **λ** (lambda); how fast that angle rotates is the **LOS rate, λ̇** (lambda‑dot).

The key insight from the **collision triangle**: if two objects are on a collision course, the *bearing to the other object stays constant* — the LOS doesn't rotate (`λ̇ = 0`). If the LOS *is* rotating, you're not on a collision course. So PN simply drives `λ̇` toward zero:

```
commanded acceleration = N · Vc · λ̇
```

- **N** — the *navigation constant* (typically 3–5). Higher N nulls the LOS rate more aggressively.
- **Vc** — the *closing velocity* (how fast the range is shrinking).
- **λ̇** — the measured LOS rate.

This is elegant because the missile never needs to know the target's future position — it just keeps the bearing from drifting. It naturally produces *lead pursuit* (aiming ahead of the target) rather than tail‑chasing.

**Watch this in the sim:** the **LOS rate chart** is the heart of it. A clean intercept drives the rate toward zero near the end. When you jink the drone hard, you spike the LOS rate, forcing the interceptor to pull more G to null it again — and if it can't (G limit, low airspeed, or it runs out of energy), you escape.

---

## Guidance laws

Selectable in the **Interceptor → Guidance law** dropdown:

| Law | Formula | Notes |
|---|---|---|
| **Pure PN** (default) | `a = N · Vm · λ̇`, ⊥ to **missile velocity** | Uses missile speed `Vm`. Acceleration is applied perpendicular to the missile's own velocity. Robust and simple; doesn't need closing‑velocity info. |
| **True PN** | `a = N · Vc · λ̇`, ⊥ to the **LOS** | Uses closing velocity `Vc`. The command is perpendicular to the line of sight; the sim projects it onto the turn axis (so it correctly loses effectiveness at large look angles). |
| **Augmented PN (APN)** | `a = N · Vc · λ̇ + ½ N · a_T⊥` | Adds a term for the *target's* acceleration (`a_T⊥`, perpendicular to LOS), estimated by the Kalman filter. Much better against a hard‑maneuvering target — at the cost of needing a target‑acceleration estimate. |

The difference between Pure and True PN is **not just a gain** — they command acceleration in different directions (perpendicular to velocity vs. perpendicular to LOS). The sim models this correctly, so they behave differently at large off‑boresight angles.

---

## Parameter reference

### Target (FPV drone)
| Parameter | Default | Meaning |
|---|---|---|
| **Initial speed** | 30 m/s | Drone speed at the start. |
| **Max speed** | 50 m/s | Top speed; it won't exceed this in any direction. |
| **Max accel** | 10 G | Max thrust the drone can apply in *any* direction — turn, accelerate, brake, reverse, or stop. This is the hard limit on how tightly you can fly it. |
| **Airframe lag τ** | 0.03 s | First‑order lag between your stick input and the drone's actual response. |

### Engagement
| Parameter | Default | Meaning |
|---|---|---|
| **Initial range** | 1000 m | Starting separation (down to 50 m). |
| **Initial offset** | 0° | Angular offset of the target from the interceptor's boresight at launch. Non‑zero starts the engagement off‑axis, with an initial seeker bearing error. |

### Interceptor
| Parameter | Default | Meaning |
|---|---|---|
| **Axial accel (boost)** | 25 G | Forward thrust acceleration during the motor burn. The missile **launches from rest** (0 m/s). |
| **Burn time** | 1 s | How long the motor burns. After burnout it **coasts**, bleeding speed to drag. |
| **Nav constant N** | 4 | The PN gain (see above). |
| **Max lateral G** | 15 G | Structural turn limit. Note: at low airspeed the *achievable* G is further limited (see aero authority below). |
| **Airframe lag τ** | 0.10 s | First‑order autopilot/airframe response lag. |
| **Guidance law** | Pure PN | See [Guidance laws](#guidance-laws). |

### Seeker
| Parameter | Default | Meaning |
|---|---|---|
| **Seeker frame rate** | 240 Hz | How often the seeker produces a measurement. |
| **Processing latency** | 10 ms | Delay from a measurement to a usable guidance command. |
| **Angle noise (1σ)** | 1 mrad | Gaussian noise on the measured LOS angle — the noise the filter must reject. |
| **Filter agility** | 45 | Kalman process noise. Low = smooth but laggy λ̇; high = responsive but noisier. |
| **Closing‑vel noise** | 20 m/s | 1σ noise on the closing‑velocity estimate. *(Hidden under Pure PN, which doesn't use closing velocity.)* |
| **FOV / gimbal limit** | 50° | If the target leaves this half‑cone, the seeker loses lock and the missile coasts. |
| **LOS‑rate filter** | Kalman | Estimator for λ̇ — Kalman (3‑state), α–β, or raw finite‑difference. |
| **Ideal sensors** | off | When on, guidance uses *perfect* λ̇ / Vc / target accel (no noise, no filter, no lag) — for A/B comparison. |
| **Effective seeker lag** | (derived) | Read‑only: steady‑state lag (latency + 1 frame) plus the one‑time track‑acquisition delay. |

### Energy / drag
| Parameter | Default | Meaning |
|---|---|---|
| **Parasitic drag** | 12 m/s² | Form/skin drag deceleration *at the 300 m/s reference speed*; scales with **V²** (dynamic pressure). |
| **Induced drag** | 0.02 ·G² | Drag from pulling G; scales with **load²** and with **1/V²** (worse when slow). Calibrated at 300 m/s. |
| **Max flight time** | 15 s | Hard cutoff; the run ends as a miss if the interceptor hasn't connected by then. |

### Simulation
| Parameter | Default | Meaning |
|---|---|---|
| **Hit radius** | 5 m | Closest approach below this counts as an intercept (computed sub‑step — no tunneling). |
| **Run speed** | 1.0× | Playback speed. 1× is real‑time; lower it to fly manually. Physics are exact at any setting. |

---

## The seeker & estimation chain

A real missile doesn't know the true LOS rate — it has to *estimate* it from a noisy, discretely‑sampled seeker. The sim models that whole chain:

1. **Sampling** — the seeker produces a measurement every `1/frame‑rate` seconds (zero‑order hold between samples).
2. **Angle noise** — each measurement has Gaussian noise (1σ set by *Angle noise*).
3. **FOV / gimbal limit** — if the target is outside the seeker's cone, the measurement is invalid → lock is lost → the missile coasts straight.
4. **Processing latency** — measurements aren't usable until a fixed delay has passed.
5. **LOS‑rate filter** — estimates λ̇ from the noisy angle stream:
   - **Raw finite‑difference**: `(λ₂ − λ₁)/Δt`. Simple but amplifies noise badly (differentiating noise). Try it to see why filtering matters.
   - **α–β filter**: a lightweight constant‑velocity tracker.
   - **Kalman (default)**: a 3‑state near‑constant‑acceleration filter `[λ, λ̇, λ̈]`. It rejects the angle noise and also yields a **target‑acceleration estimate** (used by APN). *Filter agility* sets its process noise (smoothness vs. responsiveness).
6. **Track acquisition delay** — a filter needs **multiple readings** before its estimate is trustworthy (Kalman: 5, α–β: 3, raw: 2). Guidance won't act until then; the seeker shows **`ACQUIRING TRACK`** during warm‑up.

**Why differentiating noise is the lesson here:** the raw LOS‑rate estimate is `Δangle / Δt`. With a small `Δt` (high frame rate), even tiny angle noise produces huge rate noise, which would slam the commanded G against its limit. The **LOS‑rate chart plots all three** — raw (gray, jumpy), Kalman (purple, smooth), and true (white dashed) — so you can see the filter doing its job. Without filtering, the commanded acceleration is unusable bang‑bang; with it, the missile flies smoothly.

**Effective seeker lag.** The sidebar shows the *steady‑state* delay = **processing latency + one seeker frame** (a converged filter adds no extra steady‑state lag — it predicts forward to the current measurement time). Separately it reports the **one‑time acquisition** delay (the warm‑up readings). At 240 Hz / 10 ms latency that's ~14 ms steady‑state and ~21 ms acquisition.

---

## Outputs: charts, readouts, replay, CSV

- **Live readout strip** with bar gauges showing each value as a % of its limit (commanded vs. achieved G, target G, target speed, seeker offset vs. FOV).
- **Closest approach** is reported as the *true continuous* perpendicular miss distance (computed analytically within each step), for both hits and misses — independent of the hit‑radius setting. Results show it in metres and as a fraction of the hit radius.
- **Results panel**: outcome, CPA, time of flight, peak commanded/achieved/target G, whether the command saturated the structural limit, whether lock was lost, peak/final speeds, etc.
- **Replay scrubber** to step back through the recorded engagement frame by frame.
- **Export CSV** — every internal physics step (positions, velocities, LOS, raw/filtered/true λ̇, closing velocity, commanded vs. achieved G, bearing, lock/guiding flags, CPA, …) for offline analysis.

---

## Modeled under the hood (things you don't directly see)

These are simulated faithfully even though there's no slider or label for them:

- **Fixed‑timestep integration.** Physics run at a constant **500 Hz** internal step (semi‑implicit Euler), decoupled from rendering and from the run‑speed slider, so results are deterministic and stable regardless of frame rate or playback speed.
- **Coordinated‑turn kinematics.** Lateral acceleration curves the velocity vector without changing speed: turn rate `ω = a_lateral / V`. Speed is handled separately by thrust and drag.
- **Aerodynamic turn authority ∝ V².** A missile can't pull its full structural G at low airspeed — lift scales with dynamic pressure (≈ ½ρV²). The achievable G is capped by `(V/V_ref)²` (with `V_ref ≈ 100 m/s`). This is *why* a missile launched from rest can barely turn for the first instant, and why a coasting missile that has bled its speed loses the ability to follow a hard‑maneuvering target. (This authority limit is distinct from structural saturation — only the latter is flagged as "command saturated.")
- **Velocity‑dependent drag.** Parasitic drag scales with **V²**; induced (lift‑induced) drag scales with **load² / V²** — so hard turns at low speed are doubly expensive. Energy management is a real factor: you can sometimes force a miss by dragging the interceptor into turns until it's too slow to finish.
- **First‑order airframe lag on *both* vehicles.** Neither the missile's autopilot nor the drone responds instantly to a command; both pass commands through a first‑order lag (separate time constants). This is what makes the *achieved* G trail the *commanded* G on the chart.
- **Command saturation.** The commanded lateral G is clamped to the structural limit before the airframe lag, and saturation is recorded.
- **Sub‑step closest‑approach detection.** Hits and miss distance use the *analytic* minimum of the relative trajectory within each step (closed‑form `t*`), not the discrete per‑step range. This prevents "tunneling" (a fast missile skipping past the target between steps) and reports the true perpendicular miss to centimetre precision.
- **Holonomic drone control model.** Your cursor sets a desired velocity via an "arrive" controller (it eases off and stops *on* the cursor); a proportional law turns that into an acceleration command, which is capped at the drone's max‑G and passed through its airframe lag. The **control scale follows the zoom** (full‑speed command at a fixed *on‑screen* distance), so flying feels consistent as the camera zooms in during the endgame.
- **Discrete seeker with a latency buffer.** Measurements are queued and only released to the estimator after the processing‑latency delay — a genuine pipeline, not an instantaneous read.
- **3‑state Kalman filter math.** A full predict/update with a near‑constant‑acceleration state‑transition matrix and a continuous‑white‑jerk process‑noise matrix; measurement noise is taken from the *Angle noise* parameter. Process noise is mapped logarithmically from the *Filter agility* slider.
- **Target‑acceleration estimation for APN.** The target's perpendicular acceleration is reconstructed from the Kalman λ̈ via the LOS‑rate kinematics: `a_T⊥ = R·λ̈ + 2·Ṙ·λ̇ + a_M⊥` (using the missile's own known acceleration), then bounded — so APN runs on an *estimate*, not on cheating knowledge of the target.
- **Honest sensor information by default.** Guidance consumes the *estimated* λ̇, a *noisy* closing velocity, and the *estimated* target accel — not ground truth. The **Ideal sensors** toggle swaps in perfect values so you can isolate how much performance the sensing/estimation chain costs.
- **Gaussian noise via Box–Muller.** Angle and closing‑velocity noise are drawn from a proper normal distribution.
- **Camera auto‑framing.** The view recenters on the midpoint and zooms to keep both craft in frame, tightening into the terminal endgame.

---

## Deliberate simplifications & fidelity limits

This is a teaching tool, not an engineering‑grade 6‑DOF model. Known simplifications:

- **2‑D planar, gravity‑free.** No altitude, no gravity drop, no 3‑D geometry. The drone's "G" is purely horizontal.
- **Strapdown seeker.** The FOV is measured relative to the missile's velocity vector (body‑fixed), not a decoupled gimbal.
- **Point‑mass airframes.** No angle of attack, no fin dynamics, no roll; the autopilot is a single first‑order lag rather than a higher‑order actuator + airframe model.
- **Constant‑thrust boost.** The motor produces constant acceleration; propellant mass depletion (which would make a real motor accelerate harder as it lightens) is not modeled.
- **Range‑independent angle noise.** Real seekers get noisier up close (glint ∝ 1/R); here the angle noise has constant 1σ.
- **Drag/aero coefficients are illustrative.** They use the correct *functional forms* (V², load²/V², dynamic‑pressure authority) calibrated to a reference speed, not a specific airframe's measured coefficients.

---

## Glossary

- **LOS (line of sight)** — the straight line from interceptor to target.
- **λ (lambda)** — the LOS angle; **λ̇ (lambda‑dot)** — its rate of rotation; **λ̈** — its angular acceleration.
- **Vc (closing velocity)** — rate at which the range is decreasing (`−dR/dt`).
- **Vm** — the missile's own speed.
- **N (navigation constant)** — the PN gain.
- **CPA (closest point of approach)** — the minimum perpendicular distance between the two craft over the engagement; the true miss distance.
- **Boresight** — the seeker's center axis; the **bearing offset** is the target's angle off boresight.
- **ZOH (zero‑order hold)** — holding a sampled value constant until the next sample.
- **Load factor** — acceleration expressed in g's (the "G" pulled in a turn).

---

*This README is kept in sync with the simulator's features and parameters. If a number here disagrees with the app, the app is authoritative — please flag it.*
