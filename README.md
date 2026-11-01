# 🐟 Swishy: A Bio-inspired Robotic Fish with Undulatory Locomotion and Autonomous SLAM-based Navigation using ROS 2

![Swishy swimming in its aquarium](assets/swishy_hero.png)

Swishy is a research-oriented simulation platform for studying how body-caudal-fin
(BCF) swimming can be coupled with autonomous robot navigation. A three-joint body
wave drives a fish-shaped robot that maps its aquarium with SLAM, plans with Nav2,
sees through a camera in each eye and swims choreographed missions, all in
ROS 2 Jazzy and Gazebo Harmonic.

**Contents:**
[Quick start](#quick-start) ·
[Manual start](#manual-start) ·
[1. Body](#1-body-a-traveling-body-wave) ·
[2. Senses](#2-senses-fish-eyes-lidar-and-imu) ·
[3. Habitat](#3-habitat-the-aquarium) ·
[4. Mapping](#4-mapping-slam-toolbox) ·
[5. Navigation](#5-navigation-nav2) ·
[6. Missions](#6-missions-showcase-modes) ·
[7. Models and references](#7-models-and-references) ·
[Repository](#repository) ·
[Git workflow](#git-workflow)

| Capability | How |
|---|---|
| Fish-like swimming | Traveling wave over three tail joints, 1.2 to 2.4 Hz; Lighthill tail thrust against drag sets the speed |
| Vision | An invisible camera in each eye, looking out its own side like a real fish |
| Mapping | SLAM Toolbox: 360° LiDAR, scan matching, loop closure, 5 cm occupancy map |
| Localization | SLAM Toolbox localization on the saved map |
| Navigation | Nav2: NavFn global planner, DWB controller, inflated costmaps, collision monitor |
| Missions | Showcase, patrol, dance, explorer and interactive modes, checked against a safety envelope |
| Habitat | Aquascaped tank: rippling water, fog, caustics, bubbles, swaying kelp, rocks, cacti, a pump and nine swimming real fish (scans) |

## Quick start

Build once (on a fresh machine, `./scripts/setup_workspace.sh --install` installs
the ROS dependencies and builds; `--check` only audits):

```bash
cd ~/swishy
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Then start everything with one command. It opens one Terminator window with a pane
per part (Gazebo, SLAM or localization with Nav2 and RViz, the mission, teleop and
both eye-camera viewers); each pane prints the manual command it runs.

```bash
scripts/swishy.sh                          # build a new map (SLAM + Nav2), drive with teleop
scripts/swishy.sh --map --mode patrol      # use the saved map, patrol the four corners
scripts/swishy.sh --map --mode showcase --loop
scripts/swishy.sh --stop                   # stop everything, including a stray Gazebo
```

| Flag | Effect |
|---|---|
| `--slam` (default) / `--map` | Build a new map while navigating / navigate on `maps/` |
| `--glass` | Glass-walled tank, to watch Swishy from outside ([3. Habitat](#3-habitat-the-aquarium)) |
| `--mode MODE` | `showcase`, `patrol`, `dance`, `explorer`, `interactive` or `none` (default) |
| `--loop` | Repeat the mission |
| `--no-teleop`, `--no-eyes` | Skip the teleop pane / the eye-camera viewers |

Teleop: click its pane, then use `i` `j` `k` `l` (`,` backwards, `k` stop).
Always stop with `scripts/swishy.sh --stop` (or Ctrl-C in each pane); closing the
window can leave a Gazebo server running, which makes the next run stutter.
Gazebo runs in its own partition (`GZ_PARTITION=swishy`), so it can share the
machine with another Gazebo simulation; when starting by hand next to one, run
`export GZ_PARTITION=swishy` in every terminal first.

## Manual start

The same stack by hand, one terminal each. Source ROS and the workspace in every
terminal first:

```bash
source /opt/ros/jazzy/setup.bash && source ~/swishy/install/setup.bash
```

```bash
# 1. Gazebo: aquarium, Swishy, bridges, gait controller
ros2 launch fishy_fish_navigation spawn_robot.launch.py
#    ...or the glass-walled tank:
ros2 launch fishy_fish_navigation spawn_robot.launch.py world:=fish_world_glass.sdf

# 2a. Build a new map while navigating (SLAM Toolbox mapping + Nav2 + RViz)
ros2 launch fishy_fish_navigation navigation_with_slam.launch.py
# 2b. ...or navigate on the saved map (SLAM Toolbox localization + Nav2 + RViz)
ros2 launch fishy_fish_navigation navigation_with_map.launch.py
#     if the robot is not at the origin, give its pose:
ros2 run fishy_fish_navigation_py send_initialpose --ros-args -p x:=0.0 -p y:=0.0 -p yaw:=0.0

# 3. A mission (once Nav2 is up)
ros2 launch fishy_fish_navigation_py showcase_navigation.launch.py mode:=patrol

# 4. Keyboard driving
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 5. The eye cameras, one window each
ros2 run rqt_image_view rqt_image_view /left_eye/image
ros2 run rqt_image_view rqt_image_view /right_eye/image
```

Save a map built in 2a (this overwrites the saved map):

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$HOME/swishy/src/fishy_fish_navigation/maps/serialized'}"
ros2 run nav2_map_server map_saver_cli -f ~/swishy/src/fishy_fish_navigation/maps/map
```

After changing a `.urdf`, launch file or script, rebuild with
`colcon build --symlink-install` and source `install/setup.bash` again.
[src/README.md](src/README.md) documents every node, topic and launch file.

## 1. Body: a traveling body wave

![Fish anatomy](assets/fish_anatomy.png)

Fish swim by passing a wave of lateral curvature from the body toward the caudal
fin. Swishy approximates this over three tail joints:

$`q_i(t)=b_i\kappa(t)+a_iA(s)\sin(\phi(t)-i\Delta\phi)`$, with $\dot{\phi}=2\pi f(s)$.

Coefficients $a_i$ grow toward the tail, $\Delta\phi=\pi/3$ makes each joint lag
the one ahead of it, and $\kappa$ bends the body into turns. Like a real fish, the
swing stays large at any swimming speed (at least 60% of the 0.55 rad maximum) and
the tail beats faster to go faster, $f(s)$ from 1.2 to 2.4 Hz
([bio-inspired robotic-fish research](https://doi.org/10.3390/jmse10060773)).

![Tail joint angles while swimming](assets/tail_wave.png)

Measured joint angles (degrees) over 3 s of sim time at 0.3 m/s. Blue: joint 1,
nearest the head (±11°); orange: joint 2 (±18°); aqua: joint 3 at the tail (±25°).
Each lags the one before it, so the wave travels head to tail.

A ROS 2 gait controller (`publisher.py`) turns Nav2's velocity commands into these
joint targets at 50 Hz. **The tail sets the speed:** a commanded speed only decides
how hard the tail beats, and the forward speed $U$ then comes from a 1-D force
balance between the tail's thrust and the water's drag:

$`(m+m_a)\dot{U}=T-k\,U|U|-c\,U`$, with $T=\tfrac{1}{2}m_f\left(\dot{h}^2-U^2h'^2\right)$, $m_f=\tfrac{\pi}{4}\rho s^2$.

$T$ is Lighthill's elongated-body thrust from the fin tip's sideways velocity
$\dot{h}$ and slope $h'$ (fin span $s=0.13$ m). It shrinks as the fish speeds up and
vanishes when the body moves as fast as its own wave. So Swishy accelerates over
about 0.4 s, coasts when it eases off, and surges slightly with each tail beat.
Asked to stop, it brakes hard by flaring its fins (a stop takes about 4 cm) and
then hovers, its pectoral fins cancelling the tail's push. This lets it turn on
the spot at a goal without drifting, which Nav2 relies on. When the tail beats
harder to turn than the speed needs, only the swimming share of the beat pushes
forward. A small speed loop raises the beat when Swishy is slower than commanded.
The drag $k$ is calibrated so the full-power gait balances just above Nav2's
0.5 m/s limit.

The result moves the body through planar joints:
$[\dot{x},\dot{y},\dot{\psi}]^T=[U\cos\psi,U\sin\psi,\omega]^T$. Turning ($\omega$)
is still kinematic, and there is no 3-D fluid or sideways drift. Watch the model
live with `ros2 topic echo /swim/speed` (also `/swim/thrust`, `/swim/drag` and
`/swim/strouhal`). The Strouhal number $St=fA/U$ reads about 1, against 0.2–0.4
for real fish, an honest sign that Swishy's cartoon tail swings wider than a
real one. To compare with direct velocity control, launch the simulation with
`hybrid_swimming:=false`. Every equation, parameter value and source is collected
in [7.1 Locomotion](#71-locomotion-publisherpy).

| Watch or change | What it is |
|---|---|
| `/swim/speed` | Forward speed $U$ from the force balance (m/s) |
| `/swim/thrust`, `/swim/drag` | Tail thrust $T$ and water drag (N) |
| `/swim/strouhal` | $St=fA/U$ (0 below 0.05 m/s) |
| `hybrid_swimming:=false` | Launch argument: `/cmd_vel` sets the speed directly, as before |
| `fin_span`, `body_mass`, `brake_drag_factor`, `hover_drag_factor`, `speed_gain` | Gait-controller parameters ([src/README.md](src/README.md)) |

Plot them live with `ros2 run rqt_plot rqt_plot /swim/speed/data /swim/thrust/data`.

**What coasting taught us.** Physics that is right on its own can still break the
navigator, because Nav2 assumes the robot does what it was told at once. The
first patrols stalled for two different reasons, and a baseline run pinned down
the cause:

1. *Gliding past the goal.* Near a goal, Nav2 sends speed 0 and turns the fish in
   place. The fish glided about 0.3 m past the goal toward the wall and got stuck
   there. Fix: a stop brakes hard (about 4 cm), and the fish then holds position.
2. *Turning pushed it too fast.* Leaving a station, Nav2 asks for a very slow
   forward speed plus a turn. The turn made the tail beat harder, and that extra
   push carried the fish into the plant at about 0.15 m/s instead of 0.026 m/s.
   Fix: only the swimming share of the tail beat pushes forward.
3. *Checking against a baseline.* The same patrol with `hybrid_swimming:=false`
   passed, which showed the swim model, not the map or Nav2, caused the stalls.

## 2. Senses: fish eyes, LiDAR and IMU

![Left eye, both fields of view from above, right eye](assets/eye_cameras.png)

An invisible camera sits in the centre of each eye and looks out its own side,
like a real fish. Left: the left eye's image (blue border). Middle: the map from
above with Swishy facing right and each eye's field of view. Right: the right eye's
image (orange border). Each eye covers 10° to 90° off the heading on its own side,
so together they span 160° with a 20° blind spot straight ahead and no overlap.

| Eye cameras | |
|---|---|
| Mounting | In each eyeball, 4.5 cm behind the body origin, 10 cm up |
| Direction | 50° left / 50° right of straight ahead (40° forward of sideways) |
| Field of view | 80° horizontal each, 320 × 240 px |
| Rate | 5 Hz each (both together render as many frames as one 10 Hz camera) |
| Near clip | 0.1 m, so the camera does not see its own eyeball and head |
| Topics | `/left_eye/image`, `/right_eye/image` (+ `/compressed`), `/<side>_eye/camera_info` |
| Frames | `left_eye_camera_optical`, `right_eye_camera_optical` |

The colour detector (`object_detection_node.py`) and RViz's camera display use the
left eye.

![One LiDAR scan](assets/lidar_scan.png)

One 360° LiDAR scan (blue rays, orange hits) from the middle of the tank, over the
saved map. The GPU LiDAR has 360 beams, 0.05 to 10 m range and runs at 10 Hz; it
sees the glass, the pump, the cylinder and the hard decor, but not the water, kelp or
grass (see [Habitat](#3-habitat-the-aquarium)). An IMU at 50 Hz and the odometry
feed a `robot_localization` EKF.

## 3. Habitat: the aquarium

![Aquarium from above](assets/decor_top.png)

![Aquarium, angled view](assets/decor_side.png)

![Inside the aquarium](assets/aquatic_habitat.png)

Inside the tank (left to right, top to bottom): kelp against the painted backdrop;
the cactus centrepiece with its pebble; a starfish and shells by the front glass;
lily pads seen from below; rising bubbles over the grass; kelp and rocks.

**Water** (`models/aquarium_water/`), all visual-only and drawn on the GPU:
a rippling surface from three short Gerstner ripples (0.45, 0.30 and 0.20 m
wavelengths, not ocean swell), underwater fog, caustics on the sand, swaying light
shafts, rising bubbles (three columns from the sand and a chain of small bubbles
from the pump's outlet) and a tinted water volume. Gazebo's renderer
has no fog, so 40 nested shells are re-centred on the camera by a vertex shader.
The shaders use `swishy::WaterShaderParam` (`src/swishy_water`): gz-sim's
`ShaderParam` with depth writes switched off, so transparent effects blend instead
of hiding what is behind them.

**Two tanks, same habitat.** `fish_world.sdf` is the classic tank with opaque
blue walls. `fish_world_glass.sdf` (`--glass`) is the same world for watching
from outside:
- thin glass panes on three sides (the tank mesh's 14 cm walls looked doubled
  through glass) and the coloured back wall;
- a frame in the back wall's blue, whose level base hides the floor's edge
  (the tank model's floor rises 0.18 m from west to east);
- deeper water (`aquarium_water_deep`: surface at 2.3 m, about 1.35 m above
  the sand instead of 0.6 m), so the tank has aquarium proportions; Swishy
  still swims near the bottom, at the same height as in the classic tank;
- the ground raised to 0.7 m and the glass ending just above the waterline, so
  most of what you see is water;
- the water sized to the inside of the glass.

The panes sit exactly where the old walls' inner faces were, so the robot, decor,
map and routes are identical and the saved map works in both.

**Decor** (`models/`): Fuel kelp swaying in a vertex shader (each plant out of step
with the others), a cactus-and-pebble centrepiece, cacti, rocks and a driftwood log
in the corners, grass and ferns, shells, a starfish, lily pads, a painted
backdrop and a pump on a slate plinth, where the red box used to be, with a bubble
stream from its outlet.

**Fish** (`models/fish_*`): nine real fish of five species swim around the tank:
carp, fusilier, pale bleak, a vermiculated puffer and a pompano. They are
photogrammetry scans of real specimens, reduced to 4,000 to 6,000 triangles and
a 128 px texture each, at real sizes (15 to 32 cm). Only single-piece scans are
used: scans made of many textured patches fell apart when reduced. A vertex shader (`fish_common/SwimFish`) moves each one
along its own ellipse or figure-eight, turns it along the path, bobs it and
bends its body in a wave from head to tail.
Copies of a species swim out of step. The fish are visual only and hidden from
the LiDAR and eye cameras (`visibility_flags` 4), so the map and navigation are
unchanged. `scripts/make_fish_models.py` rebuilds them from the scans, and
`FISH` in `scripts/layout_decor.py` places them in both tanks. The sun and a spot light over the centrepiece cast shadows.

**Sensor-safe by design:**
- Water visuals carry `visibility_flags` 4 and soft decor (kelp, grass, shells,
  starfish, lily pads, backdrop) 8. The LiDAR's mask hides both and the eye cameras
  hide only the water, so `/scan` and the map are unchanged while the cameras still
  see the plants.
- Soft decor has no collision (Swishy can brush through kelp); hard decor (cacti,
  rocks, driftwood) has simple collisions and is mapped.
- Decor colours stay outside the colour detector's HSV ranges.

Change the layout in `scripts/layout_decor.py`, which checks every route and rewrites
the decor block in both worlds. The water meshes come from
`scripts/make_water_grid.py` and `scripts/make_aquarium_effects.py` (which also
generates `aquarium_water_deep` from `aquarium_water`), the shells,
starfish and backdrop from `scripts/make_decor_assets.py`, and the double-sided
kelp from `scripts/make_kelp_sway_meshes.py`.

## 4. Mapping: SLAM Toolbox

![Saved map with the patrol path](assets/map_trajectory.png)

The saved map (`maps/`) with the path Swishy swam on one patrol (blue), the four
patrol stations (orange) and its start pose (aqua). SLAM Toolbox builds a 5 cm
occupancy grid from the LiDAR by scan matching and pose-graph optimization with
loop closure, conceptually
$`\mathbf{x}^{*}=\arg\min_{\mathbf{x}}\sum_k\lVert\mathbf{r}_k(\mathbf{x})\rVert_{\Omega_k}^{2}`$.
Dark cells are walls and obstacles; the rounded shapes are the pump (mapped when
it was still a 0.3 m box), the cylinder,
the centrepiece and the tank's own rocks and coral.

## 5. Navigation: Nav2

![Global costmap and planned path](assets/costmap_plan.png)

Nav2's global costmap during a patrol leg. Obstacles inflate outward (light to dark
blue as cost rises, dark navy where the robot's centre would collide), the orange
line is NavFn's planned path through the low-cost corridors, the aqua circle is the
robot's 0.22 m footprint and the dashed yellow square is the 3 × 3 m rolling local
costmap the DWB controller drives on.

| Nav2 | |
|---|---|
| Global planner | NavFn on the static map + obstacle layer, 1 Hz |
| Controller | DWB, up to 0.5 m/s and 1.5 rad/s, goal tolerance 0.15 m |
| Costmaps | 5 cm cells, robot radius 0.22 m, inflation 0.70 m |
| Safety | Collision monitor on the LiDAR, active during custom paths |

## 6. Missions: showcase modes

![Showcase routes and safety envelope](assets/showcase_routes.png)

The mission routes over the saved map: the figure-eight (blue), the orbit around the
cylinder (orange), the dance circle and S-curve (aqua) and the patrol square
(dashed yellow). Pink discs are each obstacle's safety envelope (its radius plus
Swishy's 0.22 m radius plus 0.12 m clearance); `navigation_patterns.validate()`
rejects any route point inside one, and `test_navigation_patterns.py` checks every
route against `showcase_profiles.yaml`.

| Mode | What Swishy does |
|---|---|
| `showcase` | Figure-eight, orbit around the cylinder, pauses, back home |
| `patrol` | Visits the four corners, then home |
| `dance` | Double circle, S-curve and a compact figure-eight |
| `explorer` | Random safe goals until stopped (`explorer_goal_count` limits it) |
| `interactive` | Goes to goals from RViz's "2D Goal Pose" (`/goal_pose`) |

```bash
ros2 launch fishy_fish_navigation_py showcase_navigation.launch.py mode:=showcase loop:=true
```

The active path is published on `/swishy/showcase/path` and the state on
`/swishy/showcase/status`. Settings live in
`src/fishy_fish_navigation_py/config/showcase_profiles.yaml`.
Known issue: the orbit passes very close to the tank's own rock at (0.15, 0.95) and
can stall there.

The figures in sections 2, 4, 5 and 6 are rendered from a recorded patrol by
`scripts/make_readme_figures.py` (its header shows how to record the bag).

## 7. Models and references

Every model Swishy uses, written out with its source and the values in the code,
so the project can be cited, checked and extended. Numbers in brackets point to
the [references](#references) below.

### 7.1 Locomotion (`publisher.py`)

**Body wave.** Body-caudal-fin swimmers bend in a wave that travels from head to
tail with an amplitude that grows toward the tail, as in carangiform swimming
[1, 2]. Swishy samples that wave at its three tail joints:

```math
q_i(t)=b_i\,\kappa+a_i\,A\,\sin\!\big(\phi(t)-(i-1)\,\Delta\phi\big),\qquad \dot\phi=2\pi f,\qquad i=1,2,3 \tag{1}
```

with amplitude envelope $`a=(0.45,\,0.72,\,1.0)`$, turning envelope
$`b=(0.35,\,0.65,\,1.0)`$ and phase lag $\Delta\phi=\pi/3$.

**Gait schedule.** An activity level $\alpha\in[0,1]$ sets the beat. As in real
fish, the frequency rises with speed while the amplitude stays near constant [3, 4]:

```math
f=f_{\min}+(f_{\max}-f_{\min})\,\alpha,\qquad A=A_{\max}\,\sigma\,\big(r+(1-r)\,\alpha\big),\qquad \sigma=\min(1,\alpha/0.15),\qquad \kappa=\kappa_{\max}\,\frac{\omega}{\omega_{\max}} \tag{2}
```

with $f_{\min}=1.2$ Hz, $f_{\max}=2.4$ Hz, $A_{\max}=0.55$ rad, $r=0.6$,
$\kappa_{\max}=0.28$ rad and $\omega_{\max}=1.5$ rad/s.

**Fin-tip kinematics.** For tail segments of length $`\ell=(0.050,\,0.0495,\,0.102)`$ m,
measured from the meshes, the fin tip's sideways offset $h$ and slope $h'$ in the
body frame are

```math
\theta_k=\sum_{j\le k}q_j,\qquad h=\sum_{k=1}^{3}\ell_k\sin\theta_k,\qquad h'=\tan\theta_3 \tag{3}
```

**Thrust: Lighthill's elongated-body theory** [5, 6]. A slender swimmer pushes
water sideways at its trailing edge, and the reaction gives a forward thrust:

```math
T=\tfrac{1}{2}\,m_f\left(\dot h^{2}-U^{2}h'^{2}\right),\qquad m_f=\tfrac{\pi}{4}\,\rho\,s^{2} \tag{4}
```

$m_f$ is the fluid mass per metre carried by the fin (added mass of a flat plate of
span $s$). With $\rho=1000$ kg/m³ and $s=0.13$ m, $m_f=13.3$ kg/m. The thrust falls
as the fish speeds up and vanishes when $`U=|\dot h/h'|`$, the speed of the body wave
itself. Swishy uses the instantaneous form, clamps it at zero, smooths it over
0.2 s (about half a beat) and keeps only the swimming share of the beat:
$`T_{\text{eff}}=\eta\,T`$, where $\eta=e/\alpha$ is the speed loop's effort over the
total activity, so a hard turning beat does not add forward speed.

**Surge dynamics** [7]. Forward speed $U$ follows a one-degree-of-freedom balance
of thrust against linear and quadratic drag, in the style of Fossen's
marine-craft models:

```math
(m+m_a)\,\dot U=T_{\text{eff}}-\beta\,k\,U|U|-c\,U \tag{5}
```

with $m=0.43$ kg, surge added mass $m_a=0.05$ kg and $c=0.3$ N·s/m. The factor
$\beta$ models fin braking: $\beta$ rises from 1 to 4 as the fish exceeds the
commanded speed by up to 0.1 m/s, and $\beta=30$ with $T_{\text{eff}}=0$ when it
is told to stop (hovering).

**Drag calibration.** $k$ is chosen so the full-power gait balances just above the
navigation speed limit, $U_{\text{top}}=1.2\times0.5$ m/s:

```math
k=\frac{\overline{T}(U_{\text{top}})-c\,U_{\text{top}}}{U_{\text{top}}^{2}}\approx19.2\ \text{N·s}^2/\text{m}^2 \tag{6}
```

where $\overline{T}$ is Eq. (4) averaged over one beat at $\alpha=1$
(7.1 N at 0.6 m/s).

**Implicit integration** [8]. The drag is stiff (this light fish stops within a
few 20 ms steps), so each step solves Eq. (5) backward-Euler for the new speed
$U'$, which is the positive root of a quadratic:

```math
k\beta\,\Delta t\,U'^{2}+(M+c\,\Delta t)\,U'-\big(M U+T_{\text{eff}}\,\Delta t\big)=0,\qquad M=m+m_a \tag{7}
```

(mirrored for negative momentum). Explicit Euler oscillated and diverged at
these values.

**Speed loop.** Because speed now emerges from Eq. (5), a PI loop on the speed
error sets the beat effort, with feed-forward from the commanded share of top
speed:

```math
e=\frac{|U_{\text{cmd}}|}{U_{\text{top}}}+K_p\,\frac{|U_{\text{cmd}}|-|U|}{U_{\max}}+K_i\int\frac{|U_{\text{cmd}}|-|U|}{U_{\max}}\,dt,\qquad \alpha=\max\!\big(e,\;0.55\,|\omega|/\omega_{\max}\big) \tag{8}
```

with $K_p=2.0$, $K_i=1.5$ (integral clamped to ±0.5) and $U_{\max}=0.5$ m/s.

**Planar motion.** The speed drives the body through three planar joints:

```math
\dot x=U\cos\psi,\qquad \dot y=U\sin\psi,\qquad \dot\psi=\omega \tag{9}
```

**Strouhal number** [9, 10, 11]. A check on the gait, published on
`/swim/strouhal`:

```math
St=\frac{f\,A_{pp}}{U} \tag{10}
```

where $A_{pp}$ is the fin tip's peak-to-peak sweep. Efficient swimmers cruise at
$0.2<St<0.4$ [9]. Swishy reads about 1, because its cartoon tail swings wider than
a real fish's for the same speed.

**Assumptions and limits.** One-dimensional surge only: no sway, heave or
fluid-driven yaw, and no 3-D flow or vortex wake. Turning ($\omega$) is still
kinematic. The thrust is the reactive (added-mass) term only; resistive
skin-friction forces along the body are folded into $k$ and $c$. Coefficients are
calibrated to the gait, not measured on a physical fish.

### 7.2 Estimation and navigation

**State estimation** [12]. `robot_localization` fuses the planar-joint odometry
and the IMU with an extended Kalman filter:

```math
\hat x_{k|k-1}=f(\hat x_{k-1}),\quad P_{k|k-1}=F P_{k-1} F^{\top}+Q,\quad K=P_{k|k-1}H^{\top}\big(HP_{k|k-1}H^{\top}+R\big)^{-1},\quad \hat x_k=\hat x_{k|k-1}+K\big(z_k-h(\hat x_{k|k-1})\big) \tag{11}
```

**Mapping** [13, 14]. SLAM Toolbox builds a pose graph from scan matching and loop
closures and solves for the poses $x$ that best agree with all relative
measurements $z_{ij}$:

```math
x^{*}=\arg\min_{x}\sum_{(i,j)}e_{ij}^{\top}\,\Omega_{ij}\,e_{ij},\qquad e_{ij}=z_{ij}\ominus\left(x_i^{-1}\oplus x_j\right) \tag{12}
```

**Costmap inflation** [15, 16]. Nav2's inflation layer lets cost decay
exponentially with distance $d$ from the nearest obstacle, beyond the robot's
inscribed radius $r_{\text{in}}=0.22$ m:

```math
C(d)=252\,e^{-\gamma\,(d-r_{\text{in}})},\qquad \gamma=3.0,\qquad d\le0.70\ \text{m} \tag{13}
```

**Local control** [17]. The DWB controller samples velocities $(v,\omega)$ reachable
within its acceleration limits (the dynamic window), simulates each for 1.7 s and
picks the one with the lowest weighted critic cost:

```math
(v,\omega)^{*}=\arg\min_{(v,\omega)\in V_d}\sum_{c}w_c\,C_c(v,\omega) \tag{14}
```

with critics for path alignment and distance, goal alignment and distance,
rotation to goal, oscillation and obstacles.

### 7.3 Rendering the habitat (GLSL shaders)

**Water surface: Gerstner waves** [18, 19]. Three waves move each surface vertex
$P_0$ along circular orbits (implementation adapted from VRX):

```math
P=P_0+\sum_{i=1}^{3}\Big(-q_i\,a_i\,\mathbf d_i\sin\theta_i,\;\;a_i\cos\theta_i\Big),\qquad \theta_i=k_i\,(\mathbf d_i\cdot P_0)-\Omega_i t \tag{15}
```

with steepness $q_i=0.6$, amplitude $`a_i=(7,\,5,\,3)`$ mm (ramped in as
$1-e^{-t/\tau}$), wavenumber $`k_i=(14,\,21,\,31.4)`$ rad/m (wavelengths 0.45, 0.30 and
0.20 m), direction $\mathbf d_i$ and angular frequency $`\Omega_i=(4.2,\,5.3,\,6.6)`$ rad/s,
slower than deep-water dispersion $\Omega=\sqrt{gk}$ for a calm tank.

**Reflection: Fresnel term** [20]. Reflection and water colour are blended with a
Schlick-style power of the facing angle:

```math
F=\mathrm{clamp}\big((1-\mathbf n\cdot\mathbf v)^{p},\,0.05,\,1\big) \tag{16}
```

**Underwater fog: Beer–Lambert attenuation** [21]. Each of the 40 camera-centred
shells, spaced $\delta=0.1$ m apart, adds $1-e^{-\sigma\delta}$ of fog colour, so an
object $d$ away sits behind about $d/\delta$ shells and is tinted like exponential
fog:

```math
\alpha_{\text{fog}}(d)\approx1-e^{-\sigma d} \tag{17}
```

**Caustics: cellular noise** [22]. Bright lines are drawn where two Worley cells
meet, i.e. where the gap between the nearest and second-nearest feature point is
small:

```math
I=1-\mathrm{smoothstep}\big(0,\,w,\;F_2(\mathbf p)-F_1(\mathbf p)\big) \tag{18}
```

Two layers at different scales and speeds and a gentle domain warp keep the
pattern from looking tiled.

**Kelp sway.** A cantilever-like profile keeps the base fixed and grows the bend
with height squared, so the tips move most:

```math
\Delta x=A_s\,(z/H)^{2}\sin(\Omega_s t+\varphi+2z) \tag{19}
```

with a per-plant phase $\varphi$ and a slightly different frequency for each plant.

### References

1. M. Sfakiotakis, D. M. Lane and J. B. C. Davies, "Review of fish swimming modes
   for aquatic locomotion," *IEEE J. Oceanic Eng.*, 24(2):237–252, 1999.
   [doi:10.1109/48.757275](https://doi.org/10.1109/48.757275)
2. B. Sun *et al.*, "Recent progress in modeling and control of bio-inspired fish
   robots," *J. Mar. Sci. Eng.*, 10(6):773, 2022.
   [doi:10.3390/jmse10060773](https://doi.org/10.3390/jmse10060773)
3. R. Bainbridge, "The speed of swimming of fish as related to size and to the
   frequency and amplitude of the tail beat," *J. Exp. Biol.*, 35(1):109–133, 1958.
   [doi:10.1242/jeb.35.1.109](https://doi.org/10.1242/jeb.35.1.109)
4. M. Saadat *et al.*, "On the rules for aquatic locomotion," *Phys. Rev. Fluids*,
   2:083102, 2017. [doi:10.1103/PhysRevFluids.2.083102](https://doi.org/10.1103/PhysRevFluids.2.083102)
5. M. J. Lighthill, "Note on the swimming of slender fish," *J. Fluid Mech.*,
   9(2):305–317, 1960. [doi:10.1017/S0022112060001110](https://doi.org/10.1017/S0022112060001110)
6. M. J. Lighthill, "Large-amplitude elongated-body theory of fish locomotion,"
   *Proc. R. Soc. Lond. B*, 179:125–138, 1971.
   [doi:10.1098/rspb.1971.0085](https://doi.org/10.1098/rspb.1971.0085)
7. T. I. Fossen, *Handbook of Marine Craft Hydrodynamics and Motion Control*,
   Wiley, 2011. [doi:10.1002/9781119994138](https://doi.org/10.1002/9781119994138)
8. E. Hairer and G. Wanner, *Solving Ordinary Differential Equations II: Stiff and
   Differential-Algebraic Problems*, 2nd ed., Springer, 1996.
   [doi:10.1007/978-3-642-05221-7](https://doi.org/10.1007/978-3-642-05221-7)
9. G. K. Taylor, R. L. Nudds and A. L. R. Thomas, "Flying and swimming animals
   cruise at a Strouhal number tuned for high power efficiency," *Nature*,
   425:707–711, 2003. [doi:10.1038/nature02000](https://doi.org/10.1038/nature02000)
10. G. S. Triantafyllou, M. S. Triantafyllou and M. A. Grosenbaugh, "Optimal thrust
    development in oscillating foils with application to fish propulsion,"
    *J. Fluids Struct.*, 7(2):205–224, 1993.
    [doi:10.1006/jfls.1993.1012](https://doi.org/10.1006/jfls.1993.1012)
11. Saadat *et al.* [4], for $St$ as a consequence of body shape and drag.
12. T. Moore and D. Stouch, "A generalized extended Kalman filter implementation
    for the Robot Operating System," in *Intelligent Autonomous Systems 13*,
    Springer AISC 302:335–348, 2015.
    [doi:10.1007/978-3-319-08338-4_25](https://doi.org/10.1007/978-3-319-08338-4_25)
13. S. Macenski and I. Jambrecic, "SLAM Toolbox: SLAM for the dynamic world,"
    *J. Open Source Softw.*, 6(61):2783, 2021.
    [doi:10.21105/joss.02783](https://doi.org/10.21105/joss.02783)
14. G. Grisetti, R. Kümmerle, C. Stachniss and W. Burgard, "A tutorial on
    graph-based SLAM," *IEEE Intell. Transp. Syst. Mag.*, 2(4):31–43, 2010.
    [doi:10.1109/MITS.2010.939925](https://doi.org/10.1109/MITS.2010.939925)
15. S. Macenski, F. Martín, R. White and J. Ginés Clavero, "The Marathon 2: A
    navigation system," *IEEE/RSJ IROS*, 2718–2725, 2020.
    [doi:10.1109/IROS45743.2020.9341207](https://doi.org/10.1109/IROS45743.2020.9341207)
16. D. V. Lu, D. Hershberger and W. D. Smart, "Layered costmaps for
    context-sensitive navigation," *IEEE/RSJ IROS*, 709–715, 2014.
    [doi:10.1109/IROS.2014.6942636](https://doi.org/10.1109/IROS.2014.6942636)
17. D. Fox, W. Burgard and S. Thrun, "The dynamic window approach to collision
    avoidance," *IEEE Robot. Autom. Mag.*, 4(1):23–33, 1997.
    [doi:10.1109/100.580977](https://doi.org/10.1109/100.580977)
18. A. Fournier and W. T. Reeves, "A simple model of ocean waves," *ACM SIGGRAPH
    Comput. Graph.*, 20(4):75–84, 1986.
    [doi:10.1145/15886.15894](https://doi.org/10.1145/15886.15894)
19. J. Tessendorf, "Simulating ocean water," *SIGGRAPH Course Notes*, 2001.
20. C. Schlick, "An inexpensive BRDF model for physically-based rendering,"
    *Comput. Graph. Forum*, 13(3):233–246, 1994.
    [doi:10.1111/1467-8659.1330233](https://doi.org/10.1111/1467-8659.1330233)
21. C. D. Mobley, *Light and Water: Radiative Transfer in Natural Waters*,
    Academic Press, 1994.
22. S. Worley, "A cellular texture basis function," *Proc. SIGGRAPH '96*, 291–294,
    1996. [doi:10.1145/237170.237267](https://doi.org/10.1145/237170.237267)

## Repository

```
swishy/
├── assets/                          README images
├── scripts/
│   ├── swishy.sh                    one-command launcher
│   ├── setup_workspace.sh           dependency check / install + build
│   ├── make_readme_figures.py       renders the technical figures from a bag
│   └── layout_decor.py, make_*.py   aquarium layout and generated assets
└── src/
    ├── fishy_fish_navigation/       robot, world, launch files, configs, maps
    │   ├── urdf/swishy.urdf          body, joints, eye cameras, LiDAR, IMU
    │   ├── urdf/swishy.gazebo        Gazebo plugins and sensors
    │   ├── src/publisher.py          gait controller (body wave + thrust/drag swim model)
    │   ├── worlds/fish_world.sdf     the aquarium
    │   └── maps/                     saved map (serialized + pgm)
    ├── fishy_fish_navigation_py/    showcase missions, map/pose utilities, tests
    ├── swishy_water/                water shader system plugin
    └── README.md                    detailed node and launch documentation
```

## Git workflow

`feature/*` → PR into `dev` → test → PR `dev` into `main`.

- `main`: stable, only gets `dev` through a PR.
- `dev`: development; every feature merges here first.
- `feature/<name>`: one change, cut from `dev`, deleted after merge.

```bash
# start a feature
git switch dev && git pull
git switch -c feature/<name>

# work, then push
git add <files> && git commit -m "What changed"
git push -u origin feature/<name>
# GitHub: open PR, base = dev, merge it (branch auto-deletes)

# afterwards, clean up locally
git switch dev && git pull && git branch -d feature/<name> && git fetch --prune

# release: when dev works in the sim
# GitHub: open PR, base = main, compare = dev, merge it
git switch main && git pull && git switch dev
```

## Related work

1. [VRX](https://github.com/osrf/vrx/tree/jazzy)
2. [POSIM](https://ioes-lab.github.io/POSIM/)
3. [MBARI vehicles sim (ROS 2)](https://github.com/AlePuglisi/MBARI-vehicles-sim-ros2)
4. [UUV Simulator, ROSCon 2018](https://roscon.ros.org/2018/presentations/ROSCon2018_uuvsimulator.pdf)
5. [Bio-inspired robotic fish, JMSE 10(6):773](https://www.mdpi.com/2077-1312/10/6/773)
6. [Orca4](https://github.com/clydemcqueen/orca4)
7. [Pool simulator for UUV Simulator](https://github.com/cagdasaktay/pool-simulator-uuv_sim)
8. [ASV wave sim](https://github.com/srmainwaring/asv_wave_sim)

## License

Apache License 2.0

## Acknowledgments

Built with ROS 2 Jazzy, Gazebo Harmonic, SLAM Toolbox, Nav2 and robot_localization.

- [VRX](https://github.com/osrf/vrx) (Apache 2.0): Gerstner-wave shaders and
  textures used by the aquarium water, originally from UUV Simulator / Rhys Mainwaring
- [gz-sim](https://github.com/gazebosim/gz-sim) (Apache 2.0): `ShaderParam` system,
  adapted as `swishy::WaterShaderParam` in `src/swishy_water`
- Kelp and Ocean Rock models by Cole Biesemeyer, [Gazebo Fuel](https://fuel.gazebosim.org/1.0/Cole),
  CC-BY 4.0 (rescaled, textures downscaled to 1024², rock texture desaturated)
- Fish scans (carp, fusilier, pale bleak, vermiculated puffer, pompano) by
  [ffish.asia / floraZia.com](https://sketchfab.com/ffishAsia-and-floraZia), CC0 1.0,
  via [lisa-1002/fishing3d](https://github.com/lisa-1002/fishing3d) (decimated,
  textures downscaled to 128²)
- "Pump" by Ian Chen, Open Robotics, [Gazebo Fuel](https://app.gazebosim.org/OpenRobotics/fuel/models/Pump),
  CC-BY 4.0 (scaled to 0.30 on a plinth, textures downscaled to 512², albedo
  saturation capped below the colour detector's blue range)
- Nature models (cacti, mossy rock, driftwood, grass, fern, lily pad) by
  [Quaternius](https://quaternius.com), Ultimate Nature Pack, CC0
