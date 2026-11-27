# Domino robot simulation (Robosoutěž 2026)

Godot 4.7 (.NET / C#) simulation of a two-wheel LEGO robot for the
[Domino task](https://robosoutez.fel.cvut.cz/zadani-soutezni-ulohy-domino-0).

The robot works as follows:
1. It drives over a crosswise piece with one of its two front arms lowered. The arm turns the piece by 90°:
   - **Left arm:** clockwise, so the left end ends up in front.
   - **Right arm:** counter-clockwise, so the right end ends up in front.
2. It stores **two pieces lengthwise under its belly**, joined with matching colours.
3. It drives to a slot in the yellow area and **reverses**, which leaves the pair on the floor.

## Running

The project is built with `dotnet build`, or by opening it in the Godot_mono editor and pressing Play.
Without arguments a **start menu** offers two modes:

1. **Jízda podle programu** – runs code and measures the time (90 s limit). Choose:
   - **our Python code** with any mission of `../mission.json` (see *Running our Python mission* below),
   - a recorded program from `programs/` or any JSON file (“Otevřít soubor…”) – the time is shown per step and in total,
   - the original built-in autonomous strategy (`MissionController`, its own robot `data/robot.json`).
2. **Ruční jízda se záznamem** – drive with the keyboard; every movement is recorded as one step
   (`drive` / `turn` / `curve` in the mission.json format, + = left). Pauses are not recorded, consecutive
   movements of the same kind are merged. Turns are measured from the reference heading, like
   `drive.absolute_heading` on the real robot, so the replay ends where the manual run ended (≈1 cm).
   Saved to `programs/zaznam_<time>.json`; the step array is also printed and copied to the clipboard.

Everything except the built-in strategy uses our robot (`data/robot_ours.json`) and our start (magazine against
the top wall above the left row, facing down; `--start` overrides it). Speeds come from `../mission.json` → `robot.drive` (`speed`, `turn_speed`, `curve_inner/outer_speed`, `deg_per_mm`).

| Command (from the project root) | What it does |
|---|---|
| `"/Applications/Godot_mono.app/Contents/MacOS/Godot" --path .` | Windowed run with the start menu. |
| `… --path . -- --ours test_bottom` | Mode 1 with our Python code directly (`--ours` alone = `main`). |
| `… --path . -- --theirs` | Mode 1 with the built-in autonomous strategy. |
| `… --path . -- --program programs/ukazka.json` | Mode 1 with this program directly. |
| `… --headless --path . --fixed-fps 120 -- --program FILE --quit` | Mode 1 headless, prints the step times and quits. |
| `… -- --program ../mission.json --mission-name main` | Only the driving steps of a mission, without Python (`pick_row`, `arm`, … skipped). |
| `… --path . -- --drive` | Mode 2 directly. |
| `… --headless --path . -- --selftest` | Checks the layout, formation geometry, chain plan and A* reachability. Exit code 1 on failure. |
| `… --headless --path . --fixed-fps 120 -- --ours main --mission` | Our Python code faster than real time, prints the log and quits (`--theirs --mission` = built-in strategy). |
| `… -- --round2 <seed>` | Rotates random pieces by 180°, as allowed in round 2. |
| `… -- --robot res://data/robot_small.json` | Uses a different robot geometry. The default is `data/robot.json` (280×200 mm). |
| `… -- --speed 4` | Starts at 4× speed. The physics step stays at 1/120 s. |
| `… -- --screenshot out.png --shot-time 30` | Saves the view at the given sim time and quits. |

**Program format** – a plain array of steps, or `{"start": {"x": mm, "z": mm, "heading": deg}, "steps": [...], "macros": {...}}`.
Supported steps: `drive` (`mm`, `speed`, `hold`, `tail_s`, `tail_speed`), `drive_time` (`s`, `speed`), `turn` (`deg`, `speed`),
`curve` (`deg`, `inner`, `outer`, `s_per_90`), `wait` (`s`), `arm` (`rot` CW/CCW/NONE, `slot` LEFT/RIGHT/toggle, `extra`, `nudge`;
takes as long as motor C on the real robot, from `robot.arm`), `macro` (`name`, macros of mission.json), `unload` (= macro `unload`), `log`.
Speeds are wheel deg/s as on the real robot. Other steps (`pick_row`, …) are skipped.

**Keys, mode 1:** `1`–`4` speed ×1/×2/×4/×8, `R` restart, `Esc` menu, `T` top/orbit view, `F` follow robot,
right mouse drag orbit, wheel zoom.

**Keys, mode 2:**
- `↑↓←→` / `WASD`: drive, turn on the spot; forward + left/right = arc (`curve`)
- `Shift`: slow (`drive`/`turn` with a lower `speed`; the arc pivots around the inner wheel, `inner 0 / outer 300`)
- `Q` / `E`: turn exactly 90° left / right
- `J` / `K` / `L`: hands CW / NONE / CCW, `H`: shift the magazine (`arm slot toggle`); `Space`: macro `unload`
- `Backspace`: undo the last step (restarts and replays the rest quickly)
- `Ctrl+S`: save, `Enter`: save and play it in mode 1, `R`: start over, `Esc`: menu

## Running our Python mission

In the menu: mode 1 → “Náš Python kód – mise …”, or `-- --ours MISSION` on the command line.

Drives the robot with the team's real Python code (`../main.py`, `actions.py`, `mechanism.py`,
`mission.json`) instead of `MissionController`. Python is the brain, Godot is the hardware
(wheels, encoders, gyro, colour sensor, motor C state). They talk over TCP 127.0.0.1:5577 in
lock-step: sim time only runs while Python is in `hw.sleep()`. See `../hw_sim.py` and `src/Ours/OurRunner.cs`.

| Command (from the repo root) | What it does |
|---|---|
| `"/Applications/Godot_mono.app/Contents/MacOS/Godot" --path simulation -- --ours main` | Windowed run of mission `main`; Godot starts `python3 main.py --sim main` itself. |
| `… --headless --path simulation --fixed-fps 120 -- --ours main --mission` | Fast run, prints the log and quits. |
| `… -- --ours test_bottom --start 180 900 -90` | Another mission with a start pose (axle x, z in mm, heading in degrees: 90 = facing down). |
| `… -- --ours main --no-python` + `python3 main.py --sim main` | Start Python yourself (its output in your terminal). |

Python errors are written to `simulation/ours_python.log`. The robot is `data/robot_ours.json`
(365×220 mm, axle 175 mm behind the front, `"mechanism": "ours"` → `src/Ours/OurMechanism.cs`, used by all modes):
- **hands** at the very front (pick line 175 mm ahead of the axle), left and right, 90° out of phase: `CW` / `CCW` lowers one,
  `NONE` none. A crosswise piece is caught and turned by 90° while the robot drives on
  (slot RIGHT + CCW or slot LEFT + CW = the right end goes in front);
- **neck** behind the hands for 2 pieces in a row: the pieces stand on the floor, so they move backwards relative to the
  robot when it drives forward, until they hit the piece behind or the rear wall of the active slot;
- **magazine** behind the neck: 2 slots (LEFT/RIGHT) side by side, 1 piece each; the active one lines up with the neck,
  a shift moves the magazine sideways. Max 4 pieces; a row goes 1 → slot R, 2 → slot L (after the shift), 3 + 4 neck;
- **reversing**: the neck + active slot stay on the floor and leave through the front; the other slot keeps its piece
  until a shift. Driving forward onto a piece lying lengthwise in the lane takes it back into the neck.
The neck/magazine positions are estimated (neck right behind the hands, magazine right behind the neck).
The colour sensor sits 35 mm in front of the **right** hand (24 mm right of the centre line) and sees the right half of a
crosswise piece – that half goes in front (`pick.seen_to_front`). It sees pieces only (floor = WHITE, reflected 17), no black lines.

## Structure

| Path | Purpose |
|---|---|
| `data/layout_varianta_final.json` | Left half of VARIANTA_FINAL in mm. Origin is the inner top-left corner, x to the right, z down. Contains piece colours, lines and the yellow rect. |
| `src/Field/` | JSON layout loader and builder for the floor, walls and printed lines. |
| `src/Domino/Domino.cs` | Piece: 96×32×21 mm rigid body. `C0` is the colour at the local −X end. |
| `src/Robot/Robot.cs` | Chassis plus a wheel traction model: motor slew, friction-limited slip forces, frictionless casters. |
| `src/Robot/Intake.cs` | Arms, scripted capture and rotation, two storage slots, `Release()`. |
| `src/Robot/ColorSensor.cs` | Two downward sensors 160 mm ahead, ±24 mm from the centre line. |
| `src/Strategy/ChainPlanner.cs` | Picks the pairs, the capture order and the required front colours so that all pairs form one chain. |
| `src/Strategy/FormationPlanner.cs` | Pair slots in the yellow area. The default is a U shape: 4 + 1 + 3 pairs, with 2 right-angle joints. |
| `src/Strategy/Navigator.cs`, `Motion.cs` | Grid A*, pure pursuit, line tracking, turn on the spot, precise realignment. |
| `src/Strategy/MissionController.cs` | Mission state machine, written as async code that advances once per physics tick. |
| `src/Program/` | Modes 1 and 2: program file (load/save), `StepExecutor` (runs one step), `ProgramRunner` (mode 1), `ManualDriver` (mode 2), `Scorer`. |
| `src/UI/ModeMenu.cs` | Start menu. |
| `src/Strategy/JointEvaluator.cs` | Checks each joint against the rules: ≤10°, ≤5 mm gap, ≤5 mm offset, matching colours. |

## Conventions that matter for the real robot

- **Capture order:** the first piece goes to the **rear** slot, against the rear stop. The second goes to the **front** slot.
- **Placement:** a pair is always placed by approaching from the **open side** of the chain. The front of the stored line touches the current chain end, so the second captured piece is the one that joins the chain.
- **Arm choice:** at the approach point the sensors read the left and right halves. The robot lowers the left arm if the left colour is the one required in front, otherwise the right arm. Because of this, pieces rotated by 180° in round 2 need no replanning.
- **Reachable pieces:** pieces stand in lines of 4, spaced 200 mm apart, 132 mm from the wall. Only the **outermost piece of each line** can be reached, by driving along the line from its end. ChainPlanner accounts for this.

## Findings and assumptions

- **Robot geometry** is set in `data/robot.json`. The default is 280×200 mm with the wheels at the outer width, a 50 mm corner radius (estimated) and the axle mid-length. Its turning circle is 153 mm. The wall rows are only 132 mm from the wall (122 mm on the right column), so when the robot spins next to a wall its rounded body rubs the wall and is pushed clear. This works in the sim, but it is the main source of trouble:
  - It shoves the robot off its line, so colour readings are cross-checked against the piece's known colours and the robot realigns if they don't match.
  - On the right column it can knock a piece away. After two failed reads the robot gives up on that piece and replans without it.
- **Start:** the robot starts in the top-left corner, on the top-row line, facing along the row (`robotStart` in the layout JSON).
- **Compact concept** (`data/robot_small.json`): a 210 × 100 mm body with the wheels at ±93 mm and a 121 mm turning circle. It never touches the walls and is about 50 s faster over the full run.
- **Yellow rectangle:** its size (x 400–840, z 420–1260, 20 mm line) is **estimated from the figure**. Measure the printed banner and adjust the JSON.
- **Pose:** navigation uses the true pose. Odometry and gyro error are not modelled, so release errors are about 0.1 mm here. Real errors will be much larger.
- **Reversing:** reversing while carrying is not modelled as dropping the pieces. The mission avoids reversing while it carries pieces, except the final release.
- **Current speed:** for the 280×200 robot, all 16 pieces take ≈180–190 s and 8 are placed by the 90 s mark. The compact robot takes ≈130–145 s and places 8–10 by the 90 s mark. The plan minimises estimated travel, but no time-aware scoring is done yet.
