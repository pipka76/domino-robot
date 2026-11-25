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

| Command (from the project root) | What it does |
|---|---|
| `"/Applications/Godot_mono 4.app/Contents/MacOS/Godot" --path .` | Windowed run. The mission starts automatically. |
| `… --headless --path . -- --selftest` | Checks the layout, formation geometry, chain plan and A* reachability. Exit code 1 on failure. |
| `… --headless --path . --fixed-fps 120 -- --mission` | Runs a full match faster than real time, prints a log (joints, release errors, 90 s mark) and quits. |
| `… -- --round2 <seed>` | Rotates random pieces by 180°, as allowed in round 2. |
| `… -- --robot res://data/robot_small.json` | Uses a different robot geometry. The default is `data/robot.json` (280×200 mm). |
| `… -- --speed 4` | Starts at 4× speed. The physics step stays at 1/120 s. |
| `… -- --screenshot out.png --shot-time 30` | Saves the view at the given sim time and quits. |

**Keys:**
- `1` / `2` / `3` / `4`: speed ×1 / ×2 / ×4 / ×8
- `T`: switch between top and orbit view
- `F`: follow the robot
- `R`: restart
- Right mouse drag: orbit
- Mouse wheel: zoom

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
