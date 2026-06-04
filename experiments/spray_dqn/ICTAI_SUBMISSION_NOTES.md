# ICTAI 2026 Submission Notes

## Current Paper Position

Suggested framing:

**Hierarchical deep reinforcement learning for safe variable-rate UAV spraying in an orchard autonomy simulator.**

The paper should not claim that plain DQN solves every spraying scenario. The stronger and more defensible claim is:

- DQN-family methods are effective baselines for orchard spraying path planning.
- Auto-spray and safety-controller modules make the enhanced spraying task substantially more learnable and safe.
- DRQN is useful when the task becomes partially observable, explicitly controls spray switching, or expands to sparse whole-farm coverage.
- Whole-farm coverage requires a global prior or curriculum guidance; pure short-horizon end-to-end RL is not enough.

## Must Improve Before Submission

1. **Gazebo replay evidence**
   - Add one screenshot of the apple-orchard Gazebo world.
   - Add QGroundControl mission/vehicle loaded screenshot.
   - Add takeoff, waypoint-following, and landing/completion logs.
   - Add ROS 2 position topic trace or trajectory plot.
   - Add no-collision screenshot or trajectory audit.
   - Add a "grid path vs Gazebo replay trajectory" comparison figure.
   - Keep the claim precise: grid planner result + generated AAS mission + replay evidence.

2. **Main learning curves**
   - Current ablation curves are full 5 seeds.
   - Main enhanced table is 5 seeds, but main learning curves only have newly logged seeds 23 and 31.
   - Rerun main enhanced experiment with `--force --learning-curves` for all 5 seeds if the curve is used as a main figure.
   - The ICTAI draft now avoids claiming a main five-seed learning curve until this is done.

3. **Tighten the contribution claims**
   - Emphasize hierarchical safe RL and ablation-backed design.
   - Avoid claiming universal superiority of DQN or DRQN.
   - State that 90% demand satisfaction is a stage threshold, not the final high-precision spraying target.

4. **Clean references**
   - Fill missing DOI, venue, volume, issue, and pages in `ictai2026_refs.bib`.
   - Replace `misc` entries with proper `article` or `inproceedings` entries where possible.

5. **Traditional baseline coverage**
   - Existing draft now includes greedy nearest-target and row-wise/lawnmower whole-farm results.
   - Add A* + target ordering if time allows, because it is the clearest graph-search baseline for reviewers.
   - Add a same-task heuristic auto-spray baseline under the enhanced 21-target demand setting if a stronger non-learning comparison is needed.

## Strongly Recommended

1. **Add one method diagram**
   - AAS/Gazebo world parsing -> grid MDP -> RL policy -> auto-spray/safety controller -> mission YAML -> Gazebo replay.

2. **Add one algorithm box**
   - Hierarchical action execution:
     1. observe state
     2. RL selects motion
     3. safety controller validates/overrides motion
     4. auto-spray checks local demand
     5. update coverage and demand

3. **Add statistical confidence**
   - Current mean +/- sample standard deviation is acceptable.
   - A small confidence interval or binomial success-rate note would strengthen the experimental section.

4. **Clarify evaluation threshold**
   - Explain why enhanced ablations use 90% demand satisfaction.
   - State that 97% demand satisfaction remains future/extended validation unless more runs are added.

## Optional If Time Allows

1. **Run longer enhanced experiments**
   - Try 30k or 50k timesteps under the final auto-spray+safety configuration.
   - This may improve DRQN/DQN stability and make the paper stronger.

2. **Compare against a simple heuristic**
   - Add nearest-target or lawnmower baseline under the same variable-demand metrics.
   - This helps answer whether RL is better than a deterministic rule baseline.

3. **Add one replay video or supplementary material**
   - ICTAI allows supplementary files.
   - Keep the main PDF self-contained, but put replay video/logs in supplementary material.
