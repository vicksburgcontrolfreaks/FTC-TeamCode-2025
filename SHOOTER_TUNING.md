# Shooter System Tuning Guide

This guide will help you optimize your shooter system for maximum consistency and speed.

## Table of Contents
1. [Quick Start](#quick-start)
2. [PIDF Tuning](#pidf-tuning)
3. [Timing Optimization](#timing-optimization)
4. [Velocity Settings](#velocity-settings)
5. [Troubleshooting](#troubleshooting)

---

## Quick Start

### Initial Test Run
1. Deploy the code to your robot
2. Run a TeleOp or Auto that uses `ThreeShots`
3. Watch telemetry for:
   - Pre/Post velocities for each shot
   - Current state machine state
   - Shooter velocity vs target

### What to Look For
- **Pre-velocities** should all be within ±30 tps of target
- **Post-velocities** show how much velocity drops after each shot
- **Recovery time** between shots should be minimal

---

## PIDF Tuning

### Location: `RobotHardware.java:94-102`

### Step 1: Set Maximum Velocity
```java
double maxVelocity = 2800.0;  // Max ticks/sec at 12V
```

**How to find this value:**
1. Fully charge battery (12.5V+)
2. Run shooter at full power: `shooter.setPower(1.0)`
3. Read velocity from telemetry: `shooter.getVelocity()`
4. Use that value as `maxVelocity`

### Step 2: Tune P Coefficient
```java
shooter.setVelocityPIDFCoefficients(1.5, 0.0, 0.0, F);
                                    // ^ adjust this
```

**Tuning process:**
- Start at P = 1.5 (default)
- If velocity recovery is **slow** (>150ms): increase P to 2.0, then 2.5
- If velocity **oscillates** or overshoots: decrease P to 1.0, then 0.75
- Goal: Quick recovery without oscillation

**Test method:**
1. Fire 3 shots
2. Check telemetry for post-velocities
3. Observe time to recover between shots (RECOVERING state)
4. Adjust and repeat

### Step 3: I and D Coefficients
For flywheels, keep these at **0.0**. Only adjust if you have persistent velocity drift issues:
- **I (Integral)**: Corrects steady-state error, but can cause overshoot
- **D (Derivative)**: Dampens oscillations, but can slow response

---

## Timing Optimization

### Location: `ThreeShots.java:27-30`

### Flipper Timing
```java
private static final int FLIP_TIME_MS = 300;
```

**How to optimize:**
1. Start at 300ms (default)
2. Use a slow-motion video to measure actual flip time
3. Add 50ms safety margin to measured time
4. Test and reduce by 25ms increments until shots start missing
5. Add back 50ms for reliability

**Example:**
- Measured flip time: 200ms
- Safety margin: +50ms = 250ms
- Test at 250ms, works perfectly
- Try 225ms, still works
- Try 200ms, occasionally misses
- **Final value: 225ms**

### Velocity Recovery Timeout
```java
private static final int RECOVERY_WAIT_MS = 100;
```

**How to optimize:**
1. Watch telemetry during 3-shot sequence
2. Note how long RECOVERING state lasts
3. Set `RECOVERY_WAIT_MS` to typical recovery time + 25ms
4. If recovery consistently times out, increase P coefficient instead

---

## Velocity Settings

### Location: `ThreeShots.java:24-30`

### Target Velocity
```java
private static final int SET_VELOCITY = 1500;  // ticks/sec
```

**Adjustment:**
- Higher velocity = longer range, faster shots
- Lower velocity = better accuracy, less velocity drop
- Test at different distances to find optimal value

### Velocity Tolerance
```java
private static final int VELOCITY_TOLERANCE = 30;
```

**Adjustment:**
- Tighter (20-25): More consistent, but may wait longer
- Looser (40-50): Faster cycling, but less consistent
- Balance based on your accuracy requirements

### Idle Velocity
```java
private static final int IDLE_VELOCITY = 500;
```

**Adjustment:**
- Higher (700-1000): Faster restart for second burst
- Lower (300-500): Less battery drain, quieter
- Set to 0 to fully stop between bursts

**Battery impact:**
- 500 tps idle ≈ 1-2A continuous draw
- Only matters if waiting >5 seconds between bursts

---

## Indexing Optimization

### Location: `ThreeShots.java:25-26`

### Index Distance
```java
private static final int INDEX_TICKS = 1200;
```

**How to find minimum value:**
1. Start at 1200 (default)
2. Run 3-shot sequence and verify all samples load
3. Reduce by 100 ticks
4. Repeat until samples occasionally fail to load
5. Add back 200 ticks for safety

**Example:**
- 1200 ticks: 100% reliable
- 1100 ticks: 100% reliable
- 1000 ticks: 95% reliable (occasional misfeeds)
- **Final value: 1200 ticks**

### Index Power
```java
private static final double INDEX_POWER = 1.0;
```

Usually leave at 1.0 for fastest indexing. Reduce only if:
- Samples are damaged by aggressive indexing
- Collector is overheating

---

## Troubleshooting

### Problem: Inconsistent Shot Velocity

**Symptoms:**
- Pre-velocities vary by >50 tps
- Post-velocities drop significantly

**Solutions:**
1. Check battery voltage (should be >11.5V)
2. Increase P coefficient
3. Check for mechanical friction in shooter
4. Verify motor is not overheating

### Problem: Slow Cycle Time

**Symptoms:**
- Takes >2 seconds for 3 shots
- RECOVERING state lasts >150ms

**Solutions:**
1. Reduce `FLIP_TIME_MS` (test carefully)
2. Increase P coefficient for faster recovery
3. Reduce `INDEX_TICKS` if safe
4. Increase `IDLE_VELOCITY` for faster restarts

### Problem: Missed Shots

**Symptoms:**
- Samples don't enter shooter
- Flipper fires before sample is ready

**Solutions:**
1. Increase `FLIP_TIME_MS`
2. Increase `INDEX_TICKS`
3. Verify flipper servo is properly calibrated
4. Check collector alignment with shooter

### Problem: Velocity Oscillation

**Symptoms:**
- Velocity bounces up and down
- Never reaches stable target
- WAITING_VELOCITY takes too long

**Solutions:**
1. Reduce P coefficient
2. Check for mechanical binding
3. Verify encoder is connected properly
4. Add small D coefficient (0.1-0.5)

### Problem: Battery Drain

**Symptoms:**
- Robot dies quickly during match
- Voltage drops below 11V

**Solutions:**
1. Reduce `IDLE_VELOCITY` to 0 or 300
2. Only spin up shooter when needed
3. Check for other power-hungry systems
4. Use fresh, high-quality batteries

---

## Recommended Testing Sequence

### 1. Baseline Test (5 minutes)
- Deploy code as-is
- Run 5x three-shot sequences
- Record all pre/post velocities
- Note any failures or issues

### 2. PIDF Tuning (10 minutes)
- Measure max velocity
- Test P values: 1.0, 1.5, 2.0
- Choose value with fastest recovery

### 3. Timing Optimization (10 minutes)
- Video record flipper motion
- Test reduced flip times
- Find minimum reliable value

### 4. Velocity Testing (10 minutes)
- Test different target velocities
- Measure shot accuracy at each
- Choose optimal for your game strategy

### 5. Validation (5 minutes)
- Run 10x three-shot sequences
- Verify >95% reliability
- Confirm acceptable cycle time

---

## Performance Targets

### Good Performance
- Cycle time: 2.0-2.5 seconds (3 shots)
- Pre-velocity variance: ±30 tps
- Post-velocity drop: <200 tps
- Reliability: >90%

### Excellent Performance
- Cycle time: 1.5-2.0 seconds (3 shots)
- Pre-velocity variance: ±20 tps
- Post-velocity drop: <150 tps
- Reliability: >98%

### Competition-Ready
- Cycle time: <1.5 seconds (3 shots)
- Pre-velocity variance: ±15 tps
- Post-velocity drop: <100 tps
- Reliability: >99%

---

## Advanced Tuning

### Battery Voltage Compensation
The feed-forward term automatically compensates for battery drain:
```java
double F = (32767.0 / maxVelocity) * (12.0 / batteryVoltage);
```

This means your shooter should maintain consistent performance from 13V down to 11V.

### Per-Shot Velocity Analysis
Use the telemetry after completion to analyze each shot:
```
Shot 1: Pre=1502, Post=1398  (drop: 104)
Shot 2: Pre=1498, Post=1385  (drop: 113)
Shot 3: Pre=1505, Post=1390  (drop: 115)
```

Consistent drops indicate good PIDF tuning.

### State Machine Debugging
Watch the state transitions in telemetry:
```
IDLE → WAITING_VELOCITY → FLIPPING_UP → FLIPPING_DOWN →
INDEXING → RECOVERING → WAITING_VELOCITY → ...
```

If stuck in any state, check the corresponding timeout or condition.

---

## Quick Reference

| Parameter | File | Line | Default | Range |
|-----------|------|------|---------|-------|
| Max Velocity | RobotHardware.java | 100 | 2800 | 2000-3500 |
| P Coefficient | RobotHardware.java | 102 | 1.5 | 0.5-3.0 |
| Flip Time | ThreeShots.java | 27 | 300ms | 200-400ms |
| Velocity Tolerance | ThreeShots.java | 28 | 30 | 15-50 |
| Idle Velocity | ThreeShots.java | 29 | 500 | 0-1000 |
| Recovery Timeout | ThreeShots.java | 30 | 100ms | 50-200ms |
| Index Ticks | ThreeShots.java | 25 | 1200 | 800-1500 |

---

## Support

If you encounter issues not covered here:
1. Check telemetry for error messages
2. Verify all hardware connections
3. Test individual components (shooter, collector, flipper) separately
4. Review state machine logic in `ThreeShots.java:100-194`

Good luck and happy tuning!
