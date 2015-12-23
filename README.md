# BetterTeachMap

Filter anchor point dynamically during a Teach and Repeat.

## Installation

### Dependencies

- [libpointmatcher](https://github.com/ethz-asl/libpointmatcher)
- [lemon](https://lemon.cs.elte.hu/trac/lemon)
- Boost

## Usage

```
optimizeteachrepeatmain -m teach -a 0.2 -b 0.1 -i ../IcpConfig.yaml
```

Where `a` is the desired tolerance to error in the x axis, `b` the desired tolerance in the y axis, `i` a path to an ICP config file (in the [libpointmatcher](https://github.com/ethz-asl/libpointmatcher) format) and `m` is the path to a teach and repeat map (in the [husky_trainer](https://github.com/MobileRobotics-Ulaval/husky-trainer) format).
