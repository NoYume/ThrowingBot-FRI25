# Throwing Robot Arm Project - UT Austin RobIn FRI

## I. INTRODUCTION

The goal of the group is to develop and test a working
program that can be used on the Sawyer robot arms to catch
and throw with a human or another Sawyer robot arm. The
main goal is to focus on the throwing robot arm program,
which can perform a general-purpose tossing action. The robot
will learn to throw a tennis ball to keep the object consistent
with test runs and the final product. The throwing robot will
throw at a selected spot when run and never change the
selected target spot. If time is available, we plan to integrate a
vision model to detect human figures and toss a ball towards
them accurately.

## II. METHOD

Teaching the robot arm the throwing action with synthetic data calculated with inverse kinematics:

### • Reinforcement learning (Neural network)

– Implement one main learning process movements throwing
the object, the robot holds the ball before hand

-Create synthetic data based on pre determined
parameters using a kinematic equations, if given
target (x,y), calculate throwing angle, velocity, and
joint angle

– Use a multi-layer neural network to predict the prob-
ability of throwing success across a sample data
generated with inverse kinematic equations
for the tennis ball

– Learn from trial and error without explicit supervi-
sion. The success will be awarded from the accuracy
of throws, the distance of the thrown object, the
proper arch-like projectile path, the speed to land
on the target, and hitting the decided target

– The network will also learn from physics-based
predictions to compensate for unmodeled dynamics.
This will combine analytical models and deep learn-
ing to improve throwing accuracy

– Once satisfied, move to the physical robot and test
out how close it performs to the simulation and
consistency
