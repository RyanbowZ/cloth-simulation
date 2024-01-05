Assignment 5 — Implicit Cloth

Name: Yuanlong Zhou
UIN: 634005710
Email: ryanbowz@tamu.edu

----------
Summary: Successfully completed all of the task points (including cloth setup, explicit integration, implicit integration, sphere collision and sparse matrices).

The final submission is an implicit integration version with sphere collision implementation using sparse matrices. To debug explicit integration, simply replace the parameter computations with the corresponding equation.

The performance of the code is optimized in several ways, including using vectors to store the massive triplets to avoid space and computing redundancy. In release mode, each step can be executed in less than 10ms.

-----------
Parameter Exploration:
1. Stiffness Parameter:
The reasonable range for the stiffness parameter is (1,100].
If the stiffness parameter is lower than 1, then the cloth will be too loose, like a liquid, losing the tension of normal material. The non-fixed point will continuously fall down by the influence of gravity and too small string force, it will stretch longer than the normal situation of cloth materials.
If the stiffness parameter is greater than 100, the cloth will be too tight and dense, just like a panel, without soft feelings. If we increase the stiffness parameter to even larger, the cloth will not function normally, it will explode and fall into a chaos.
2. Time Step Parameter:
The reasonable range for the Time Step parameter is (2e-3,1e-2].
If the time step is too low, the animation will be rendered too slowly. Although we can still get the correct cloth simulation result, we need to wait for a longer time to view the effects.
However, if the time step is too large, the animation will not function properly. The cloth is highly likely to be exploded and go into a chaos. This is because it exceeds the computing limits that allow for a correct simulation and rendering. Meanwhile, it worth mentioning that as the resolution increases, it is advised to increase the time step parameter a little bit within the reasonable range for a smoother rendering.  
3. Highest resolution on my computer:
I used MacBook Pro for this assignment with 2.6 GHz 6-Core Intel i7 CPU and AMD Radeon Pro 5300M Graphics Card.
The highest resolution I can achieve with a proper simulation on my computer is 45x45. If the resolution exceeds this value, first the performance will be very slow, second the cloth is very unstable, which is highly likely to explode, leading to an incorrect simulation result.


