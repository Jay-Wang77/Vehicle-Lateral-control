# Vehicle-Lateral-control

## Pure Pursuit
![Geometric Bicycle Model](https://img-blog.csdnimg.cn/direct/44d5a88dd60344e799b18b7106ceacc5.png)
**Geometric Bicycle Model**

A common simplification of an Ackerman steered vehicle used for geometric path tracking is the bicycle model.

$$\tan \delta = \frac{L}{R}$$

This model approximates the motion of a car reasonably well at low speeds and moderate steering angles.

where
- $\delta$ is the steering angle of the front wheel,
- $L$ is the distance between the front axle and rear axle (wheelbase),
- $R$ is the radius of the circle that the rear axle will travel along at the given steering angle.

*This model approximates the motion of a car reasonably well at low speeds and moderate steering angles.*

![Pure Pursuit geometry](https://img-blog.csdnimg.cn/direct/34234dbe746246b586b26aa20f555ac5.png)
### 1. The Pure Pursuit Method

- The curvature of a circular arc that connects the rear axle location to a goal point on the path ahead of the vehicle.
- The goal point is determined from a look-ahead distance $l_d$ from the current rear axle position to the desired path.

Where $\sin \sin \sin \sin \sin \sin \alpha = \frac{e}{l_d}$ and $e$ is the lateral displacement from the path at some look-ahead distance $l_d$ in front of the vehicle.

**The vehicle's steering angle $\delta$ determination:**

The vehicle's steering angle $\delta$ can be determined using only the goal point location and the angle $\alpha$ between the vehicle's heading vector and the look-ahead vector. The relationship is given by:

$$\frac{l_d}{\sin(2\alpha)} = \frac{R}{\sin(\frac{\pi}{2} - \alpha)}$$
Simplifying, we get:

$$\frac{l_d}{2\sin(\alpha)\cos(\alpha)} = R$$

Which can be written as:

$$\frac{l_d}{\sin(2\alpha)} = 2R$$

This leads to the inverse relationship:

$$\frac{1}{R} = \frac{2\sin(\alpha)}{l_d}$$

The steering angle $\delta$ is related to the curvature by:

$$\tan(\delta) = \frac{L}{R}$$

Thus, the steering angle can be calculated as:

$$\delta = \tan^{-1}\left(\frac{L}{R}\right)$$

Substituting for $R$, we have:

$$\delta(t) = \tan^{-1}\left(\frac{2L\sin(\alpha(t))}{l_d}\right)$$

Where the look-ahead distance $l_d$ is constant and $\sin(\alpha)$ can be expressed as the ratio of the lateral offset $e$ at the look-ahead distance to $l_d$:

$$\sin(\alpha) = \frac{e_{l_d}}{l_d}$$

So the steering angle $\delta$ is given by:

$$\delta(t) = \tan^{-1}\left(\frac{2L}{l_d^2}e_{l_d}(t)\right)$$

The pure pursuit method acts as a **proportional controller** of the steering angle, operating on the cross-track error with a gain of $\frac{2L}{l_d^2}$.

**Basic Procedures:**

1. Determine the current location of the vehicle.
2. Find the path point closest to the vehicle.
3. Identify the goal point $G$.
4. Transform the goal point to the vehicle's coordinate system.
5. Calculate the curvature and request the vehicle to set the steering to that curvature:

   $$\delta_t = \tan^{-1}\left(\frac{2L}{l_d^2}e_{l_d}(t)\right)$$

6. Update the vehicle's position.

### 2. Pure Pursuit Tuning
**Most Common Way:**

- Scaling the look-ahead distance ($l_d$) with speed.
- Establishing a range for look-ahead distance: [min, max].

**Characteristic:**

1. A short look-ahead distance provides more accurate tracking, whereas a longer distance ensures smoother tracking.
2. A $k$ value that is too small may result in instability, while a $k$ value that is too large can lead to poor tracking.
3. A high level of robustness is maintained, for example, ensuring good handling of discontinuities in the path.

The steering angle $\delta(t)$ is calculated as follows:

$$
\delta(t) = \tan^{-1}\left(\frac{2L \sin(\alpha(t))}{l_d}\right)
$$

Which can also be expressed as:

$$
\delta(t) = \tan^{-1}\left(\frac{2L \sin(\alpha(t))}{k v_x(t)}\right)
$$

Here, $k$ is a proportionality constant that adjusts the look-ahead distance based on the vehicle's current speed $v_x(t)$.

### 3. How to Pick the Best Look-Ahead Distance

Selecting the optimal look-ahead distance is a nuanced task. Commonly, the look-ahead distance varies with speed—higher speeds typically necessitate a longer look-ahead distance to accommodate the increased reaction time and maintain a smoother trajectory. 

However, it's logical to consider that the look-ahead distance should also account for:

- **Path Curvature**: Sharper curves may require shorter look-ahead distances to allow for more responsive steering adjustments.
- **Cross-Track Error**: A significant deviation from the path could justify reducing the look-ahead distance to correct the vehicle's position more promptly.
- **Longitudinal Velocity**: As mentioned, a direct correlation with speed, where the look-ahead distance increases with vehicle velocity to ensure there's sufficient time to react to path changes.
- 
## Stanley Method
### 1. Stanley Method

The Stanley method is a path tracking algorithm developed by Stanford University's autonomous vehicle entry in the DARPA Grand Challenge, named Stanley.
![请添加图片描述](https://img-blog.csdnimg.cn/direct/349393c053b0436fa73df5368cfeb80f.png)


### 2. Core Concept

- Utilizes the center of the front axle as the reference point.
- Considers both the heading error and the position error relative to the closest point on the path.
- Establishes an intuitive steering law that:
  - Corrects heading error.
  - Corrects position error.
  - Respects the bounds of maximum steering angle.

### 3. Three Key Requirements

1. **Align Heading with Desired Heading:**
   - Steering is proportional to heading error: $\theta_e$.
    $$\delta(t) = \theta_e(t)$$

2. **Eliminate Cross Track Error:**
   - Proportional to the error $e$.
   - Inversely proportional to speed to avoid oversteering at high velocities.
	- Limits the effect for large errors using the inverse tangent function.
   - The gain $k$ is determined experimentally.
   $$\delta(t) = \delta_e(t) = \tan^{-1} \left(\frac{k e(t)}{v_f(t)}\right)$$

3. **Maximum and Minimum Steering Angles:**
   - The steering angle $\delta(t)$ is bounded within a specified range: 
   $$\delta(t) \in [\delta_{min}, \delta_{max}]$$

By combining these elements, the Stanley method provides robust and intuitive control for autonomous vehicles, effectively addressing both heading and lateral position errors for precise path tracking.

### 4. Stanley Control Law

- The Stanley control law for path tracking is defined as:
  
  $$\delta(t) = \theta_e(t) + \tan^{-1}\left(\frac{k e(t)}{v_f(t)}\right), \quad \delta(t) \in [\delta_{min}, \delta_{max}]$$

- In the absence of heading error, or in cases of large cross track error, the steering angle is limited by both $\delta_{max}$ and the arctan function within the range:

  $$\tan^{-1}(e) \in \left[-\frac{\pi}{2}, \frac{\pi}{2}\right]$$

### 5. For Large Heading Errors

- For significant heading errors, the vehicle will steer in the opposite direction of the error, with steering corrections increasing as the heading error increases.
- Steering corrections are fixed at the limit beyond the maximum steering angle, assuming no cross track error is present.

### 6. For Large Positive Cross Track Errors

- For large positive cross track errors, the steering angle adjustment is calculated as:

  $$\tan^{-1}\left(\frac{k e(t)}{v_f(t)}\right) \approx \frac{\pi}{2} - \delta(t) \approx \theta_e(t) + \frac{\pi}{2}$$

- As the steering angle changes, the heading correction counters the cross track correction, driving the steering angle back towards zero.
- As the vehicle approaches the path, the cross track error decreases, and the steering command adjusts to align the heading correctly.

### 7. Error Dynamics

- When not at the maximum steering angle, the error dynamics are described by:

  $$\dot{e}(t) = -v_f(t) \sin(\delta_e) = -v_f(t) \sin\left(\tan^{-1}\left(\frac{k e(t)}{v_f(t)}\right)\right)$$
  
  Which simplifies to:

  $$\dot{e}(t) = -\frac{k e(t)}{\sqrt{1 + \left(\frac{k e(t)}{v_f(t)}\right)^2}}$$

- For a small cross track error, this leads to an exponential decay characteristic:

  $$\dot{e}(t) \approx -k e(t)$$

This formulation indicates that the error dynamics for small errors approximate a first-order response, with the error reducing exponentially over time.

### 8. Stanley Method Adjustment
1. **Low Speed Operation**
   - Inverse speed can cause numerical instability.
   - Add a positive softening constant to the controller:

  $$ \delta(t) = \theta_e(t) + \tan^{-1}\left(\frac{k e(t)}{k_s + v_f(t)}\right) $$

2. **Extra Damping on Heading**
   - Becomes an issue at higher speed in real vehicle scenarios.

3. **Steer into Constant Radius Curves**
   - Improves tracking on curves by adding a feedforward term on heading.

### 9. Stanley Method Summary

- The Stanley method is more intuitive to tune compared to Pure Pursuit, but it suffers from similar pitfalls when tuning.
- The Stanley tracker can be over-tuned to a specific course in a similar manner because the only way it can overcome dynamic effects is with a high gain that may lead to instability on other paths.
- In contrast to Pure Pursuit, a well-tuned Stanley tracker will not "cut corners" but rather overshoot turns. This effect can be attributed to not having a look-ahead.
- Similar to the Pure Pursuit method, steady-state errors in curves at moderate speeds become significant.
