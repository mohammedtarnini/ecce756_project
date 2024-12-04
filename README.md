# ECCE 756 Robotic Perception Project

![KU Logo](Images/kulogo.png)

## Submitted By
**Mohammed Tarnini**  
**ID:** 100049735  

**Submitted To**  
Prof. Jorge Dias  

---

## Part A: Kalman Filter and Particle Filter Tracking on ROAD-UAE Dataset

### Kalman Filter Tracking  
[![Kalman Filter Video](Videos/Kalman_Filter.mp4)](https://drive.google.com/file/d/1SLc1VCgwfsk34ouQoHYq00SYQyQ3As6s/view?usp=drive_link)  
[Download Video](https://drive.google.com/file/d/1SLc1VCgwfsk34ouQoHYq00SYQyQ3As6s/view?usp=drive_link)

### Particle Filter Tracking  
[![Particle Filter Video](Videos/Particle_Filter.mp4)](https://drive.google.com/file/d/1nFS2QjcDpSSR-EIi0hthBR1Ar6dSCexW/view?usp=sharing)  
[Download Video](https://drive.google.com/file/d/1nFS2QjcDpSSR-EIi0hthBR1Ar6dSCexW/view?usp=sharing)

---

## Part B: Underwater Aggregation and Dispersion of AUVs using Event-based Detection

### Detecting using events from v2e toolbox (inaccurate and unusable)  
[![v2e Demo Video](Videos/v2e_track_demo.mp4)](https://drive.google.com/file/d/1Hj5_Zy-ciNWZ3rUnNMphh3oVcWEQtIhW/view?usp=sharing)  
[Download Video](https://drive.google.com/file/d/1Hj5_Zy-ciNWZ3rUnNMphh3oVcWEQtIhW/view?usp=sharing)

### Detecting using events from SpikeCoding toolbox (more accurate and usable)  
[![SpikeCoding Demo Video](Videos/SpikeCoding_track_demo.mp4)](https://drive.google.com/file/d/1jwOVhf8d_WTJmg7xIYKtAuCg_U-rrwmq/view?usp=sharing)  
[Download Video](https://drive.google.com/file/d/1jwOVhf8d_WTJmg7xIYKtAuCg_U-rrwmq/view?usp=sharing)

---

### Calculating States of Detected AUVs  
![State Calculation](Images/states.png)  

Formulas:  
- \( d_\text{A} = \frac{h_\text{A}}{\beta} \)  
- \( \beta = \frac{h_\text{b}}{h_\text{f}} \cdot FOV_\text{v} \)  
- \( \Delta \theta = \frac{x_\text{c}}{w_\text{f}} \cdot FOV_\text{h} \)  

---

### Lennard-Jones Potential Function  

Potential function for attractive/repulsive forces:  
- \( V = 4\varepsilon\left[\left(\frac{\sigma}{r}\right)^{2n} - \left(\frac{\sigma}{r}\right)^n\right] \)  
- \( F = \frac{dV}{dr} = 4\varepsilon\left[-2n\left(\frac{\sigma^{2n}}{r^{2n+1}}\right) + n\left(\frac{\sigma^n}{r^{2n+1}}\right)\right] \)  

![Force Calculation](Images/Forces.png)

---

### Aggregation and Dispersion Behavior  

#### Aggregation using 16 AUVs inside a square boundary  
[![Aggregation Video](Aggregation.mp4)](https://drive.google.com/file/d/1gvD8SlYGDpqKb1Qagcyr6S5mC70ELUec/view?usp=sharing)  
[Download Video](https://drive.google.com/file/d/1gvD8SlYGDpqKb1Qagcyr6S5mC70ELUec/view?usp=sharing)

#### Dispersion using 16 AUVs inside a square boundary  
[![Dispersion Video](Dispersion.mp4)](https://drive.google.com/file/d/1ScYbKrkACsOV50BlK3L_imkKiRjoQO-9/view?usp=sharing)  
[Download Video](https://drive.google.com/file/d/1ScYbKrkACsOV50BlK3L_imkKiRjoQO-9/view?usp=sharing)
