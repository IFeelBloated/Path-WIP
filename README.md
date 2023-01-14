# Features #
- Unbiased Monte Carlo path tracing
- Global illumination: diffuse interreflection (color bleed), soft shadows, caustics, ambient occlusion, indirect lighting, etc.
- Anti-aliased sampling (Gaussian kernel)
- Finite-size lens (depth of field)
- Multiple materials
 - emissive
 - diffuse
 - specular metal
 - specular glass
 - rough metal
 - frosted glass
 - plastic/porcelain (diffuse + specular/glossy reflection)
- (Multiple) importance sampling for each material
 - Power heuristics
- Fresnel for dielectrics
- Russian roulette path selection/termination

![](a.png)