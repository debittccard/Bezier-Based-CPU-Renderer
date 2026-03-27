# Bezier-Based-CPU-Tube-Renderer

A CPU-based 3D renderer that generates and rasterizes tube geometry from cubic Bezier curves using a custom software graphics pipeline.

## Features

- Bezier curve -> tube mesh generation  
- Software rasterization (no GPU)  
- Multithreaded rendering  
- Frustum and basic distance culling  
- Shadow mapping  
- Configurable camera and projection system  

## Architecture

- **scene.c** - scene traversal and tube rendering  
- **render.c** - rasterization and pixel output  
- **math.c** - vector and matrix math  
- **types.h** - core data structures and constants  

## Pipeline

1. Bezier curves are defined in world space  
2. Curves are tessellated into tube meshes  
3. Vertices are transformed (model → view → projection)  
4. Geometry is culled (frustum / distance)  
5. Triangles are rasterized on CPU  
6. Depth buffering resolves visibility  
7. Lighting and shadows are applied  
8. Image is written to a PPM file  

## Build

```bash
gcc -O3 -march=native -ffast-math -funroll-loops -flto \
-pthread main.c scene.c renderer.c geometry.c math.c -lm -o MTCPUR
make
```
## CLI COMMANDS


### Geometry

1.-t <int>
Number of tubes (default: 100000)

2.-seg <int>
Segments per tube (default: 20)

3.-sid <int>
Sides per tube (default: 12)

4.-r <float>
Tube radius (default: 0.05)

5.-scale <float>
Global scene scale (default: 1.5)

### Curve Control (Bezier)

1.-p0 x,y,z
2.-p1 x,y,z
3.-p2 x,y,z
4.-p3 x,y,z

Defines a custom cubic Bezier curve using 4 control points.

If any of -p0 through -p3 are provided, all four must be specified.

### Transformations

1.-rx <float>  Rotation multiplier (X)
2.-ry <float>  Rotation multiplier (Y)
3.-rz <float>  Rotation multiplier (Z)

4.-rcx <float> Constant rotation (X)
5.-rcy <float> Constant rotation (Y)
6.-rcz <float> Constant rotation (Z)

-as <float>  Rotation step per frame (default: 0.001)

### Translation

1.-tx <float> Translate X
2.-ty <float> Translate Y
3.-tz <float> Translate Z

### Camera

1.-cx <float> Camera X position
2.-cy <float> Camera Y position
3.-cz <float> Camera Z position (default: -200.0)

4.-fov <float> Field of view in degrees (default: 90)

5.-focus x,y,z
Set camera target (look-at point){Broken right now only x,y values from 0-100 working, keep at 0,0,0}

### Lighting

1.-lx <float> Light X
2.-ly <float> Light Y
3.-lz <float> Light Z

### Color
1. -rgb
Enable rainbow coloring

2. -color r,g,b
Set a solid RGB color (0–255)

### Output

1. -o <string>
Output file name (default: output.ppm)

2. -ow <int>
Output width (default: 1920)

3. -oh <int>
Output height (default: 1080)

### Examples

./renderer
Default render

./renderer -t 200000 -seg 30 -sid 16 -r 0.03 -ow 2560 -oh 1440
High quality render

./renderer -rgb
Rainbow coloring

./renderer -color 255,100,50
Solid color render

./renderer -cx 0 -cy 10 -cz -300 -lx 10 -ly 20 -lz -50 -fov 75
Custom camera and lighting

./renderer 
-p0 0,0,0 
-p1 10,20,5 
-p2 -10,5,15 
-p3 5,-20,25
Custom Bezier curve

#### Notes

* Output format is binary PPM (P6)
* Large tube counts increase memory usage
* All four curve points must be provided together
* Unknown arguments will terminate execution with a usage message

