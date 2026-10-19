![language: c++17](https://img.shields.io/badge/c++-17-blue.svg)
[![unlicense](https://img.shields.io/badge/Unlicense-%23373737)](https://unlicense.org/)

# polytri

**_polytri_** is a _fast_ header-only triangulation library for [simple polygons](https://en.wikipedia.org/wiki/Simple_polygon) with hole support, based on Raimund Seidel's randomized algorithm¹.

With an expected time complexity of $O(n \log^{*} n)$, it significantly outperforms alternative libraries on large-scale datasets (> 10⁴ vertices).
See [benchmark](#benchmark).

<!-- [![comparison.webp](https://i.postimg.cc/QCds9qkX/comparison.webp)](https://postimg.cc/cg2PPwbj) -->

### Usage

The library depends only on the _STL_ and has no build or link dependencies.

```cpp
#include <iostream>

/* Single header only. */
#include "polytri/polytri.hpp"

/* The internal PointTrait type works for types with public x / y members
 * and for anything indexable with [0] / [1] accessors (std::array, ..).  */
template<typename T>
struct Vertex {
  T x{};
  T y{};
};

/* The vertices list is expect to be a Container-type. */
template<typename T>
using Contour = std::vector<Vertex<T>>;

int main(int argc, char *argv[])
{
  /* The first contour is the outer boundary, the following ones are holes.
   * Contours can have any orientation and must not repeat their first point. */
  std::vector<Contour<float>> polygon{
    { {-10.0, -9.0}, {11.0, -12.0}, {0.0, 8.0}, {-5.0, 11.0} },   // outer boundary
    { {-2.0, -2.0}, {-2.0, 2.0}, {2.0, 0.0} },                    // hole
  };

  /* Basic usage: Returns an std::vector<uint32_t > of indices. */
  {
    auto indices = polytri::triangulate( polygon );

    for (auto const& i : indices) {
      std::cerr << i << " ";
    }
    std::cerr << std::endl;
  }

  /* Alternatively we could define a reusable Context and use any unsigned
   * index type as outputs. */
  {
    polytri::Context context{};
    std::vector<uint16_t> small_indices{};

    /* The returned Result type can be casted as a boolean and hold an error
     * message when it fails. */
    if (auto ok = context.triangulate( polygon, small_indices ); !ok) {
      std::cerr << ok.message() << std::endl;
      return -1;
    }

    for (auto const& i : small_indices) {
      std::cerr << i << " ";
    }
    std::cerr << std::endl;
  }

  return 0;
}

```

#### PointTraits specialization

Any point type can be accepted by the library by specializing `polytri::PointTraits`.

```cpp
class MyPoint {
 public:
  MyPoint(double x, double y) : x_(x), y_(y) {}
  double get_x() const { return x_; }
  double get_y() const { return y_; }

 private:
  double x_, y_;
};

namespace polytri {
template<>
struct PointTraits<MyPoint> {
  static double x(const MyPoint &p) { return p.get_x(); }
  static double y(const MyPoint &p) { return p.get_y(); }
};
}

```

#### Requirements, Constraints & Notes

- Contours must be _[simple](https://en.wikipedia.org/wiki/Simple_polygon)_.
- Holes must lie inside the outer contour and must neither touch nor cross each other or the outer contour.
- Vertices must be distinct points (_no dupplicates_).
- Invalid input (_non simple polygon, fewer than 3 vertices, a contour repeating its first point_) is reported by a `polytri::Result` and an empty indices vector.
- All orientation tests are exact, so degenerate inputs (_collinear vertices, equal `x` or `y`_) are supported.
- The library will refuse to compile with `-ffast-math` (_it breaks the exact predicates_).
- Segments are inserted in a deterministic pseudo random order (using `POLYTRI_RANDOM_SEED`), which gives the expected running time. Without it ordered contours would take `O(n²)`.
- A `polytri::Context` keeps its memory between calls.

---

### Demos

The project is shipped with demos that will fetch their own dependencies via CMake `FetchContent`.


```bash
cmake -S demos -B build
cmake --build build

# Interactive shape editor using raylib.
./build/editor/polytri-editor

# Benchmark against other libraries (time, memory and peak heap).
./build/benchmark/polytri-benchmark --big --csv results.csv

# Produce diagrams from the previous results.
python3 demos/compare/plot.py results.csv --out plots
```

You can disable individual demo targets by setting the CMake option `POLYTRI_BUILD_${DEMO_NAME}`
to `OFF`:

```bash
# Generate build files without `polytri-editor`
cmake -S demos -B build -DPOLYTRI_BUILD_EDITOR=OFF
```

#### editor

A simple Raylib editor for plotting outer and hole vertices with automatic retriangulation.

- Left click to add a vertex.
- Left click near the first vertex to close the contour.
- Right click near a vertex to remove it.
- `R` key press to reset the canvas.

#### benchmark

A benchmark is provided to test the library against others popular triangulation
implementations :

- `earcut` (via [mapbox/earcut](https://github.com/mapbox/earcut/tree/v3.2.4)),
- `Constrained Delaunay Triangulation` (via [CDT](https://github.com/artem-ogre/CDT/tree/2.0.1) and [poly2tri](https://github.com/jhasse/poly2tri/commit/1008179917fd38c1824b8fe810f020fef0e5f6b5)).

On a _star with many holes_ shape, `polytri` outperform `earcut` above 10⁴ vertices and ranks second to it below that threshold.

[![polytri-time.png](https://i.postimg.cc/K8sFSJhX/polytri-time.png)](https://postimg.cc/186bD0zM)
[![polytri-ratio.png](https://i.postimg.cc/mBYDtsSF/polytri-ratio.png)](https://postimg.cc/YhTJ4DtV)
_(The benchmark was run on a 13th Gen Intel i7-13700HX (24) CPU @ 4.800GHz)._

Note that other metrics, such as triangle quality, must also be considered, for which _constrained Delaunay triangulation_ is widely regarded as giving the best results due to maintaining near-equilateral shapes without violating structural constraints.

---

### References

1. R. Seidel. [*A simple and fast incremental randomized algorithm for computing trapezoidal decompositions and for triangulating polygons*](https://www.sciencedirect.com/science/article/pii/0925772191900124?via%3Dihub). Comput. Geom. Theory Appl., 1:51–64, 1991.
2. A. Narkhede and D. Manocha, [*Fast Polygon Triangulation Based on Seidel's Algorithm*](https://www.cs.unc.edu/~dm/CODE/GEM/chapter.html), Department of Computer Science, UNC Chapel Hill.
3. Fournier, Alain & Montuno, Delfin. (1984). [*Triangulating Simple Polygons and Equivalent Problems*](https://dl.acm.org/doi/10.1145/357337.357341). ACM Trans. Graph.. 3. 153-174. 10.1145/357337.357341.
4. M. de Berg, O. Cheong, M. van Kreveld, M. Overmars. [*Computational Geometry, Algorithms and Applications, Third Edition*](https://link.springer.com/book/10.1007/978-3-540-77974-2), Chap. 3 Polygon Triangulation, Springer, 2008.


## (Un)License

This is free and unencumbered software released into the public domain.