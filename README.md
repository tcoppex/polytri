[![unlicense](https://img.shields.io/badge/Unlicense-%23373737)](https://unlicense.org/)
![language: c++17](https://img.shields.io/badge/c++-17-blue.svg)
![Stage: alpha](https://img.shields.io/badge/-alpha-red)

# polytri

_**polytri**_ is a polygon triangulator for [simple polygons](https://en.wikipedia.org/wiki/Simple_polygon) _without holes_ based on Raimund Seidel's algorithm [1].


[![comparison.webp](https://i.postimg.cc/QCds9qkX/comparison.webp)](https://postimg.cc/cg2PPwbj)

<!--
## Quickstart

```bash
# Build the demo.
cmake -B build . -DCMAKE_BUILD_TYPE=Release
cmake --build build

# Triangulate a simple shape.
./build/polytri-cli ./shape.data

# Display it on the browser.
firefox -new-window ./tools/polygon.html
```
-->

#### Usage

```cpp
/* Define this in a single compilation unit. */
#define POLYTRI_IMPLEMENTATION
#include "polytri/polytri.hpp"

/* Any type goes if they have public x / y attributes. */
template<typename T>
struct Vertex_t {
  T x{};
  T y{};
};

/* The vertices list is expect to be a Container-type. */
template<typename T>
using Contour_t = std::vector<Vertex_t<T>>;

int main(int argc, char *argv[])
{
  /* Contour must be in counter clockwise order. */
  std::vector<Contour_t<float>> polygons = {
    { {-10.0, -9.0}, {11.0, -12.0}, {0.0, 8.0}, {-5.0, 11.0} }
  };

  auto indices = PolyTri::Triangulate( polygons );

  for (auto const& i : indices) {
    std::cerr << i << " ";
  }
  std::cerr << "\n";

  return 0;
}

```

#### Limitations

Triangulating polygons with _holes_ is technically possible but not currently supported. You can check those libraries for alternatives :

- [Triangle](http://www.cs.cmu.edu/~quake/triangle.html), A Two-Dimensional Quality Mesh Generator and Delaunay Triangulator.
- [mapbox/earcut.hpp](https://github.com/mapbox/earcut.hpp), A C++ port of earcut.js, a fast, header-only polygon triangulation library.

---

### References

1. R. Seidel. [*A simple and fast incremental randomized algorithm for computing trapezoidal decompositions and for triangulating polygons*](https://www.sciencedirect.com/science/article/pii/0925772191900124?via%3Dihub). Comput. Geom. Theory Appl., 1:51–64, 1991.
2. A. Narkhede and D. Manocha, [*Fast Polygon Triangulation Based on Seidel's Algorithm*](https://www.cs.unc.edu/~dm/CODE/GEM/chapter.html), Department of Computer Science, UNC Chapel Hill.
3. Fournier, Alain & Montuno, Delfin. (1984). [*Triangulating Simple Polygons and Equivalent Problems*](https://dl.acm.org/doi/10.1145/357337.357341). ACM Trans. Graph.. 3. 153-174. 10.1145/357337.357341.
4. M. de Berg, O. Cheong, M. van Kreveld, M. Overmars. [*Computational Geometry, Algorithms and Applications, Third Edition*](https://link.springer.com/book/10.1007/978-3-540-77974-2), Chap. 3 Polygon Triangulation, Springer, 2008.

