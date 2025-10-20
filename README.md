![Stage: alpha](https://img.shields.io/badge/-alpha-red)

# polytri

PolyTri is a polygon triangulator for [simple polygons](https://en.wikipedia.org/wiki/Simple_polygon) based on Seidel's algorithm [1].

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

---

## References

- [1] R. Seidel. *A simple and fast incremental randomized algorithm for computing trapezoidal decompositions and for triangulating polygons*. Comput. Geom. Theory Appl., 1:51–64, 1991.
- [2] A. Narkhede and D. Manocha, *Fast Polygon Triangulation Based on Seidel's Algorithm*
- [3] Fournier, Alain & Montuno, Delfin. (1984). *Triangulating Simple Polygons and Equivalent Problems*. ACM Trans. Graph.. 3. 153-174. 10.1145/357337.357341. 
- [4] M. de Berg, O. Cheong, M. van Kreveld, M. Overmars. *Computational Geometry, Algorithms and Applications, Third Edition*, Chap. 3 Polygon Triangulation, Springer, 2008.

## Alternative

- [Triangle](http://www.cs.cmu.edu/~quake/triangle.html), A Two-Dimensional Quality Mesh Generator and Delaunay Triangulator.

## License

*PolyTri* is released under the [Unlicense](<http://unlicense.org>).
