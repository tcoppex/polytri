// ----------------------------------------------------------------------------
//
//  polytri-benchmark
//
//     Benchmark of triangulation libraries on polygons with holes.
//
//  For each case and each library : median time of one call, and the peak heap
//  memory allocated during one call (measured by replacing operator new, so it
//  includes the input copies and the output, but not the polygon itself).
//  Every result is validated (triangle count and area) outside of the timings.
//
// To add a library : write an adapter `std::vector<uint32_t> run_xxx(const Shape&)`
// returning triangle indices into the concatenated contours, and register it in
// `libraries()`.
//
// ----------------------------------------------------------------------------

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <algorithm>
#include <array>
#include <vector>
#include <chrono>
#include <functional>
#include <random>
#include <string>
#include <exception>

#include <mapbox/earcut.hpp>
#include <CDT.h>
#include <poly2tri/poly2tri.h>

#include "polytri/polytri.hpp"


// ----------------------------------------------------------------------------
//  Heap tracking
// ----------------------------------------------------------------------------

namespace heap {

constexpr size_t kHeader = 16;

static size_t live = 0, peak = 0;

void *alloc(size_t n) {
  void *p = std::malloc(n + kHeader);
  if (!p) throw std::bad_alloc();
  *static_cast<size_t *>(p) = n;
  live += n;
  if (live > peak) peak = live;
  return static_cast<char *>(p) + kHeader;
}

void release(void *p) noexcept {
  if (!p) return;
  void *base = static_cast<char *>(p) - kHeader;
  live -= *static_cast<size_t *>(base);
  std::free(base);
}

}  // namespace heap

void *operator new(size_t n) { return heap::alloc(n); }
void *operator new[](size_t n) { return heap::alloc(n); }
void operator delete(void *p) noexcept { heap::release(p); }
void operator delete[](void *p) noexcept { heap::release(p); }
void operator delete(void *p, size_t) noexcept { heap::release(p); }
void operator delete[](void *p, size_t) noexcept { heap::release(p); }

// ----------------------------------------------------------------------------
//  Shapes
// ----------------------------------------------------------------------------

using Pt = std::array<double, 2>;
using Shape = std::vector<std::vector<Pt>>;  // first contour = outer, others = holes

static
size_t vertex_count(const Shape &s) {
  size_t n = 0;
  for (const auto &c : s) n += c.size();
  return n;
}

// Star shaped contour : evenly spread angles, jittered radius (always simple).
static
std::vector<Pt> star(std::mt19937_64 &rng, size_t n, double cx, double cy, double rmin, double rmax) {
  std::uniform_real_distribution<double> u(0.0, 1.0);
  std::vector<Pt> c(n);
  for (size_t i = 0; i < n; ++i) {
    const double a = 2.0 * M_PI * (static_cast<double>(i) + 0.8 * u(rng)) / static_cast<double>(n);
    const double r = rmin + (rmax - rmin) * u(rng);
    c[i] = {cx + r * std::cos(a), cy + r * std::sin(a)};
  }
  return c;
}

// Outer star of `n_outer` vertices (radius in [0.7, 1] * 1000) with `holes` holes of
// `n_hole` vertices, each one in its own grid cell inside the inner disc : they
// can touch neither each other nor the outer contour.
static
Shape make_shape(size_t n_outer, size_t holes, size_t n_hole, uint64_t seed) {
  std::mt19937_64 rng(seed);
  Shape s;
  s.push_back(star(rng, n_outer, 0, 0, 700.0, 1000.0));
  if (0 == holes) return s;

  const double half = 700.0 / std::sqrt(2.0);  // square inscribed in the inner disc
  const size_t g = static_cast<size_t>(std::ceil(std::sqrt(static_cast<double>(holes))));
  const double cell = 2.0 * half / static_cast<double>(g);
  for (size_t k = 0; k < holes; ++k) {
    const double cx = -half + cell * (static_cast<double>(k % g) + 0.5);
    const double cy = -half + cell * (static_cast<double>(k / g) + 0.5);
    s.push_back(star(rng, n_hole, cx, cy, 0.15 * cell, 0.4 * cell));
  }
  return s;
}

static
double signed_area(const std::vector<Pt> &c) {
  double a = 0;
  for (size_t i = 0; i < c.size(); ++i) {
    const auto &p = c[i], &q = c[(i + 1) % c.size()];
    a += p[0] * q[1] - q[0] * p[1];
  }
  return 0.5 * a;
}

// ----------------------------------------------------------------------------
//  Libraries
// ----------------------------------------------------------------------------

using Indices = std::vector<uint32_t>;

static
Indices run_polytri(const Shape &s) {
  return polytri::triangulate<uint32_t>(s);
}

static
Indices run_earcut(const Shape &s) {
  return mapbox::earcut<uint32_t>(s);
}

static
Indices run_cdt(const Shape &s) {
  std::vector<CDT::V2d<double>> v;
  std::vector<CDT::Edge> e;
  v.reserve(vertex_count(s));
  e.reserve(vertex_count(s));

  for (const auto &c : s) {
    const auto first = static_cast<CDT::VertInd>(v.size());
    for (const auto &p : c) {
      v.push_back(CDT::V2d<double>(p[0], p[1]));
    }
    for (size_t i = 0; i < c.size(); ++i) {
      e.emplace_back(first + static_cast<CDT::VertInd>(i), first + static_cast<CDT::VertInd>((i + 1) % c.size()));
    }
  }

  CDT::Triangulation<double> t(
    CDT::VertexInsertionOrder::Auto,
    CDT::IntersectingConstraintEdges::NotAllowed,
    0.0
  );
  t.insertVertices(v);
  t.insertEdges(e);
  t.eraseOuterTrianglesAndHoles();

  Indices out;
  out.reserve(3 * t.triangles.size());
  for (const auto &tri : t.triangles) {
    for (int k = 0; k < 3; ++k) {
      out.push_back(tri.vertices[k]);
    }
  }
  return out;
}

static
Indices run_poly2tri(const Shape &s) {
  std::vector<p2t::Point> pts{};
  pts.reserve(vertex_count(s));

  for (const auto &c : s) {
    for (const auto &p : c) {
      pts.emplace_back(p[0], p[1]);
    }
  }

  std::vector<std::vector<p2t::Point *>> lines(s.size());
  size_t k = 0;
  for (size_t i = 0; i < s.size(); ++i) {
    lines[i].reserve(s[i].size());
    for (size_t j = 0; j < s[i].size(); ++j) {
      lines[i].push_back(&pts[k++]);
    }
  }
  p2t::CDT cdt(lines[0]);
  for (size_t i = 1; i < lines.size(); ++i) {
    cdt.AddHole(lines[i]);
  }
  cdt.Triangulate();

  Indices out;
  for (auto *t : cdt.GetTriangles()) {
    for (int j = 0; j < 3; ++j) {
      out.push_back(static_cast<uint32_t>(t->GetPoint(j) - pts.data()));
    }
  }
  return out;
}

struct Library {
  const char *name;
  std::function<Indices(const Shape &)> run;
  size_t max_vertices;  // 0 : no limit (slower algorithms are skipped above it)
};

static
std::vector<Library> libraries() {
  return {
    {"polytri",   run_polytri, 0},
    {"earcut",    run_earcut, 0},
    {"CDT",       run_cdt, 0},
    {"poly2tri",  run_poly2tri, 0},
  };
}


// ----------------------------------------------------------------------------
//  Validation : triangle count and total area.
// ----------------------------------------------------------------------------

static
bool validate(const Shape &s, const Indices &idx, std::string &why) {
  const size_t n = vertex_count(s);
  if (idx.empty() || idx.size() % 3 != 0) {
    why = "no output";
    return false;
  }

  std::vector<Pt> all;
  all.reserve(n);
  for (const auto &c : s) {
    all.insert(all.end(), c.begin(), c.end());
  }

  double area = 0;
  for (size_t i = 0; i < idx.size(); i += 3) {
    if (idx[i] >= n || idx[i + 1] >= n || idx[i + 2] >= n) {
      why = "bad index";
      return false;
    }
    const auto &a = all[idx[i]], &b = all[idx[i + 1]], &c = all[idx[i + 2]];
    area += std::abs((b[0] - a[0])
          * (c[1] - a[1]) - (c[0] - a[0])
          * (b[1] - a[1]))
          * 0.5;
  }

  double expected = std::abs(signed_area(s[0]));
  for (size_t i = 1; i < s.size(); ++i) {
    expected -= std::abs(signed_area(s[i]));
  }

  const size_t want = n - 2 + 2 * (s.size() - 1);
  if (idx.size() / 3 != want) {
    why = "triangles " + std::to_string(idx.size() / 3) + " != " + std::to_string(want);
    return false;
  }
  if (std::abs(area - expected) > 1e-6 * expected) {
    why = "area mismatch";
    return false;
  }
  return true;
}

// ----------------------------------------------------------------------------
//  Measures
// ----------------------------------------------------------------------------

using Clock = std::chrono::steady_clock;
static volatile size_t g_sink;

struct Measure {
  bool ok = false;
  std::string why;
  double ms = 0;       // median
  double peak_mib = 0; // peak heap during one call
};

static
Measure measure(const Library &lib, const Shape &s) {
  Measure m;

  // Memory + validation (one call, outside of the timings).
  {
    const size_t base = heap::live;
    heap::peak = base;
    try {
      auto idx = lib.run(s);
      m.peak_mib = static_cast<double>(heap::peak - base) / (1024.0 * 1024.0);
      m.ok = validate(s, idx, m.why);
    } catch (const std::exception &e) {
      m.why = std::string("exception: ") + e.what();
    }
  }
  if (!m.ok) return m;

  // Time : repeat for about 0.3 s (3 to 200 runs), take the median.
  std::vector<double> t;
  double total = 0;
  while (((t.size() < 3) && (total < 1500.0)) || ((total < 300.0) && (t.size() < 200))) {
    const auto t0 = Clock::now();
    auto idx = lib.run(s);
    g_sink = idx.size();
    const double ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    t.push_back(ms);
    total += ms;
  }
  std::sort(t.begin(), t.end());
  m.ms = t[t.size() / 2];
  return m;
}

// ----------------------------------------------------------------------------

struct Case {
  std::string name;
  Shape shape;
};

int main(int argc, char **argv) {
  bool big = false;
  const char *csv_path = nullptr;

  for (int i = 1; i < argc; ++i) {
    if (0 == std::strcmp(argv[i], "--big")) {
      big = true;
    } else if (0 == std::strcmp(argv[i], "--csv") && i + 1 < argc) {
      csv_path = argv[++i];
    } else {
      fprintf(stderr, "usage: %s [--big] [--csv file]\n", argv[0]);
      return EXIT_FAILURE;
    }
  }

  std::vector<Case> cases;
  std::vector<size_t> sizes = {100, 1000, 10000, 100000};
  if (big) sizes.push_back(1000000);
  for (size_t n : sizes) {
    cases.push_back({"star", make_shape(n, 0, 0, 1)});
    cases.push_back({"star + 4 holes", make_shape(n / 2, 4, std::max<size_t>(n / 8, 3), 2)});
    cases.push_back({"star + many holes", make_shape(n / 2, std::max<size_t>(n / 20, 1), 10, 3)});
  }

  const auto libs = libraries();
  FILE *csv = csv_path ? std::fopen(csv_path, "w") : nullptr;
  if (csv) {
    fprintf(csv, "case,vertices,holes,library,ok,ms,peak_mib,failure\n");
  }

  printf("%-20s %9s %6s", "case", "vertices", "holes");
  for (const auto &l : libs) {
    printf(" | %-17s", l.name);
  }
  printf("\n%-20s %9s %6s", "", "", "");
  for (size_t i = 0; i < libs.size(); ++i) {
    printf(" | %8s %8s", "ms", "MiB");
  }
  printf("\n");

  for (const auto &c : cases) {
    const size_t n = vertex_count(c.shape);
    printf("%-20s %9zu %6zu", c.name.c_str(), n, c.shape.size() - 1);
    for (const auto &l : libs) {
      if (l.max_vertices && n > l.max_vertices) {
        printf(" | %17s", "-");
        continue;
      }
      const auto m = measure(l, c.shape);

      if (m.ok) {
        printf(" | %8.3f %8.2f", m.ms, m.peak_mib);
      } else {
        printf(" | %-17.17s", ("FAIL " + m.why).c_str());
      }

      if (csv) {
        fprintf(
          csv,
          "%s,%zu,%zu,%s,%d,%.4f,%.3f,%s\n",
          c.name.c_str(),
          n,
          c.shape.size() - 1,
          l.name,
          m.ok ? 1 : 0, m.ms,
          m.peak_mib,
          m.why.c_str()
        );
      }
      std::fflush(stdout);
    }
    printf("\n");
  }

  if (csv) {
    std::fclose(csv);
  }

  return EXIT_SUCCESS;
}

// ----------------------------------------------------------------------------
