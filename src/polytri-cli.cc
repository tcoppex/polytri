#include <cstdlib>
#include <iostream>
#include <fstream>

#include "polygon_triangulation.h"

/* -------------------------------------------------------------------------- */

namespace {

/** Read a data file to retrieve contour vertices */
int LoadSegments(char *const filename,
                 std::vector<uint32_t> &contour_lengths,
                 std::vector<vertex_t> &vertices)
{
  std::ifstream fd(filename);

  if (fd.fail()) {
    std::cerr << "Error : can't read the file." << std::endl;
    return EXIT_FAILURE;
  }

  unsigned int ncontours;
  fd >> ncontours;

  if (ncontours == 0u) {
    std::cerr << "Invalid contour count : " << ncontours << std::endl;
    return EXIT_FAILURE;
  }
  
  for (auto cid = 0u; cid < ncontours; ++cid) {
    unsigned int npoints;
    fd >> npoints;
    contour_lengths.push_back(npoints);

    for (auto i = 0u; i < npoints; ++i) {
      vertex_t v;
      fd >> v.x >> v.y;
      vertices.push_back(v);
    }
  }

  return EXIT_SUCCESS;
}

void ExportData(PolygonTriangulation::TriangleBuffer_t &triangles,
                 std::vector<vertex_t> &vertices)
{
  const char* filename = APP_DIRECTORY "tools/js/data.js";

  std::ofstream fd(filename);
  if (fd.fail()) {
    std::cerr << "Error : cannot locate the file \"" << filename << "\"" << std::endl;
    exit(EXIT_FAILURE);
  }

  fd << "var TRI = [" << std::endl;

  for (auto &t : triangles) {
    fd << "   [ "
       << t.v0 << ", "
       << t.v1 << ", "
       << t.v2 << "]," << std::endl;
  }
  fd << "];" << std::endl;
  fd << std::endl;

  fd << "var vertices = [" << std::endl;
  for (auto i = 0u; i < vertices.size(); ++i) {
    auto &v = vertices[i];
    fd << "   new TPoint( " << v.x << ", " << v.y << ")," << std::endl;
  }
  fd << "];" << std::endl;
}

}

/* -------------------------------------------------------------------------- */

int main(int argc, char *argv[])
{
  if (argc < 2) {
    std::cerr << "usage : " << argv[0] << " filename." << std::endl;
    return EXIT_FAILURE;
  }

  std::vector<uint32_t> contour_lengths;
  std::vector<vertex_t> vertices;
  if (LoadSegments(argv[1u], contour_lengths, vertices)) {
    std::cerr << "error while reading the data file." << std::endl;
    return EXIT_FAILURE;
  }

  PolygonTriangulation::TriangleBuffer_t triangles;
  PolygonTriangulation::Triangulate(
    contour_lengths.size(),
    contour_lengths.data(),
    vertices.data(),
    triangles
  );

  ExportData(triangles, vertices);

  return EXIT_SUCCESS;
}
