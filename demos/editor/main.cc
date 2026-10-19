// ----------------------------------------------------------------------------
//
//  polytri-editor
//
//    Draw an outer contour, then holes, automatically retriangulate.
//    The first contour is the outer boundary, the following ones are holes.
//
//   left click  : add a point (first click starts a new contour) / drag a vertex
//   Enter, or click on the first point : close the contour being drawn
//   right click : remove the last point being drawn, or the vertex under the cursor
//   Esc         : cancel the contour being drawn
//   R           : back to the default shape
//
// ----------------------------------------------------------------------------

#include <cmath>
#include <cstdio>
#include <cstring>

#include <chrono>
#include <string>
#include <vector>

#include <raylib.h>

#include "polytri/polytri.hpp"

// ----------------------------------------------------------------------------

struct Editor {
  using Contour = std::vector<Vector2>;

  static constexpr float kPick = 9.0f;

  std::vector<Contour> contours{};
  Contour pending{};
  std::vector<unsigned> indices{};

  polytri::Context polytri{};

  double micros = 0.0;
  int drag_c = -1, drag_v = -1;
  std::string error{};


  void reset() {
    contours.clear();
    pending.clear();
  }

  void retriangulate() {
    indices.clear();
    error.clear();

    if (contours.empty()) {
      return;
    }

    const auto t0 = std::chrono::steady_clock::now();
    const auto r = polytri.triangulate(contours, indices);

    micros = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
    if (!r) {
      indices.clear();
      error = r.message();
      if (r.contour != polytri::Result::kNoContour) {
        error += " (contour " + std::to_string(r.contour) + ")";
      }
    }
  }

  bool pick(Vector2 m, int &c, int &v) const {
    for (int i = 0; i < static_cast<int>(contours.size()); ++i)
      for (int j = 0; j < static_cast<int>(contours[i].size()); ++j) {
        const float dx = contours[i][j].x - m.x, dy = contours[i][j].y - m.y;
        if (dx * dx + dy * dy <= kPick * kPick) {
          c = i;
          v = j;
          return true;
        }
      }
    return false;
  }

  void close_pending() {
    if (pending.size() < 3) {
      return;
    }
    contours.push_back(pending);
    pending.clear();
    retriangulate();
  }

  void update() {
    const Vector2 m = GetMousePosition();

    if (IsKeyPressed(KEY_R)) {
      reset();
      return;
    }

    if (IsKeyPressed(KEY_ESCAPE)) {
      if (!pending.empty()) {
        pending.clear();
      }
    }

    if (IsKeyPressed(KEY_ENTER)) {
      close_pending();
    }

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
      int c, v;
      if (!pending.empty()) {
        const float dx = pending[0].x - m.x, dy = pending[0].y - m.y;
        if (pending.size() >= 3 && dx * dx + dy * dy <= kPick * kPick) {
          close_pending();
        } else {
          pending.push_back(m);
        }
      } else if (pick(m, c, v)) {
        drag_c = c; drag_v = v;
      } else {
        pending.push_back(m);
      }
    }

    if (drag_c >= 0) {
      if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        contours[drag_c][drag_v] = m;
        retriangulate();
      } else {
        drag_c = drag_v = -1;
      }
    }

    if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
      int c, v;
      if (!pending.empty()) {
        pending.pop_back();
      } else if (pick(m, c, v)) {
        contours[c].erase(contours[c].begin() + v);
        if (contours[c].size() < 3) {
          contours.erase(contours.begin() + c);
        }
        retriangulate();
      }
    }
  }

  void draw() const {
    ClearBackground({24, 26, 32, 255});

    for (size_t i = 0; i + 2 < indices.size(); i += 3) {
      const Vector2 a = contours_at(indices[i]);
      const Vector2 b = contours_at(indices[i + 1]);
      const Vector2 c = contours_at(indices[i + 2]);

      DrawTriangle(a, c, b, {70, 120, 190, 110});
      DrawTriangleLines(a, b, c, {150, 190, 240, 200});
    }

    for (size_t i = 0; i < contours.size(); ++i) {
      const Color col = !error.empty()
                      ? Color{230, 80, 80, 255}
                      : (i == 0) ? Color{245, 245, 245, 255}
                                 : Color{255, 170, 70, 255};
      const auto &c = contours[i];
      for (size_t j = 0; j < c.size(); ++j) {
        DrawLineEx(c[j], c[(j + 1) % c.size()], 2.5f, col);
      }
      for (const auto &p : c) {
        DrawCircleV(p, 4.5f, col);
      }
    }

    for (size_t j = 0; j < pending.size(); ++j) {
      if (j + 1 < pending.size()) {
        DrawLineEx(pending[j], pending[j + 1], 2.0f, GREEN);
      }
      DrawCircleV(pending[j], 4.5f, GREEN);
    }
    if (!pending.empty()) {
      DrawLineEx(pending.back(), GetMousePosition(), 1.0f, {120, 200, 120, 160});
    }

    const char *hint = pending.empty()
      ? "click: new contour / drag vertex   right click: delete vertex   R: reset"
      : (contours.empty() ? "drawing the OUTER contour - Enter or click first point to close"
                          : "drawing a HOLE - Enter or click first point to close, Esc to cancel");
    DrawText(hint, 12, 10, 18, LIGHTGRAY);

    if (!error.empty()) {
      DrawText(("invalid : " + error).c_str(), 12, 36, 20, {240, 100, 100, 255});
    } else {
      DrawText(TextFormat("%d triangles - %.0f us", static_cast<int>(indices.size() / 3), micros), 12, 36, 18, GRAY);
    }
  }

  Vector2 contours_at(unsigned index) const {
    for (const auto &c : contours) {
      if (index < c.size()) return c[index];
      index -= static_cast<unsigned>(c.size());
    }
    return {};
  }
};

// ----------------------------------------------------------------------------

int main(int argc, char **argv) {
  Editor editor{
    .contours = {
      {{150, 120}, {520, 80}, {850, 130}, {900, 400}, {820, 620}, {500, 560}, {180, 620}, {110, 380}},
      {{300, 260}, {420, 220}, {440, 340}, {330, 380}},
      {{600, 250}, {760, 280}, {680, 420}},
    }
  };
  editor.retriangulate();

  SetTraceLogLevel(LOG_WARNING);
  InitWindow(1000, 700, "polytri editor");
  SetTargetFPS(60);

  while (!WindowShouldClose()) {
    editor.update();
    BeginDrawing();
    editor.draw();
    EndDrawing();
  }
  CloseWindow();

  return 0;
}

// ----------------------------------------------------------------------------
