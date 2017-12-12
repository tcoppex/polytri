
var show_contour = true;

/* -------------------------------------------------------------------------- */

var Data = {
  init: function() {
    this.points = vertices.map(pt => new TPoint(pt.x, CanvasView.canvas.height - pt.y));
  },

  // clear the point dataset
  clearPoints: function() {
    this.points = [];
  }
};

/* -------------------------------------------------------------------------- */

var App = {
  // Initialize the applicaton.
  init: function() {
    CanvasView.init();
    CanvasView.canvas.addEventListener('mousedown', function(event) {
      var x = event.pageX - CanvasView.canvas.offsetLeft,
          y = event.pageY - CanvasView.canvas.offsetTop;

      Data.points.push(new TPoint(x, y));
    }, false);

    CanvasView.setPointColor(CanvasView.pointColor);

    Data.init();

    App.update();
  },

  // Reset data pointset and rendering.
  reset: function() {
    // Generate new datapoints.
    Data.clearPoints();
    TRI = [];
  },

  render: function() {
    CanvasView.clear();

    if (Data.points.length <= 0) {
      return;
    }

    // render triangles
    TRI.forEach((e) => {
      var A = Data.points[e[0]];
      var B = Data.points[e[1]];
      var C = Data.points[e[2]];
      CanvasView.drawLine(A, B);
      CanvasView.drawLine(B, C);
      CanvasView.drawLine(C, A);
    });

    if (show_contour) {
      // draw lines
      var vertices = [Data.points[Data.points.length-1]]
      vertices.push.apply(vertices, Data.points);
      
      vertices.reduce((a, b) => {
        var c = CanvasView.lineColor;
        CanvasView.ctx.strokeStyle = '#C33';
        CanvasView.drawLine(a, b);
        CanvasView.ctx.strokeStyle = c;
        return b;
      });
    }

    CanvasView.setPointColor('#00ff00');
    CanvasView.drawPoints(Data.points);
  },
  
  export: function() {
    var s = "";
    s += Data.points.length + "\n";
    for (var i in Data.points) {
      var x = Data.points[i].x;// / CanvasView.canvas.width;
      var y = CanvasView.canvas.height - Data.points[i].y;// / CanvasView.canvas.height;
      s += x + " " + y + "\n";
    }

    console.log(s);
  },

  contour: function() {
    show_contour = !show_contour;
  },

  update: function() {
    App.render();
    window.requestAnimationFrame(App.update);
  }
};

/* -------------------------------------------------------------------------- */
