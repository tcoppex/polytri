'use strict';

function TPoint(x, y) {
  this.x = x;
  this.y = y;

  this.equals = function(o) {
    return (o instanceof TPoint)
        && (this.x == o.x)
        && (this.y == o.y);
  }

  // return the dot product of instance with parameter
  this.dot = function(p) {
    return this.x * p.x + this.y * p.y;
  }

  this.add = function(p) {
    this.x += p.x;
    this.y += p.y;
    return this;
  }

  this.mult = function(s) {
    this.x *= s;
    this.y *= s;
    return this;
  }

  // return the point relative to another (its vector)
  this.relativeTo = function(o) {
    return new TPoint(this.x - o.x, this.y - o.y);
  }

  this.asArray = function() {
    return [this.x, this.y];
  }

  // return the squared distance to a point
  this.squaredDistanceTo = function(p) {
    var v = this.relativeTo(p);
    return v.dot(v);
  }

  this.getNormal = function() {
    var n = new TPoint(-this.y, this.x);
    var dp = n.dot(n);
    if (dp > 0.0) {
      var invLen = Math.sqrt(1.0/dp);
      n.x *= invLen;
      n.y *= invLen;
    }
    return n;
  }
}

/* -------------------------------------------------------------------------- */

var CanvasView = {
  init: function(pointRadius) {
    this.canvas = document.getElementById('canvas-app');
    this.ctx = this.canvas.getContext('2d');

    this.lineColor = '#457590';
    this.pointColor = '#953030';
    this.pointRadius = (pointRadius == undefined) ? 3 : pointRadius;

    this.ctx.fillStyle = this.pointColor;
    this.ctx.strokeStyle = this.lineColor;
    this.ctx.lineWidth = 1;
  },

  clear: function() {
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
  },

  drawLine: function(a, b) {
    this.ctx.beginPath();
    this.ctx.moveTo(a.x, a.y);
    this.ctx.lineTo(b.x, b.y);
    this.ctx.stroke();
  },

  drawPoint: function(p) {
    this.ctx.beginPath();
    this.ctx.arc(p.x, p.y, this.pointRadius, 0.0, 2.0*Math.PI);
    this.ctx.fill();
  },

  drawPoints: function(points) {
    points.map(this.drawPoint, this);
  },

  setPointColor: function(color) {
    this.ctx.fillStyle = color;
  },

  setLineWidth: function(width) {
    this.ctx.lineWidth = width;
  },
};
