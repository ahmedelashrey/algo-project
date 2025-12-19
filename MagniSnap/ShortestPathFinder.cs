using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Runtime.CompilerServices;

namespace MagniSnap
{
    /// <summary>
    /// Fast min-heap for (nodeIndex, priority) pairs.
    /// </summary>
    internal sealed class MinHeap
    {
        private int[] _node;
        private double[] _prio;
        private int _count;

        public MinHeap(int capacity)
        {
            if (capacity < 1) capacity = 1;
            _node = new int[capacity];
            _prio = new double[capacity];
            _count = 0;
        }

        public int Count { get { return _count; } }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clear()
        {
            _count = 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Insert(int nodeIndex, double priority)
        {
            if (_count == _node.Length) Grow();

            int i = _count++;
            SiftUp(i, nodeIndex, priority);
        }

        public int ExtractMin(out double priority)
        {
            if (_count == 0) throw new InvalidOperationException("Heap is empty");

            int minNode = _node[0];
            double minPrio = _prio[0];

            _count--;
            if (_count > 0)
            {
                int lastNode = _node[_count];
                double lastPrio = _prio[_count];
                SiftDown(0, lastNode, lastPrio);
            }

            priority = minPrio;
            return minNode;
        }

        private void Grow()
        {
            int newCap = _node.Length * 2;
            Array.Resize(ref _node, newCap);
            Array.Resize(ref _prio, newCap);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void SiftUp(int i, int nodeIndex, double priority)
        {
            while (i > 0)
            {
                int parent = (i - 1) >> 1;
                if (priority >= _prio[parent]) break;

                _node[i] = _node[parent];
                _prio[i] = _prio[parent];
                i = parent;
            }

            _node[i] = nodeIndex;
            _prio[i] = priority;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void SiftDown(int i, int nodeIndex, double priority)
        {
            int half = _count >> 1; // nodes with at least one child
            while (i < half)
            {
                int left = (i << 1) + 1;
                int right = left + 1;

                int child = left;
                double childPrio = _prio[left];

                if (right < _count && _prio[right] < childPrio)
                {
                    child = right;
                    childPrio = _prio[right];
                }

                if (childPrio >= priority) break;

                _node[i] = _node[child];
                _prio[i] = childPrio;
                i = child;
            }

            _node[i] = nodeIndex;
            _prio[i] = priority;
        }
    }

    /// <summary>
    /// Windowed shortest path finder.
    /// Instead of running Dijkstra on the full image, it runs only on a fixed window (default 128x128).
    /// The window is positioned around the current cursor (then shifted to keep the anchor inside).
    /// If the target is far, it is clamped to the window edge; MultiAnchorPathFinder can "walk" in steps on click.
    /// </summary>
    public sealed class ShortestPathFinder
    {
        private const bool LOG_TIMINGS = false;

        private RGBPixel[,] _imageMatrix;
        private int _height;
        private int _width;
        private int _n; // _height * _width

        // Full-image edge weights (precomputed once per image)
        private float[] _weightRight;
        private float[] _weightDown;

        // Window params
        private int _windowSize;       // e.g., 128
        private int _halfWindow;       // windowSize/2

        // Current window bounds (inclusive)
        private int _winLeft;
        private int _winTop;
        private int _winRight;
        private int _winBottom;
        private int _winWidth;
        private int _winHeight;
        private int _winN;
        private bool _winValid;

        // Window Dijkstra buffers
        private double[] _dist;
        private int[] _parent; // parent local index, -1 if none
        private MinHeap _pq;
        private bool _dijkstraValid;

        // Anchor (global)
        private int _anchorX;
        private int _anchorY;
        private int _anchorIdxGlobal;
        private int _anchorIdxLocal;
        private bool _hasAnchor;

        public bool HasAnchor { get { return _hasAnchor; } }

        public Point AnchorPoint
        {
            get
            {
                if (!_hasAnchor) return new Point(-1, -1);
                return new Point(_anchorX, _anchorY);
            }
        }

        public ShortestPathFinder()
            : this(128)
        {
        }

        public ShortestPathFinder(int windowSize)
        {
            if (windowSize < 16) windowSize = 16;
            _windowSize = windowSize;
            _halfWindow = windowSize / 2;

            _hasAnchor = false;
            _winValid = false;
            _dijkstraValid = false;

            _pq = new MinHeap(windowSize * windowSize);
        }

        public void SetImage(RGBPixel[,] imageMatrix)
        {
            if (imageMatrix == null) throw new ArgumentNullException("imageMatrix");

            _imageMatrix = imageMatrix;
            _height = ImageToolkit.GetHeight(imageMatrix);
            _width = ImageToolkit.GetWidth(imageMatrix);
            _n = _height * _width;

            EnsureWeightBuffers(_n);
            BuildGraph(); // precompute edge weights once per image

            ClearAnchor();
        }

        public void ClearAnchor()
        {
            _hasAnchor = false;
            _anchorX = _anchorY = -1;
            _anchorIdxGlobal = -1;
            _anchorIdxLocal = -1;

            _winValid = false;
            _dijkstraValid = false;
        }

        private void EnsureWeightBuffers(int n)
        {
            if (_weightRight == null || _weightRight.Length != n) _weightRight = new float[n];
            if (_weightDown == null || _weightDown.Length != n) _weightDown = new float[n];
        }

        /// <summary>
        /// Build edge weights (graph construction).
        /// Weight = 1/(energy + epsilon).
        /// </summary>
        private void BuildGraph()
        {
            const double epsilon = 0.0001;

            Stopwatch sw = null;
            if (LOG_TIMINGS) sw = Stopwatch.StartNew();

            for (int row = 0; row < _height; row++)
            {
                int baseIdx = row * _width;
                for (int col = 0; col < _width; col++)
                {
                    int idx = baseIdx + col;
                    Vector2D energy = ImageToolkit.CalculatePixelEnergies(col, row, _imageMatrix);

                    _weightRight[idx] = (float)(1.0 / (energy.X + epsilon));
                    _weightDown[idx] = (float)(1.0 / (energy.Y + epsilon));
                }
            }

            if (LOG_TIMINGS)
            {
                sw.Stop();
                Console.WriteLine("Graph build: " + sw.ElapsedMilliseconds + " ms");
            }
        }

        public void SetAnchorPoint(int x, int y)
        {
            if (_imageMatrix == null) throw new InvalidOperationException("Image is not set.");

            if ((uint)x >= (uint)_width || (uint)y >= (uint)_height)
                throw new ArgumentOutOfRangeException("Anchor point out of bounds.");

            _anchorX = x;
            _anchorY = y;
            _anchorIdxGlobal = y * _width + x;
            _hasAnchor = true;

            // Force recompute next time (window depends on cursor / target)
            _winValid = false;
            _dijkstraValid = false;
        }

        /// <summary>
        /// Ensures the window and Dijkstra tree are ready for a given target point.
        /// Returns an "effective" target inside the current window (may be clamped to the window edge).
        /// </summary>
        public Point PrepareForTarget(int targetX, int targetY)
        {
            if (!_hasAnchor || _imageMatrix == null)
                return new Point(targetX, targetY);

            // Build / move window if needed
            if (!_winValid || !IsInsideWindow(targetX, targetY) || !IsInsideWindow(_anchorX, _anchorY))
            {
                int newLeft, newTop;
                ComputeWindowTopLeft(targetX, targetY, out newLeft, out newTop);

                bool windowChanged = (!_winValid) || (newLeft != _winLeft) || (newTop != _winTop);

                if (windowChanged)
                {
                    SetWindow(newLeft, newTop);
                    _dijkstraValid = false;
                }
            }

            // Clamp target into window
            int effX = targetX;
            int effY = targetY;

            if (effX < _winLeft) effX = _winLeft;
            if (effX > _winRight) effX = _winRight;
            if (effY < _winTop) effY = _winTop;
            if (effY > _winBottom) effY = _winBottom;

            // Ensure anchor is inside (should be by construction)
            if (!IsInsideWindow(_anchorX, _anchorY))
            {
                // As a fallback, re-center around anchor.
                int newLeft2, newTop2;
                ComputeWindowTopLeft(_anchorX, _anchorY, out newLeft2, out newTop2);
                SetWindow(newLeft2, newTop2);
                _dijkstraValid = false;

                if (effX < _winLeft) effX = _winLeft;
                if (effX > _winRight) effX = _winRight;
                if (effY < _winTop) effY = _winTop;
                if (effY > _winBottom) effY = _winBottom;
            }

            // Compute Dijkstra for this window if needed
            if (!_dijkstraValid)
            {
                RunDijkstraWindow();
                _dijkstraValid = true;
            }

            return new Point(effX, effY);
        }

        private bool IsInsideWindow(int x, int y)
        {
            if (!_winValid) return false;
            return x >= _winLeft && x <= _winRight && y >= _winTop && y <= _winBottom;
        }

        private void ComputeWindowTopLeft(int cursorX, int cursorY, out int left, out int top)
        {
            // Start centered around cursor
            left = cursorX - _halfWindow;
            top = cursorY - _halfWindow;

            // Clamp to image so window fits
            int maxLeft = _width - _windowSize;
            int maxTop = _height - _windowSize;
            if (maxLeft < 0) maxLeft = 0;
            if (maxTop < 0) maxTop = 0;

            if (left < 0) left = 0;
            if (top < 0) top = 0;
            if (left > maxLeft) left = maxLeft;
            if (top > maxTop) top = maxTop;

            // Shift to keep anchor inside (if possible)
            // If the cursor is too far, the cursor will be clamped later.
            if (_hasAnchor)
            {
                if (_anchorX < left) left = _anchorX;
                if (_anchorX > left + _windowSize - 1) left = _anchorX - _windowSize + 1;

                if (_anchorY < top) top = _anchorY;
                if (_anchorY > top + _windowSize - 1) top = _anchorY - _windowSize + 1;

                if (left < 0) left = 0;
                if (top < 0) top = 0;
                if (left > maxLeft) left = maxLeft;
                if (top > maxTop) top = maxTop;
            }
        }

        private void SetWindow(int left, int top)
        {
            _winLeft = left;
            _winTop = top;

            _winRight = _winLeft + _windowSize - 1;
            _winBottom = _winTop + _windowSize - 1;

            if (_winRight >= _width) _winRight = _width - 1;
            if (_winBottom >= _height) _winBottom = _height - 1;

            _winWidth = _winRight - _winLeft + 1;
            _winHeight = _winBottom - _winTop + 1;
            _winN = _winWidth * _winHeight;

            EnsureWindowBuffers(_winN);

            // Update anchor local index
            _anchorIdxLocal = GlobalToLocal(_anchorX, _anchorY);

            _winValid = true;
        }

        private void EnsureWindowBuffers(int winN)
        {
            if (_dist == null || _dist.Length != winN) _dist = new double[winN];
            if (_parent == null || _parent.Length != winN) _parent = new int[winN];

            // Ensure heap can handle winN items (it can grow automatically)
            if (_pq == null) _pq = new MinHeap(Math.Max(1024, winN));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private int GlobalToLocal(int x, int y)
        {
            return (y - _winTop) * _winWidth + (x - _winLeft);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void LocalToGlobal(int localIdx, out int x, out int y)
        {
            int row = localIdx / _winWidth;
            int col = localIdx - row * _winWidth;
            x = _winLeft + col;
            y = _winTop + row;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private int LocalToGlobalIdx(int localIdx)
        {
            int row = localIdx / _winWidth;
            int col = localIdx - row * _winWidth;
            int gy = _winTop + row;
            int gx = _winLeft + col;
            return gy * _width + gx;
        }

        /// <summary>
        /// Dijkstra within the current window only.
        /// Source = anchor (must be inside window).
        /// </summary>
        private void RunDijkstraWindow()
        {
            if (!_winValid) return;

            Stopwatch sw = null;
            if (LOG_TIMINGS) sw = Stopwatch.StartNew();

            for (int i = 0; i < _winN; i++)
            {
                _dist[i] = double.PositiveInfinity;
                _parent[i] = -1;
            }

            _pq.Clear();

            // Anchor must be inside window; if not, just bail
            if (_anchorIdxLocal < 0 || _anchorIdxLocal >= _winN)
                return;

            _dist[_anchorIdxLocal] = 0.0;
            _pq.Insert(_anchorIdxLocal, 0.0);

            while (_pq.Count > 0)
            {
                double du;
                int uLocal = _pq.ExtractMin(out du);

                if (du > _dist[uLocal])
                    continue;

                int uRow = uLocal / _winWidth;
                int uCol = uLocal - uRow * _winWidth;

                int uGlobalIdx = LocalToGlobalIdx(uLocal);

                // Right
                if (uCol + 1 < _winWidth)
                {
                    int vLocal = uLocal + 1;
                    double nd = du + _weightRight[uGlobalIdx];
                    if (nd < _dist[vLocal])
                    {
                        _dist[vLocal] = nd;
                        _parent[vLocal] = uLocal;
                        _pq.Insert(vLocal, nd);
                    }
                }

                // Left
                if (uCol > 0)
                {
                    int vLocal = uLocal - 1;
                    int vGlobalIdx = uGlobalIdx - 1;
                    double nd = du + _weightRight[vGlobalIdx];
                    if (nd < _dist[vLocal])
                    {
                        _dist[vLocal] = nd;
                        _parent[vLocal] = uLocal;
                        _pq.Insert(vLocal, nd);
                    }
                }

                // Down
                if (uRow + 1 < _winHeight)
                {
                    int vLocal = uLocal + _winWidth;
                    double nd = du + _weightDown[uGlobalIdx];
                    if (nd < _dist[vLocal])
                    {
                        _dist[vLocal] = nd;
                        _parent[vLocal] = uLocal;
                        _pq.Insert(vLocal, nd);
                    }
                }

                // Up
                if (uRow > 0)
                {
                    int vLocal = uLocal - _winWidth;
                    int vGlobalIdx = uGlobalIdx - _width;
                    double nd = du + _weightDown[vGlobalIdx];
                    if (nd < _dist[vLocal])
                    {
                        _dist[vLocal] = nd;
                        _parent[vLocal] = uLocal;
                        _pq.Insert(vLocal, nd);
                    }
                }
            }

            if (LOG_TIMINGS)
            {
                sw.Stop();
                Console.WriteLine("Window Dijkstra (" + _winWidth + "x" + _winHeight + "): " + sw.ElapsedMilliseconds + " ms");
            }
        }

        /// <summary>
        /// Backtrack path from effective target to anchor (within current window).
        /// Returns points from target -> anchor (same behavior as before).
        /// </summary>
        public List<Point> GetPathToAnchor(int targetX, int targetY, out Point effectiveTarget)
        {
            effectiveTarget = PrepareForTarget(targetX, targetY);

            List<Point> path = new List<Point>(256);

            if (!_hasAnchor || !_winValid)
                return path;

            int effX = effectiveTarget.X;
            int effY = effectiveTarget.Y;

            if (!IsInsideWindow(effX, effY))
                return path;

            int curLocal = GlobalToLocal(effX, effY);
            int anchorLocal = _anchorIdxLocal;

            // If anchor local invalid, return empty
            if (anchorLocal < 0 || anchorLocal >= _winN)
                return path;

            while (curLocal != -1)
            {
                int gx, gy;
                LocalToGlobal(curLocal, out gx, out gy);
                path.Add(new Point(gx, gy));

                if (curLocal == anchorLocal)
                    break;

                curLocal = _parent[curLocal];
            }

            return path;
        }

        /// <summary>
        /// Fast drawing: draws the path from the (possibly clamped) target to anchor without allocating lists.
        /// </summary>
        public void DrawPath(Graphics g, int targetX, int targetY, Color pathColor, int thickness)
        {
            if (!_hasAnchor || _imageMatrix == null)
                return;

            Point eff = PrepareForTarget(targetX, targetY);

            if (!_winValid)
                return;

            int effX = eff.X;
            int effY = eff.Y;

            if (!IsInsideWindow(effX, effY))
                return;

            int curLocal = GlobalToLocal(effX, effY);
            int anchorLocal = _anchorIdxLocal;

            int nextLocal = _parent[curLocal];
            if (nextLocal == -1 || curLocal == anchorLocal)
                return;

            using (Pen pen = new Pen(pathColor, thickness))
            {
                while (curLocal != -1 && nextLocal != -1)
                {
                    int x1, y1, x2, y2;
                    LocalToGlobal(curLocal, out x1, out y1);
                    LocalToGlobal(nextLocal, out x2, out y2);

                    g.DrawLine(pen, x1, y1, x2, y2);

                    if (nextLocal == anchorLocal)
                        break;

                    curLocal = nextLocal;
                    nextLocal = _parent[curLocal];
                }
            }
        }
    }

    /// <summary>
    /// Manages multiple anchor points for creating closed selections.
    /// Uses window stepping on clicks when the click is far (repeated 128x128 window solves).
    /// </summary>
    public sealed class MultiAnchorPathFinder
    {
        private RGBPixel[,] _imageMatrix;
        private readonly ShortestPathFinder _pathFinder;

        private readonly List<Point> _anchorPoints;      // explicit anchors (clicks)
        private readonly List<Point[]> _confirmedPaths;  // segments (each in forward order)

        private bool _isClosed;

        public MultiAnchorPathFinder()
        {
            _pathFinder = new ShortestPathFinder(128); // change this if you want a different window size
            _anchorPoints = new List<Point>();
            _confirmedPaths = new List<Point[]>();
            _isClosed = false;
        }

        public void SetImage(RGBPixel[,] imageMatrix)
        {
            _imageMatrix = imageMatrix;
            _pathFinder.SetImage(imageMatrix);
            Clear();
        }

        public void AddAnchorPoint(int x, int y)
        {
            if (_imageMatrix == null)
                return;

            if (_isClosed)
                return;

            // First anchor
            if (_anchorPoints.Count == 0)
            {
                _anchorPoints.Add(new Point(x, y));
                _pathFinder.SetAnchorPoint(x, y);
                return;
            }

            // Walk in window-sized steps until we reach (x,y)
            CommitPathInSteps(x, y);

            // Register explicit anchor at the actual click location
            _anchorPoints.Add(new Point(x, y));
            _pathFinder.SetAnchorPoint(x, y);
        }

        /// <summary>
        /// Prepare for drawing (returns the effective point actually used for live drawing; may be clamped).
        /// </summary>
        public Point UpdateCursor(int x, int y)
        {
            if (!HasAnchors || _isClosed)
                return new Point(x, y);

            return _pathFinder.PrepareForTarget(x, y);
        }

        private void CommitPathInSteps(int targetX, int targetY)
        {
            // Safety: avoid infinite loops if something goes wrong
            const int MAX_STEPS = 2048;
            int steps = 0;

            while (steps++ < MAX_STEPS)
            {
                Point effective;
                List<Point> freeToAnchor = _pathFinder.GetPathToAnchor(targetX, targetY, out effective);

                if (freeToAnchor == null || freeToAnchor.Count < 2)
                    break;

                // Convert to forward segment: anchor -> effective
                freeToAnchor.Reverse();

                // Avoid adding degenerate segments
                if (freeToAnchor.Count >= 2)
                    _confirmedPaths.Add(freeToAnchor.ToArray());

                // If we reached the real target, done
                if (effective.X == targetX && effective.Y == targetY)
                    break;

                // Otherwise, we only reached a clamped "edge" point: make it the new implicit anchor and continue
                _pathFinder.SetAnchorPoint(effective.X, effective.Y);
            }
        }

        public void CloseSelection()
        {
            if (_anchorPoints.Count < 2 || _isClosed)
                return;

            Point first = _anchorPoints[0];

            // Connect current anchor to the first anchor using window steps
            CommitPathInSteps(first.X, first.Y);

            _isClosed = true;
        }

        public void Draw(Graphics g, int freeX, int freeY, Color pathColor, Color anchorColor)
        {
            // Draw confirmed paths
            using (Pen pen = new Pen(pathColor, 2))
            {
                for (int i = 0; i < _confirmedPaths.Count; i++)
                {
                    Point[] pts = _confirmedPaths[i];
                    if (pts != null && pts.Length >= 2)
                        g.DrawLines(pen, pts);
                }
            }

            // Draw live path (windowed; may clamp the cursor internally)
            _pathFinder.DrawPath(g, freeX, freeY, pathColor, 2);

            // Draw explicit anchor points
            using (SolidBrush brush = new SolidBrush(anchorColor))
            {
                for (int i = 0; i < _anchorPoints.Count; i++)
                {
                    Point a = _anchorPoints[i];
                    g.FillEllipse(brush, a.X - 4, a.Y - 4, 8, 8);
                }
            }
        }

        public void Clear()
        {
            _anchorPoints.Clear();
            _confirmedPaths.Clear();
            _isClosed = false;

            _pathFinder.ClearAnchor();
        }

        public int AnchorCount { get { return _anchorPoints.Count; } }
        public bool HasAnchors { get { return _anchorPoints.Count > 0; } }

        public bool IsClosed { get { return _isClosed; } }

        public Point[] GetClosedPolygon()
        {
            if (!IsClosed)
                return null;

            List<Point> poly = new List<Point>(1024);

            for (int i = 0; i < _confirmedPaths.Count; i++)
            {
                Point[] segment = _confirmedPaths[i];
                if (segment == null || segment.Length == 0)
                    continue;

                if (poly.Count > 0 && poly[poly.Count - 1] == segment[0])
                {
                    // Skip the first point to avoid duplication
                    for (int j = 1; j < segment.Length; j++)
                        poly.Add(segment[j]);
                }
                else
                {
                    for (int j = 0; j < segment.Length; j++)
                        poly.Add(segment[j]);
                }
            }

            return poly.Count >= 3 ? poly.ToArray() : null;
        }
    }
}
