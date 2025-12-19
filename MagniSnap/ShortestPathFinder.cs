using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;

namespace MagniSnap
{
    /// <summary>
    /// Fast min-heap for (nodeIndex, priority) pairs.
    /// No structs/IComparable, no swaps: sift-up/down with "hole" technique.
    /// </summary>
    internal sealed class MinHeap
    {
        private int[] _node;
        private double[] _prio;
        private int _count;

        public MinHeap(int capacity = 1024)
        {
            if (capacity < 1) capacity = 1;
            _node = new int[capacity];
            _prio = new double[capacity];
            _count = 0;
        }

        public int Count => _count;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clear() => _count = 0;

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
    /// Finds shortest path using Dijkstra on a 4-connected pixel grid.
    /// Major optimizations:
    /// - Flattened 2D arrays into 1D for cache efficiency
    /// - Removed GetEdgeWeight() swapping/branching from inner loop
    /// - Reuse arrays between runs to avoid large allocations/GC pressure
    /// - Fast heap + stale-entry skip (no visited[,] needed)
    /// - DrawPath() backtracks and draws without allocating a List every mouse-move
    /// </summary>
    public sealed class ShortestPathFinder
    {
        private RGBPixel[,] _imageMatrix;
        private int _height;
        private int _width;
        private int _n; // _height * _width

        // Edge weights stored per pixel (flattened):
        // weightRight[idx] = weight from (r,c) to (r,c+1)
        // weightDown[idx]  = weight from (r,c) to (r+1,c)
        private float[] _weightRight;
        private float[] _weightDown;

        // Dijkstra results (flattened)
        private double[] _distance;
        private int[] _parentIdx; // previous node index, -1 if none

        private int _anchorIdx = -1;
        private bool _hasAnchor;

        public bool HasAnchor => _hasAnchor;
        public Point AnchorPoint => _hasAnchor ? IdxToPoint(_anchorIdx) : new Point(-1, -1);

        public ShortestPathFinder()
        {
            _hasAnchor = false;
        }

        public void SetImage(RGBPixel[,] imageMatrix)
        {
            _imageMatrix = imageMatrix ?? throw new ArgumentNullException(nameof(imageMatrix));
            _height = ImageToolkit.GetHeight(imageMatrix);
            _width = ImageToolkit.GetWidth(imageMatrix);
            _n = _height * _width;

            EnsureWorkingBuffers(_n);
            BuildGraph(); // precompute edge weights once per image

            _hasAnchor = false;
            _anchorIdx = -1;
        }

        private void EnsureWorkingBuffers(int n)
        {
            if (_weightRight == null || _weightRight.Length != n) _weightRight = new float[n];
            if (_weightDown == null || _weightDown.Length != n) _weightDown = new float[n];

            if (_distance == null || _distance.Length != n) _distance = new double[n];
            if (_parentIdx == null || _parentIdx.Length != n) _parentIdx = new int[n];
        }

        /// <summary>
        /// Build edge weights (graph construction).
        /// Weight = 1/(energy + epsilon).
        /// </summary>
        private void BuildGraph()
        {
            const double epsilon = 0.0001;

            Stopwatch sw = Stopwatch.StartNew();

         
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
            

            sw.Stop();

            Console.WriteLine($"Graph build: {sw.ElapsedMilliseconds} ms");
        }

        public void SetAnchorPoint(int x, int y)
        {
            if (_imageMatrix == null) throw new InvalidOperationException("Image is not set.");

            if ((uint)x >= (uint)_width || (uint)y >= (uint)_height)
                throw new ArgumentOutOfRangeException("Anchor point out of bounds.");

            _anchorIdx = y * _width + x;
            _hasAnchor = true;

            RunDijkstra(_anchorIdx);
        }

        /// <summary>
        /// Dijkstra from a single source over the whole grid.
        /// Uses stale-entry skipping (distFromHeap > distance[u]) instead of visited[].
        /// </summary>
        private void RunDijkstra(int srcIdx)
        {
            Stopwatch sw = Stopwatch.StartNew();

            // Reset arrays (still O(n), but avoids reallocations/GC)
            for (int i = 0; i < _distance.Length; i++) _distance[i] = double.PositiveInfinity;
            for (int i = 0; i < _parentIdx.Length; i++) _parentIdx[i] = -1;

            _distance[srcIdx] = 0.0;

            MinHeap pq = new MinHeap(capacity: Math.Min(_n, 1 << 20));
            pq.Insert(srcIdx, 0.0);

            while (pq.Count > 0)
            {
                int u = pq.ExtractMin(out double du);

                // Stale entry (we already found a better distance)
                if (du > _distance[u])
                    continue;

                int row = Math.DivRem(u, _width, out int col);

                // Right neighbor: (row, col+1)
                if (col + 1 < _width)
                {
                    int v = u + 1;
                    double nd = du + _weightRight[u];
                    if (nd < _distance[v])
                    {
                        _distance[v] = nd;
                        _parentIdx[v] = u;
                        pq.Insert(v, nd);
                    }
                }

                // Left neighbor: (row, col-1) uses weightRight[u-1]
                if (col > 0)
                {
                    int v = u - 1;
                    double nd = du + _weightRight[v]; // v == u-1
                    if (nd < _distance[v])
                    {
                        _distance[v] = nd;
                        _parentIdx[v] = u;
                        pq.Insert(v, nd);
                    }
                }

                // Down neighbor: (row+1, col)
                if (row + 1 < _height)
                {
                    int v = u + _width;
                    double nd = du + _weightDown[u];
                    if (nd < _distance[v])
                    {
                        _distance[v] = nd;
                        _parentIdx[v] = u;
                        pq.Insert(v, nd);
                    }
                }

                // Up neighbor: (row-1, col) uses weightDown[u-width]
                if (row > 0)
                {
                    int v = u - _width;
                    double nd = du + _weightDown[v]; // v == u-width
                    if (nd < _distance[v])
                    {
                        _distance[v] = nd;
                        _parentIdx[v] = u;
                        pq.Insert(v, nd);
                    }
                }
            }

            sw.Stop();


            Console.WriteLine($"Dijkstra: {sw.ElapsedMilliseconds} ms");
        }

        /// <summary>
        /// Backtrack shortest path from free point to anchor point.
        /// Returns points from free -> anchor (same as your original behavior).
        /// </summary>
        public List<Point> GetPathToAnchor(int freeX, int freeY)
        {
            List<Point> path = new List<Point>(256);

            if (!_hasAnchor || _imageMatrix == null)
                return path;

            if ((uint)freeX >= (uint)_width || (uint)freeY >= (uint)_height)
                return path;

            int cur = freeY * _width + freeX;
            int anchor = _anchorIdx;

            while (cur != -1)
            {
                path.Add(IdxToPoint(cur));
                if (cur == anchor)
                    break;

                cur = _parentIdx[cur];
            }

            return path;
        }

        /// <summary>
        /// Fast drawing: backtracks and draws segments without allocating a List (best for live mouse-move).
        /// </summary>
        public void DrawPath(Graphics g, int freeX, int freeY, Color pathColor, int thickness = 2)
        {
            if (!_hasAnchor || _imageMatrix == null)
                return;

            if ((uint)freeX >= (uint)_width || (uint)freeY >= (uint)_height)
                return;

            int cur = freeY * _width + freeX;
            int anchor = _anchorIdx;

            // Need at least one segment
            int next = _parentIdx[cur];
            if (next == -1 || cur == anchor)
                return;

            using (Pen pen = new Pen(pathColor, thickness))
            {
                while (cur != -1 && next != -1)
                {
                    Point p1 = IdxToPoint(cur);
                    Point p2 = IdxToPoint(next);
                    g.DrawLine(pen, p1, p2);

                    if (next == anchor)
                        break;

                    cur = next;
                    next = _parentIdx[cur];
                }
            }
        }

        public void DrawAnchorPoint(Graphics g, Color color, int size = 6)
        {
            if (!_hasAnchor)
                return;

            Point a = IdxToPoint(_anchorIdx);
            using (SolidBrush brush = new SolidBrush(color))
            {
                g.FillEllipse(brush, a.X - size / 2, a.Y - size / 2, size, size);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private Point IdxToPoint(int idx)
        {
            int y = Math.DivRem(idx, _width, out int x);
            return new Point(x, y);
        }
    }

    /// <summary>
    /// Manages multiple anchor points for creating closed selections.
    /// Optimizations:
    /// - Store confirmed paths as Point[] and draw with Graphics.DrawLines (1 call per path).
    /// - Live path drawing uses ShortestPathFinder.DrawPath() (no allocations per frame).
    /// </summary>
    public sealed class MultiAnchorPathFinder
    {
        private RGBPixel[,] _imageMatrix;
        private readonly ShortestPathFinder _pathFinder;
        private readonly List<Point> _anchorPoints;
        private readonly List<Point[]> _confirmedPaths;

        public MultiAnchorPathFinder()
        {
            _pathFinder = new ShortestPathFinder();
            _anchorPoints = new List<Point>();
            _confirmedPaths = new List<Point[]>();
        }

        public void SetImage(RGBPixel[,] imageMatrix)
        {
            _imageMatrix = imageMatrix;
            _pathFinder.SetImage(imageMatrix);
            Clear();
        }

        public void AddAnchorPoint(int x, int y)
        {
            // If there's a previous anchor, save the path to this new point
            if (_anchorPoints.Count > 0)
            {
                List<Point> path = _pathFinder.GetPathToAnchor(x, y);
                if (path.Count >= 2)
                    _confirmedPaths.Add(path.ToArray());
            }

            _anchorPoints.Add(new Point(x, y));
            _pathFinder.SetAnchorPoint(x, y);
        }

        /// <summary>
        /// If you still need the points (allocates). Prefer Draw() for live usage.
        /// </summary>
        public List<Point> GetLivePath(int freeX, int freeY)
        {
            return _pathFinder.GetPathToAnchor(freeX, freeY);
        }

        public void CloseSelection()
        {
            if (_anchorPoints.Count < 2)
                return;

            Point first = _anchorPoints[0];
            List<Point> closingPath = _pathFinder.GetPathToAnchor(first.X, first.Y);
            if (closingPath.Count >= 2)
                _confirmedPaths.Add(closingPath.ToArray());
        }

        public void Draw(Graphics g, int freeX, int freeY, Color pathColor, Color anchorColor)
        {
            // Draw confirmed paths (single GDI+ call per path)
            using (Pen pen = new Pen(pathColor, 2))
            {
                for (int i = 0; i < _confirmedPaths.Count; i++)
                {
                    Point[] pts = _confirmedPaths[i];
                    if (pts != null && pts.Length >= 2)
                        g.DrawLines(pen, pts);
                }
            }

            // Draw live path WITHOUT allocations
            _pathFinder.DrawPath(g, freeX, freeY, pathColor, 2);

            // Draw anchor points
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
        }

        public int AnchorCount => _anchorPoints.Count;
        public bool HasAnchors => _anchorPoints.Count > 0;
    }
}
