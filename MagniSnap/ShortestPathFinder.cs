using System;
using System.Collections.Generic;
using System.Drawing;

namespace MagniSnap
{
    /// <summary>
    /// Node in the priority queue for Dijkstra's algorithm
    /// </summary>
    public struct PQNode : IComparable<PQNode>
    {
        public int Row;
        public int Col;
        public double Distance;

        public PQNode(int row, int col, double distance)
        {
            Row = row;
            Col = col;
            Distance = distance;
        }

        public int CompareTo(PQNode other)
        {
            return Distance.CompareTo(other.Distance);
        }
    }

    /// <summary>
    /// Min-Heap priority queue for Dijkstra's algorithm
    /// Time: O(log n) for insert and extract-min
    /// </summary>
    public class MinHeap
    {
        private List<PQNode> heap;

        public MinHeap()
        {
            heap = new List<PQNode>();
        }

        public int Count => heap.Count;

        public void Insert(PQNode node)
        {
            heap.Add(node);
            HeapifyUp(heap.Count - 1);
        }

        public PQNode ExtractMin()
        {
            if (heap.Count == 0)
                throw new InvalidOperationException("Heap is empty");

            PQNode min = heap[0];
            heap[0] = heap[heap.Count - 1];
            heap.RemoveAt(heap.Count - 1);

            if (heap.Count > 0)
                HeapifyDown(0);

            return min;
        }

        private void HeapifyUp(int index)
        {
            while (index > 0)
            {
                int parent = (index - 1) / 2;
                if (heap[index].CompareTo(heap[parent]) >= 0)
                    break;

                Swap(index, parent);
                index = parent;
            }
        }

        private void HeapifyDown(int index)
        {
            while (true)
            {
                int smallest = index;
                int left = 2 * index + 1;
                int right = 2 * index + 2;

                if (left < heap.Count && heap[left].CompareTo(heap[smallest]) < 0)
                    smallest = left;

                if (right < heap.Count && heap[right].CompareTo(heap[smallest]) < 0)
                    smallest = right;

                if (smallest == index)
                    break;

                Swap(index, smallest);
                index = smallest;
            }
        }

        private void Swap(int i, int j)
        {
            PQNode temp = heap[i];
            heap[i] = heap[j];
            heap[j] = temp;
        }
    }

    /// <summary>
    /// Finds shortest path using Dijkstra's algorithm on the image graph
    /// Graph: pixels are vertices, 4-connected edges with weights = 1/G (G = edge energy)
    /// </summary>
    public class ShortestPathFinder
    {
        private RGBPixel[,] imageMatrix;
        private int height;
        private int width;

        // Dijkstra results
        private double[,] distance;
        private Point[,] parent;
        private bool[,] visited;

        // Current anchor point
        private Point anchorPoint;
        private bool hasAnchor;

        // 4-connectivity: up, down, left, right
        private static readonly int[] dRow = { -1, 1, 0, 0 };
        private static readonly int[] dCol = { 0, 0, -1, 1 };

        // Precomputed edge weights for efficiency
        private double[,] weightRight;  // Weight to right neighbor (col+1)
        private double[,] weightDown;   // Weight to bottom neighbor (row+1)

        public ShortestPathFinder()
        {
            hasAnchor = false;
        }

        /// <summary>
        /// Set the image and precompute edge weights
        /// Time: O(N²) where N = max(width, height)
        /// </summary>
        public void SetImage(RGBPixel[,] imageMatrix)
        {
            this.imageMatrix = imageMatrix;
            this.height = ImageToolkit.GetHeight(imageMatrix);
            this.width = ImageToolkit.GetWidth(imageMatrix);

            // Precompute all edge weights (graph construction)
            BuildGraph();

            hasAnchor = false;
        }

        /// <summary>
        /// Build the weighted graph by precomputing edge weights
        /// Weight = 1/G where G is edge energy (strength)
        /// Low G (homogeneous) -> high weight, High G (edge) -> low weight
        /// Time: O(N²)
        /// </summary>
        private void BuildGraph()
        {
            weightRight = new double[height, width];
            weightDown = new double[height, width];

            const double epsilon = 0.0001; // Avoid division by zero

            for (int row = 0; row < height; row++)
            {
                for (int col = 0; col < width; col++)
                {
                    // Get edge energy: X = right, Y = down
                    Vector2D energy = ImageToolkit.CalculatePixelEnergies(col, row, imageMatrix);

                    // Weight = 1/G (inverse of energy)
                    // Higher energy (strong edge) -> lower weight (attractive)
                    weightRight[row, col] = 1.0 / (energy.X + epsilon);
                    weightDown[row, col] = 1.0 / (energy.Y + epsilon);
                }
            }
        }

        /// <summary>
        /// Get edge weight between two adjacent pixels
        /// </summary>
        private double GetEdgeWeight(int row1, int col1, int row2, int col2)
        {
            // Ensure (row1,col1) is the smaller one for consistency
            if (row2 < row1 || col2 < col1)
            {
                int tr = row1; row1 = row2; row2 = tr;
                int tc = col1; col1 = col2; col2 = tc;
            }

            if (row2 == row1 + 1) // Down edge
                return weightDown[row1, col1];
            else // Right edge
                return weightRight[row1, col1];
        }

        /// <summary>
        /// Set anchor point and run Dijkstra from it
        /// Time: O(E' log V') where E' and V' are visited edges/vertices
        /// </summary>
        public void SetAnchorPoint(int x, int y)
        {
            anchorPoint = new Point(x, y);
            hasAnchor = true;

            // Run Dijkstra from anchor point
            RunDijkstra(y, x); // Note: y = row, x = col
        }

        /// <summary>
        /// Dijkstra's algorithm using min-heap priority queue
        /// Computes shortest path from source to all reachable vertices
        /// Time: O(E log V) where E = O(4*V) = O(V) for 4-connected grid
        /// </summary>
        private void RunDijkstra(int srcRow, int srcCol)
        {
            // Initialize arrays
            distance = new double[height, width];
            parent = new Point[height, width];
            visited = new bool[height, width];

            for (int r = 0; r < height; r++)
            {
                for (int c = 0; c < width; c++)
                {
                    distance[r, c] = double.MaxValue;
                    parent[r, c] = new Point(-1, -1);
                    visited[r, c] = false;
                }
            }

            // Min-heap priority queue
            MinHeap pq = new MinHeap();

            // Initialize source
            distance[srcRow, srcCol] = 0;
            pq.Insert(new PQNode(srcRow, srcCol, 0));

            while (pq.Count > 0)
            {
                PQNode current = pq.ExtractMin();
                int row = current.Row;
                int col = current.Col;

                // Skip if already visited (stale entry)
                if (visited[row, col])
                    continue;

                visited[row, col] = true;

                // Explore 4 neighbors
                for (int i = 0; i < 4; i++)
                {
                    int newRow = row + dRow[i];
                    int newCol = col + dCol[i];

                    // Check bounds
                    if (newRow < 0 || newRow >= height || newCol < 0 || newCol >= width)
                        continue;

                    // Skip if already visited
                    if (visited[newRow, newCol])
                        continue;

                    // Calculate new distance
                    double weight = GetEdgeWeight(row, col, newRow, newCol);
                    double newDist = distance[row, col] + weight;

                    // Relaxation
                    if (newDist < distance[newRow, newCol])
                    {
                        distance[newRow, newCol] = newDist;
                        parent[newRow, newCol] = new Point(col, row); // Store as (x, y)
                        pq.Insert(new PQNode(newRow, newCol, newDist));
                    }
                }
            }
        }

        /// <summary>
        /// Backtrack shortest path from free point to anchor point
        /// Time: O(N) where N = path length (at most width + height)
        /// </summary>
        public List<Point> GetPathToAnchor(int freeX, int freeY)
        {
            List<Point> path = new List<Point>();

            if (!hasAnchor || imageMatrix == null)
                return path;

            // Bounds check
            if (freeX < 0 || freeX >= width || freeY < 0 || freeY >= height)
                return path;

            // Backtrack from free point to anchor
            int curRow = freeY;
            int curCol = freeX;

            while (curRow != -1 && curCol != -1)
            {
                path.Add(new Point(curCol, curRow));

                // Check if we reached anchor
                if (curCol == anchorPoint.X && curRow == anchorPoint.Y)
                    break;

                Point p = parent[curRow, curCol];
                curCol = p.X;
                curRow = p.Y;
            }

            return path;
        }

        /// <summary>
        /// Draw the shortest path on a Graphics object
        /// Time: O(N) where N = path length
        /// </summary>
        public void DrawPath(Graphics g, int freeX, int freeY, Color pathColor, int thickness = 2)
        {
            List<Point> path = GetPathToAnchor(freeX, freeY);

            if (path.Count < 2)
                return;

            using (Pen pen = new Pen(pathColor, thickness))
            {
                for (int i = 0; i < path.Count - 1; i++)
                {
                    g.DrawLine(pen, path[i], path[i + 1]);
                }
            }
        }

        /// <summary>
        /// Draw anchor point marker
        /// </summary>
        public void DrawAnchorPoint(Graphics g, Color color, int size = 6)
        {
            if (!hasAnchor)
                return;

            using (SolidBrush brush = new SolidBrush(color))
            {
                g.FillEllipse(brush, anchorPoint.X - size / 2, anchorPoint.Y - size / 2, size, size);
            }
        }

        public bool HasAnchor => hasAnchor;
        public Point AnchorPoint => anchorPoint;
    }

    /// <summary>
    /// Manages multiple anchor points for creating closed selections
    /// </summary>
    public class MultiAnchorPathFinder
    {
        private RGBPixel[,] imageMatrix;
        private ShortestPathFinder pathFinder;
        private List<Point> anchorPoints;
        private List<List<Point>> confirmedPaths;

        public MultiAnchorPathFinder()
        {
            pathFinder = new ShortestPathFinder();
            anchorPoints = new List<Point>();
            confirmedPaths = new List<List<Point>>();
        }

        public void SetImage(RGBPixel[,] imageMatrix)
        {
            this.imageMatrix = imageMatrix;
            pathFinder.SetImage(imageMatrix);
            Clear();
        }

        /// <summary>
        /// Add a new anchor point
        /// </summary>
        public void AddAnchorPoint(int x, int y)
        {
            // If there's a previous anchor, save the path to this new point
            if (anchorPoints.Count > 0)
            {
                List<Point> path = pathFinder.GetPathToAnchor(x, y);
                if (path.Count > 0)
                    confirmedPaths.Add(path);
            }

            anchorPoints.Add(new Point(x, y));
            pathFinder.SetAnchorPoint(x, y);
        }

        /// <summary>
        /// Get the live path from last anchor to free point
        /// </summary>
        public List<Point> GetLivePath(int freeX, int freeY)
        {
            return pathFinder.GetPathToAnchor(freeX, freeY);
        }

        /// <summary>
        /// Close the selection by connecting last anchor to first
        /// </summary>
        public void CloseSelection()
        {
            if (anchorPoints.Count < 2)
                return;

            Point first = anchorPoints[0];
            List<Point> closingPath = pathFinder.GetPathToAnchor(first.X, first.Y);
            if (closingPath.Count > 0)
                confirmedPaths.Add(closingPath);
        }

        /// <summary>
        /// Draw all paths and anchor points
        /// </summary>
        public void Draw(Graphics g, int freeX, int freeY, Color pathColor, Color anchorColor)
        {
            // Draw confirmed paths
            using (Pen pen = new Pen(pathColor, 2))
            {
                foreach (var path in confirmedPaths)
                {
                    if (path.Count < 2) continue;
                    for (int i = 0; i < path.Count - 1; i++)
                    {
                        g.DrawLine(pen, path[i], path[i + 1]);
                    }
                }
            }

            // Draw live path
            pathFinder.DrawPath(g, freeX, freeY, pathColor, 2);

            // Draw all anchor points
            using (SolidBrush brush = new SolidBrush(anchorColor))
            {
                foreach (var anchor in anchorPoints)
                {
                    g.FillEllipse(brush, anchor.X - 4, anchor.Y - 4, 8, 8);
                }
            }
        }

        /// <summary>
        /// Clear all anchors and paths
        /// </summary>
        public void Clear()
        {
            anchorPoints.Clear();
            confirmedPaths.Clear();
        }

        public int AnchorCount => anchorPoints.Count;
        public bool HasAnchors => anchorPoints.Count > 0;
    }
}
