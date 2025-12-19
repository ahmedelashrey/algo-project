using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace MagniSnap
{
    #region
    /// 4d17639adfad0a300acd78759e07a4f2
    #endregion
    public partial class MainForm : Form
    {
        RGBPixel[,] ImageMatrix;
        bool isLassoActive = false;   // tool accepts input & live paths
        bool isLassoVisible = false;  // selection is drawn

        // Livewire (windowed)
        MultiAnchorPathFinder pathFinder;

        // Raw mouse position (for UI)
        Point currentMousePos;

        // Effective draw position (may be clamped to the current window)
        Point liveDrawPos;

        public MainForm()
        {
            InitializeComponent();
            indicator_pnl.Hide();

            // Window size is configured inside ShortestPathFinder (default 128x128)
            pathFinder = new MultiAnchorPathFinder();

            currentMousePos = new Point(0, 0);
            liveDrawPos = new Point(0, 0);

            // Enable double buffering to reduce flicker
            mainPictureBox.GetType().GetProperty("DoubleBuffered",
                System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic)
                .SetValue(mainPictureBox, true, null);
        }

        private void menuButton_Click(object sender, EventArgs e)
        {
            #region Do Change Remove Template Code
            /// 4d17639adfad0a300acd78759e07a4f2
            #endregion

            indicator_pnl.Top = ((Control)sender).Top;
            indicator_pnl.Height = ((Control)sender).Height;
            indicator_pnl.Left = ((Control)sender).Left;
            ((Control)sender).BackColor = Color.FromArgb(37, 46, 59);
            indicator_pnl.Show();
        }

        private void menuButton_Leave(object sender, EventArgs e)
        {
            ((Control)sender).BackColor = Color.FromArgb(26, 32, 40);
        }

        private void exitToolStripMenuItem_Click(object sender, EventArgs e)
        {
            Application.Exit();
        }

        private void openToolStripMenuItem_Click(object sender, EventArgs e)
        {
            #region Do Change Remove Template Code
            /// 4d17639adfad0a300acd78759e07a4f2
            #endregion

            OpenFileDialog openFileDialog1 = new OpenFileDialog();
            if (openFileDialog1.ShowDialog() == DialogResult.OK)
            {
                // Open the browsed image and display it
                string OpenedFilePath = openFileDialog1.FileName;
                ImageMatrix = ImageToolkit.OpenImage(OpenedFilePath);
                ImageToolkit.ViewImage(ImageMatrix, mainPictureBox);

                int width = ImageToolkit.GetWidth(ImageMatrix);
                txtWidth.Text = width.ToString();
                int height = ImageToolkit.GetHeight(ImageMatrix);
                txtHeight.Text = height.ToString();

                // Initialize path finder with the image
                pathFinder.SetImage(ImageMatrix);

                liveDrawPos = new Point(0, 0);
                mainPictureBox.Refresh();
            }
        }

        private void clearToolStripMenuItem_Click(object sender, EventArgs e)
        {
            // Clear the livewire paths
            if (ImageMatrix != null)
                pathFinder.SetImage(ImageMatrix);

            liveDrawPos = currentMousePos;
            mainPictureBox.Refresh();
        }

        private void btnLivewire_Click(object sender, EventArgs e)
        {
            menuButton_Click(sender, e);

            mainPictureBox.Cursor = Cursors.Cross;

            isLassoActive = true;
            isLassoVisible = true;
        }

        private void btnLivewire_Leave(object sender, EventArgs e)
        {
            menuButton_Leave(sender, e);

            mainPictureBox.Cursor = Cursors.Default;
            isLassoActive = false;
            isLassoVisible = false;
        }

        private void mainPictureBox_MouseClick(object sender, MouseEventArgs e)
        {
            if (ImageMatrix == null || !isLassoActive)
                return;

            int width = ImageToolkit.GetWidth(ImageMatrix);
            int height = ImageToolkit.GetHeight(ImageMatrix);

            // Bounds check
            if (e.X < 0 || e.X >= width || e.Y < 0 || e.Y >= height)
                return;

            if (e.Button == MouseButtons.Left)
            {
                // Add anchor point (may internally "walk" in window-steps if far)
                pathFinder.AddAnchorPoint(e.X, e.Y);

                // Update live draw pos
                liveDrawPos = pathFinder.UpdateCursor(e.X, e.Y);

                mainPictureBox.Refresh();
            }
            else if (e.Button == MouseButtons.Right)
            {
                // Close selection (connect last anchor to first)
                pathFinder.CloseSelection();

                // Exit lasso tool: stop live interaction but keep drawing
                isLassoActive = false;
                mainPictureBox.Cursor = Cursors.Default;

                mainPictureBox.Refresh();
            }
        }

        private void mainPictureBox_MouseMove(object sender, MouseEventArgs e)
        {
            txtMousePosX.Text = e.X.ToString();
            txtMousePosY.Text = e.Y.ToString();

            currentMousePos = new Point(e.X, e.Y);

            if (ImageMatrix != null && isLassoActive && pathFinder.HasAnchors)
            {
                // Prepare / clamp cursor for the current window and redraw
                liveDrawPos = pathFinder.UpdateCursor(e.X, e.Y);
                mainPictureBox.Invalidate();
            }
            else
            {
                liveDrawPos = currentMousePos;
            }
        }

        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);
            // Subscribe to Paint event for drawing livewire
            mainPictureBox.Paint += mainPictureBox_Paint;
        }

        private void mainPictureBox_Paint(object sender, PaintEventArgs e)
        {
            if (ImageMatrix == null || !isLassoVisible || !pathFinder.HasAnchors)
                return;

            // Draw all paths (confirmed + live)
            pathFinder.Draw(e.Graphics, liveDrawPos.X, liveDrawPos.Y,
                Color.Lime, Color.Red);
        }

        private void saveToolStripMenuItem_Click(object sender, EventArgs e)
        {
            if (ImageMatrix == null)
                return;

            // Ensure selection is closed
            if (!pathFinder.IsClosed)
            {
                MessageBox.Show("Please close the selection before saving.",
                    "Incomplete Selection",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Warning);
                return;
            }

            Point[] polygon = pathFinder.GetClosedPolygon();
            if (polygon == null || polygon.Length < 3)
                return;

            SaveFileDialog sfd = new SaveFileDialog();
            sfd.Filter = "PNG Image (*.png)|*.png";
            sfd.DefaultExt = "png";
            sfd.FileName = string.Format("selection_{0:yyyy-MM-dd_HH-mm-ss}.png", DateTime.Now);

            if (sfd.ShowDialog() != DialogResult.OK)
                return;

            // Compute bounding box
            Rectangle bounds = GetPolygonBounds(polygon);

            Bitmap result = new Bitmap(bounds.Width, bounds.Height,
                System.Drawing.Imaging.PixelFormat.Format32bppArgb);

            using (Graphics g = Graphics.FromImage(result))
            using (GraphicsPath path = new GraphicsPath())
            {
                g.Clear(Color.Transparent);

                // Shift polygon to local coordinates
                Point[] shifted = new Point[polygon.Length];
                for (int i = 0; i < polygon.Length; i++)
                {
                    shifted[i] = new Point(
                        polygon[i].X - bounds.X,
                        polygon[i].Y - bounds.Y);
                }

                path.AddPolygon(shifted);
                g.SetClip(path);

                // Draw cropped area
                g.DrawImage(
                    mainPictureBox.Image,
                    new Rectangle(0, 0, bounds.Width, bounds.Height),
                    bounds,
                    GraphicsUnit.Pixel);
            }

            result.Save(sfd.FileName, System.Drawing.Imaging.ImageFormat.Png);
        }

        private static Rectangle GetPolygonBounds(Point[] pts)
        {
            int minX = int.MaxValue, minY = int.MaxValue;
            int maxX = int.MinValue, maxY = int.MinValue;

            foreach (Point p in pts)
            {
                if (p.X < minX) minX = p.X;
                if (p.Y < minY) minY = p.Y;
                if (p.X > maxX) maxX = p.X;
                if (p.Y > maxY) maxY = p.Y;
            }

            return Rectangle.FromLTRB(minX, minY, maxX + 1, maxY + 1);
        }
    }
}
