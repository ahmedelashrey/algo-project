using System;
using System.Drawing;
using System.Windows.Forms;

namespace MagniSnap
{
    #region
    /// 4d17639adfad0a300acd78759e07a4f2
    #endregion
    public partial class MainForm : Form
    {
        RGBPixel[,] ImageMatrix;
        bool isLassoEnabled = false;

        // Shortest path finder for livewire
        MultiAnchorPathFinder pathFinder;
        Point currentMousePos;

        public MainForm()
        {
            InitializeComponent();
            indicator_pnl.Hide();
            pathFinder = new MultiAnchorPathFinder();
            currentMousePos = new Point(0, 0);

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
                //Open the browsed image and display it
                string OpenedFilePath = openFileDialog1.FileName;
                ImageMatrix = ImageToolkit.OpenImage(OpenedFilePath);
                ImageToolkit.ViewImage(ImageMatrix, mainPictureBox);

                int width = ImageToolkit.GetWidth(ImageMatrix);
                txtWidth.Text = width.ToString();
                int height = ImageToolkit.GetHeight(ImageMatrix);
                txtHeight.Text = height.ToString();

                // Initialize path finder with the image
                pathFinder.SetImage(ImageMatrix);
            }
        }

        private void clearToolStripMenuItem_Click(object sender, EventArgs e)
        {
            // Clear the livewire paths
            if (ImageMatrix != null)
                pathFinder.SetImage(ImageMatrix);
            mainPictureBox.Refresh();
        }

        private void btnLivewire_Click(object sender, EventArgs e)
        {
            menuButton_Click(sender, e);

            mainPictureBox.Cursor = Cursors.Cross;

            isLassoEnabled = true;
        }

        private void btnLivewire_Leave(object sender, EventArgs e)
        {
            menuButton_Leave(sender, e);

            mainPictureBox.Cursor = Cursors.Default;
            isLassoEnabled = false;
        }

        private void mainPictureBox_MouseClick(object sender, MouseEventArgs e)
        {
            if (ImageMatrix == null || !isLassoEnabled)
                return;

            int width = ImageToolkit.GetWidth(ImageMatrix);
            int height = ImageToolkit.GetHeight(ImageMatrix);

            // Bounds check
            if (e.X < 0 || e.X >= width || e.Y < 0 || e.Y >= height)
                return;

            if (e.Button == MouseButtons.Left)
            {
                // Add anchor point and run Dijkstra
                pathFinder.AddAnchorPoint(e.X, e.Y);
                mainPictureBox.Refresh();
            }
            else if (e.Button == MouseButtons.Right)
            {
                // Close selection (connect last anchor to first)
                pathFinder.CloseSelection();
                mainPictureBox.Refresh();
            }
        }

        private void mainPictureBox_MouseMove(object sender, MouseEventArgs e)
        {
            txtMousePosX.Text = e.X.ToString();
            txtMousePosY.Text = e.Y.ToString();

            currentMousePos = new Point(e.X, e.Y);

            if (ImageMatrix != null && isLassoEnabled && pathFinder.HasAnchors)
            {
                // Refresh to redraw the live path
                mainPictureBox.Invalidate();
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
            if (ImageMatrix == null || !isLassoEnabled || !pathFinder.HasAnchors)
                return;

            // Draw all paths (confirmed + live)
            pathFinder.Draw(e.Graphics, currentMousePos.X, currentMousePos.Y,
                Color.Lime, Color.Red);
        }

        private void saveToolStripMenuItem_Click(object sender, EventArgs e)
        {
            
        }
    }
}