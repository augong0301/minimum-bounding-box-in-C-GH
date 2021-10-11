using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace MinBBox.Algorithm
{
    /// <summary>
    /// RC stands for rotating calipers algorithm
    /// </summary>
    public class MinBBox
    {
        protected List<Point3d> _pts3d;
        #region properties
        public List<Point3d> ptList
        {
            get => this._pts3d;
        }

        public Transform Trans
        {
            get
            {
                Plane ptsPlane = new Plane(_pts3d[1], _pts3d[2], _pts3d[3]);
                Transform transToPtsPlane = Transform.ChangeBasis(Plane.WorldXY, ptsPlane);
                return transToPtsPlane;
            }
        }

        public Point3d P0
        {
            get
            {
                return Utility.SearchP0(ptList);
            }
        }

        public Point3d P1
        {
            get
            {
                List<double> angleList;
                return Utility.SortPts(ptList, out angleList)[1];
            }
        }

        /// <summary>
        /// The list of convex hull points.
        /// </summary>
        public List<Point2d> Sta
        {
            get; set;
        }

        public int Top
        {
            get; set;
        }

        #endregion

        #region constructor
        public MinBBox(List<Point3d> pts3D)
        {
            this._pts3d = pts3D;

        }
        #endregion

        #region Method
        /// <summary>
        /// Compute the position relation between p1 & p2 based on p0. If return false, p2 will be removed.
        /// </summary>
        /// <param name="P1"></param>
        /// <param name="P2"></param>
        /// <param name="P0"></param>
        /// <returns></returns>
        public bool Cmp(Point3d P1, Point3d P2, Point3d P0)
        {
            double tt = Utility.Multi(P1, P2, P0);
            if (tt < 0)
            {
                return false;
            }
            //If P1&P2 are on the same line, the further on will remain.
            if (tt == 0 && Utility.Dis(P1, P0) > Utility.Dis(P2, P0))
            {
                return false;
            }
            return true;
        }

        /// <summary>
        /// Graham Scan algorithm.This is used to scan all the points
        /// </summary>
        /// <param name="pts"></param>
        /// <param name="selPts"></param>
        public void GrahamScan(List<Point3d> pts, ref List<Point3d> selPts)
        {
            if (pts.Count > 0)
            {
                var pt = pts[0];
                if (selPts.Count <= 1)
                {
                    selPts.Add(pt);
                    pts.RemoveAt(0);
                }
                else
                {
                    var pt1 = selPts[selPts.Count - 1];
                    var pt2 = selPts[selPts.Count - 2];
                    Vector3d dir1 = pt1 - pt2;
                    Vector3d dir2 = pt - pt1;
                    //var cross = Vector3d.CrossProduct(dir1, dir2);

                    double mul = Utility.Multi(selPts[selPts.Count - 1], pt, selPts[selPts.Count - 2]);

                    if (mul < 0)
                    {
                        selPts.RemoveAt(selPts.Count - 1);
                    }
                    else
                    {
                        selPts.Add(pt);
                        pts.RemoveAt(0);
                    }
                }


            }

        }

        /// <summary>
        /// Graham algorithm.
        /// Scan all the points anticlockwise.If P2 is on the clockwise of P1, then P2 is not the boundary point of the final convex hull.
        /// Traverse all the points.
        /// </summary>
        /// <param name="Sta"></param>
        /// <param name="n"></param>
        /// <returns></returns>
        public bool Graham(List<Point3d> sortedPts, out List<Point3d> Sta)
        {
            //The Graham scan will return to P0, which is the start point of the point list.
            List<Point3d> selPts = new List<Point3d>();
            while (sortedPts.Count > 0)
            {
                GrahamScan(sortedPts, ref selPts);
            }
            Sta = selPts;
            return true;
        }






        /// <summary>
        /// Main method of Rotate Caliper.
        /// </summary>
        /// <param name="ConvexPts">The points of convex hull.</param>
        /// <param name="RecXY">The bounding box rectangle of the given geometry.</param>
        /// <returns></returns>
        public bool RotateCaliper3D(List<Point3d> ConvexPts, out Polyline RecXY)
        {
            double h;
            double area;    
            double areaMin = double.MaxValue;
            double minD1 = 0;
            double minD2 = 0;
            double minH = 0;
            //Record the index of the longest distance between points pair.
            int minArea_n = 0;
            int minArea_j = 0;

            Plane planePTS = new Plane(_pts3d[1], _pts3d[2], _pts3d[3]);
            Transform TransToPtsPlane = Transform.ChangeBasis(Plane.WorldXY, planePTS);

            Point3d edgeP1 = new Point3d();
            Point3d edgeP2 = new Point3d();

            for (int n = 3; n < ConvexPts.Count; n++)
            {
                int j = 2;
                int index_plus = 0;
                
                Vector3d v1 = new Vector3d(ConvexPts[n + 1].X - ConvexPts[n].X, ConvexPts[n + 1].Y - ConvexPts[n].Y, ConvexPts[n + 1].Z - ConvexPts[n].Z);
                Vector3d v2 = new Vector3d(1, 0, 0);
                if (n + j + 1 >= ConvexPts.Count)
                {
                    index_plus = n + j + 1 - ConvexPts.Count;
                }
                else
                {
                    index_plus = n + j + 1;
                }
                int index = index_plus - 1;
                //Find the furthest point of given point.
                while (Math.Abs(Utility.Multi(ConvexPts[n], ConvexPts[n + 1], ConvexPts[index])) < Math.Abs(Utility.Multi(ConvexPts[n], ConvexPts[n + 1], ConvexPts[index_plus])))
                {
                    index_plus++;
                    index++;
                    if (index_plus >= ConvexPts.Count)
                    {
                        index_plus = 0;
                    }
                    if(index >= ConvexPts.Count)
                    {
                        index = 0;
                    }
                }
                Point3d currentP3 = new Point3d();
                Point3d currentP4 = new Point3d();
                h = Utility.HeightABY(ConvexPts[n], ConvexPts[n + 1], ConvexPts[index]);
                area = h * Utility.WidthAB(ConvexPts[n], ConvexPts[n + 1], ConvexPts, n, out currentP3, out currentP4, out double d1, out double d2);
                //Update the min area of rectangle and the index of i,j.
                if (area < areaMin)
                {
                    areaMin = area;
                    minArea_n = n;
                    minD1 = d1;
                    minD2 = d2;
                    minH = h;
                }
                //Rotating stops when it passes through 90°.
                if (Utility.DotProduct3d(v1, v2) < 0)
                {
                    break;
                }
            }
            //Compute the min area
            edgeP1 = ConvexPts[minArea_n];
            edgeP2 = ConvexPts[minArea_n + 1];
            //Find the four vertices of rectangle.
            Vector2d vectorMin = new Vector2d(edgeP2.X - edgeP1.X, edgeP2.Y - edgeP1.Y);
            vectorMin.Unitize();
            Vector2d vectD1 = new Vector2d(vectorMin.X * minD1, vectorMin.Y * minD1);
            Vector2d vectD2 = new Vector2d(vectorMin.X * minD2, vectorMin.Y * minD2);
            Point2d Cal1 = new Point2d(edgeP1.X + vectD1.X, edgeP1.Y + vectD1.Y);
            Point2d Cal2 = new Point2d(edgeP1.X + vectD2.X, edgeP1.Y + vectD2.Y);
            //Rotate 90°
            vectorMin.Rotate(Math.PI * 0.5);
            Vector2d vectD3 = new Vector2d(vectorMin.X * minH, vectorMin.Y * minH);
            Vector2d vectD4 = new Vector2d(vectorMin.X * minH, vectorMin.Y * minH);

            Point2d Cal3 = new Point2d(Cal1.X + vectD3.X, Cal1.Y + vectD3.Y);
            Point2d Cal4 = new Point2d(Cal2.X + vectD4.X, Cal2.Y + vectD4.Y);
            List<Point2d> CalRec = new List<Point2d>();
            CalRec.AddRange(new List<Point2d> { Cal1, Cal2, Cal4, Cal3, Cal1});

            List<Point3d> resultPTs = new List<Point3d>();
            for (int i = 0; i < CalRec.Count; i++)
            {
                Point2d pt2d = CalRec[i];
                Point3d pt3d = new Point3d(pt2d.X, pt2d.Y, 0);
                resultPTs.Add(pt3d);
            }
            Polyline Rec = new Polyline(resultPTs);
            RecXY = Rec;
            //RecXY.Transform(this.Trans);
            return true;
        }


        //public bool Run()
        //{

        //}
        #endregion
    }
}
