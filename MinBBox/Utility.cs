using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;
using Rhino.Collections;

namespace MinBBox
{
    public class Utility
    {

        #region method
        /// <summary>
        /// Find P0.Transfer the points onto planeXY to compute.
        /// </summary>
        /// <param name="pts"></param>
        /// <param name="TransToPtsPlane"></param>
        /// <returns></returns>
        public static Point3d SearchP0(List<Point3d> pts)
        {
            Point3dList pt3dList = new Point3dList(pts);
            List<double> yList = new List<double>();
            Plane worldXY = Plane.WorldXY;
            Plane ptPlane = new Plane(pt3dList[1], pt3dList[2], pt3dList[3]);
            Transform TransToXY = Transform.ChangeBasis(ptPlane, Plane.WorldXY);
            pt3dList.Transform(TransToXY);

            int minIndex = 0;
            double y;
            for (int i = 0; i < pt3dList.Count; i++)
            {
                yList.Add(pt3dList[i].Y);
            }
            double ymin = yList[0];
            for (int i = 1; i < yList.Count; i++)
            {
                y = yList[i];
                if (y < ymin)
                {
                    ymin = y;
                    minIndex = i;
                }
                //If the y values equal, remain the smaller one.
                if (y == ymin)
                {
                    if (pt3dList[i].X < pt3dList[minIndex].X)
                    {
                        ymin = y;
                        minIndex = i;
                    }
                }
            }
            return pt3dList[minIndex];
        }

        /// <summary>
        /// Compute the distance between 2 points.
        /// </summary>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <returns></returns>
        public static double Dis(Point3d p1, Point3d p2)
        {
            return Math.Sqrt(Math.Pow(p1.X - p2.X, 2) + Math.Pow(p1.Y - p2.Y, 2)) + Math.Pow(p1.Z - p2.Z, 2);
        }

        /// <summary>
        /// Compute the Cross product of 2 points P1&P2.
        /// </summary>
        /// <param name="pointEdge_1"></param>
        /// <param name="point"></param>
        /// <returns></returns>
        public static double Multi(Point3d p1, Point3d p2, Point3d p0)
        {
            double x1 = p1.X - p0.X;
            double x2 = p2.X - p0.X;

            double y1 = p1.Y - p0.Y;
            double y2 = p2.Y - p0.Y;

            return x1 * y2 - x2 * y1;
        }

        public static double DotProduct3d(Vector3d v1, Vector3d v2)
        {
            return v1.X * v2.X + v1.Y * v2.Y + v1.Z * v2.Z;
        }

        /// <summary>
        /// Compute the angle between 2 vectors.
        /// </summary>
        /// <param name="vecA"></param>
        /// <param name="vecB"></param>
        /// <returns></returns>
        public static double GetAngleAB(Vector3d vecA, Vector3d vecB)
        {
            double dot = DotProduct3d(vecA, vecB);
            double multA = Math.Sqrt(Math.Pow(vecA.X, 2) + Math.Pow(vecA.Y, 2) + Math.Pow(vecA.Z, 2));
            double multB = Math.Sqrt(Math.Pow(vecB.X, 2) + Math.Pow(vecB.Y, 2) + Math.Pow(vecB.Z, 2));
            double cosAB = (dot / multA) / multB;
            double angleAB = Math.Acos(cosAB);
            angleAB = Utility.RadiansToDegrees(angleAB);
            return angleAB;
        }


        /// <summary>
        /// Compute the length of a vector.
        /// </summary>
        /// <param name="vector"></param>
        /// <returns></returns>
        public static double Magnitude(Vector3d vector)
        {
            return Math.Sqrt(Math.Pow(vector.X, 2) + Math.Pow(vector.Y, 2) + Math.Pow(vector.Z, 2));
        }

        public static double RadiansToDegrees(double radians)
        {
            return radians * 180 / Math.PI;
        }


        /// <summary>
        /// Compute the length v1 projected on v2.
        /// </summary>
        /// <param name="v1"></param>
        /// <param name="v2"></param>
        /// <returns></returns>
        public static double Projected(Vector3d v1, Vector3d v2)
        {
            double dot = DotProduct3d(v1, v2);
            double proj = dot / Magnitude(v2);
            return proj;
        }

        /// <summary>
        /// Compute the height of Y to line AB.
        /// </summary>
        /// <param name="PointA"></param>
        /// <param name="PointB"></param>
        /// <param name="PointY"></param>
        /// <returns></returns>
        public static double HeightABY(Point3d PointA, Point3d PointB, Point3d PointY)
        {
            Vector3d v1 = new Vector3d(PointY.X - PointA.X, PointY.Y - PointA.Y, PointY.Z - PointA.Z);
            Vector3d v2 = new Vector3d(PointB.X - PointA.X, PointB.Y - PointA.Y, PointY.Z - PointB.Z);

            double proj = Projected(v1, v2);
            double h = Math.Sqrt(Math.Pow(Magnitude(v1), 2) - Math.Pow(proj, 2));
            return h;
        }

        /// <summary>
        /// Compute the width of Convex hull.
        /// </summary>
        /// <param name="PointA"></param>
        /// <param name="PointB"></param>
        /// <param name="pTest"></param>
        /// <returns></returns>
        public static double WidthAB(Point3d PointA, Point3d PointB, List<Point3d> pTest, int n, out Point3d p1, out Point3d p2, out double d1, out double d2)
        {
            Vector3d vector1 = new Vector3d(PointB.X - PointA.X, PointB.Y - PointA.Y, PointB.Z - PointA.Z);
            double d;
            d1 = -1;
            d2 = 1;
            //Index of max width point on the positive side.
            int maxD = 0;   
            //Index of max width point on the negetive side.
            int VmaxD = 0;  
            for (int i = 0; i < pTest.Count; i++)
            {
                int index = 0;
                if (i + n > pTest.Count)
                {
                    index = i + n - pTest.Count;
                }
                else
                {
                    index = i + n;
                }
                Point3d pt = pTest[index];
                Vector3d vector2 = new Vector3d(pt.X - PointA.X, pt.Y - PointA.Y, pt.Z - PointA.Z);
                d = Projected(vector2, vector1);
                //Compute the max distance on the positive side.
                if (d > d1)
                {
                    d1 = d;
                    maxD = index;
                }
                else
                {
                    break;
                }
            }
            //Compute the max distance on the negetive side.
            for (int i = 0; i < pTest.Count; i++)
            {
                int index = 0;
                if (pTest.Count - i + n > pTest.Count)
                {
                    index = n - i;
                }
                else
                {
                    index = pTest.Count - i + n;
                }
                Point3d pt = pTest[index - 1];
                Vector3d vector2 = new Vector3d(pt.X - PointA.X, pt.Y - PointA.Y, pt.Z - PointA.Z);
                d = Projected(vector2, vector1);
                if (d < d2)
                {
                    d2 = d;
                    VmaxD = index - 1;
                }
                else
                {
                    break;
                }
            }
            //Find the boundary points.
            p1 = pTest[maxD];
            p2 = pTest[VmaxD];
            return d1 - d2;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="dotProduct"></param>
        /// <param name="mag1"></param>
        /// <param name="mag2"></param>
        /// <returns></returns>
        public static double Angle(double dotProduct, double mag1, double mag2)
        {
            if (mag1 == 0 || mag2 == 0)
            {
                return 180;
            }
            double result = Utility.RadiansToDegrees(Math.Acos((dotProduct) / (mag1 * mag2)));
            return result;
        }

        public static double GetAngle(Vector3d P1, Vector3d P2)
        {
            double dotProduct = 0;
            double mag1 = 0;
            double mag2 = 0;
            double angle = 0;

            if (P1 != null && P2 != null)
            {
                //get the dot product
                //arcos(A DOT B / ||A|| * ||B||)
                dotProduct = Utility.DotProduct3d(P1, P2);

                mag1 = Utility.Magnitude(P1);
                mag2 = Utility.Magnitude(P2);


                angle = Utility.Angle(dotProduct, mag1, mag2);
                angle = Math.Round(angle, 2);
                return angle;
            }
            return 0;
        }

        /// <summary>
        /// Sort all the points anticlockwise.
        /// </summary>
        /// <param name="pts3D"></param>
        /// <param name="angleList"></param>
        /// <returns>Return the sorted point list.P0 is the 1st.</returns>
        public static List<Point3d> SortPts(List<Point3d> pts3D, out List<double> angleList)
        {
            angleList = new List<double>();
            //Find P0 and remove
            Point3d P0 = SearchP0(pts3D);
            pts3D.Remove(pts3D.ElementAt(0));
            Point3d P1 = P0;
            P1.X += 1;
            Vector3d vector0 = new Vector3d(P1.X - P0.X, P1.Y - P0.Y, P1.Z - P0.Z);
            double angle;
            Dictionary<double, Point3d> ptList = new Dictionary<double, Point3d>();
            foreach (Point3d P in pts3D)
            {
                Vector3d v = new Vector3d(P.X - P0.X, P.Y - P0.Y, P.Z - P0.Z);
                //Get the angles of each point.
                angle = GetAngle(v, vector0);
                //If 2 points share the same angle, remain the further one.
                if (ptList.Keys.Contains(angle))
                {
                    Point3d ptCurrent = ptList[angle];
                    Vector3d vecCurrent = new Vector3d(ptCurrent.X - P0.X, ptCurrent.Y - P0.Y, ptCurrent.Z - P0.Z);
                    if (vecCurrent.Length < v.Length)
                    {
                        ptList[angle] = P;
                    }
                    else
                    {
                        continue;
                    }
                }
                else
                {
                    ptList.Add(angle, P);
                }
            }
            var resultPTs = from item in ptList orderby item.Key select item;
            List<Point3d> SortedPts = new List<Point3d>();
            for (int i = 0; i < resultPTs.Count(); i++)
            {
                SortedPts.Add(resultPTs.ElementAt(i).Value);
                angleList.Add(resultPTs.ElementAt(i).Key);
            }
            angleList.Sort();
            SortedPts.Insert(0, P0);
            
            //Remove the points that are too close.
            Utility.RemoveSimilarPts(SortedPts);
            return SortedPts;
        }

        /// <summary>
        /// Remove the points that are too close.In this the precision is set to 0.001
        /// </summary>
        /// <param name="SortedPts"></param>
        /// <returns></returns>
        public static List<Point3d> RemoveSimilarPts(List<Point3d> SortedPts)
        {
            for (int i = 0; i < SortedPts.Count; i++)
            {
                Point3d Pi = SortedPts[i];
                for (int j = 0; j < SortedPts.Count - i - 1; j++)
                {
                    Point3d Pj = SortedPts[i + j + 1];
                    if (Math.Abs( Pi.X - Pj.X) < 0.01 && Math.Abs( Pi.Y - Pj.Y) < 0.01)
                    {
                        SortedPts.RemoveAt(i + j + 1);
                    }
                }
            }
            return SortedPts;
        }
        #endregion

    }
}
