using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace MinBBox.Component
{
    public class MinBBoxComponent : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the test class.
        /// </summary>
        public MinBBoxComponent()
          : base("Minimum Bounding Box", "Min BBOX",
              "Find the minimum bounding box for a set of breps",
              "MinBBox", "MinBBox")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPointParameter("Points", "Points", "Points", GH_ParamAccess.list);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Rectangle polyline", "Rectangle polyline", "Rectangle polyline", GH_ParamAccess.item);
            pManager.AddPointParameter("Convex hull points", "Convex hull points", "Construct the points of the convex hull", GH_ParamAccess.list);
            //The following two are used for testing.
            //pManager.AddPointParameter("Sorted pts", "Sorted pts", "Sorted pts", GH_ParamAccess.list);
            //pManager.AddNumberParameter("Angles", "", "", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Point3d> pts = new List<Point3d>();
            List<Point3d> ConvexPts = new List<Point3d>();
            List<Point3d> SortedPts = new List<Point3d>();
            List<double> angleList;
            Polyline polyline;
            if (!DA.GetDataList<Point3d>(0, pts) || pts.Count == 0)
            {
                return;
            }
            Algorithm.MinBBox rc = new Algorithm.MinBBox(pts);
            Rhino.Collections.Point3dList point3Ds = new Rhino.Collections.Point3dList(pts);
            //Transfer the point3d onto PlaneXY
            point3Ds.Transform(rc.Trans);
            for (int i = 0; i < pts.Count; i++)
            {
                pts[i] = point3Ds[i];
            }
            SortedPts = Utility.SortPts(pts, out angleList);
            List<Point3d> SortedPts_out = new List<Point3d>(SortedPts);
            //Search the convex hull pts
            if (!rc.Graham(SortedPts, out ConvexPts))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "The Graham process is not working");
            }
            DA.SetDataList(1, ConvexPts);
            //Testing
            //DA.SetDataList(2, SortedPts_out);
            //DA.SetDataList(3, angleList);
            if (rc.RotateCaliper3D(ConvexPts, out polyline))
            {
                DA.SetData(0, polyline);
            }
            else
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "The RC process is not working");
            }
        }

        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("4fbc80c1-3cd3-4823-8a00-d63012dcd76e"); }
        }
    }
}