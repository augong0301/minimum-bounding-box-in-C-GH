using Grasshopper.Kernel;
using System;
using System.Drawing;

namespace MinBBox
{
    public class MinBBoxInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "MinBBox";
            }
        }
        public override Bitmap Icon
        {
            get
            {
                //Return a 24x24 pixel bitmap to represent this GHA library.
                return null;
            }
        }
        public override string Description
        {
            get
            {
                //Return a short string describing the purpose of this GHA library.
                return "";
            }
        }
        public override Guid Id
        {
            get
            {
                return new Guid("fae3f44b-160f-405d-8da0-2f6232c92b87");
            }
        }

        public override string AuthorName
        {
            get
            {
                //Return a string identifying you or your company.
                return "";
            }
        }
        public override string AuthorContact
        {
            get
            {
                //Return a string representing your preferred contact details.
                return "";
            }
        }
    }
}
