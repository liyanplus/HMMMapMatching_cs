using System;
namespace HMMMapMatching
{
    public class YEdge
    {
        #region property
        public int Id
        {
            get;
            set;
        }

        public int StartNodeId
        {
            get;
            set;
        }
        public int EndNodeId
        {
            get;
            set;
        }

        #endregion
        public YEdge()
        {
        }
    }
}
