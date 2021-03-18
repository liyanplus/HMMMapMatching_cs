using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HMMMapMatching
{
    public class YSnapPoint : YPointBase
    {
        #region property
        public bool Direction { get; set; }
        public double SnappedDistance { get; set; }
        public double SnappedEmissionProb { get; set; }
        public uint SnapWayId { get; set; }
        #endregion

        public YSnapPoint()
        {
        }

        public YSnapPoint(YSnapPoint p_pt)
            : base(p_pt.Id, p_pt.Latitude, p_pt.Longitude)
        {
            SnappedDistance = p_pt.SnappedDistance;
            SnappedEmissionProb = p_pt.SnappedEmissionProb;
        }

        public YSnapPoint(int p_id, float p_lat, float p_lon)
            : base(p_id, p_lat, p_lon)
        {
        }
    }
}
