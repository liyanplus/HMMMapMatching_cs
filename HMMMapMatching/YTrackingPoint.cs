using System;
using System.Collections.Generic;

namespace HMMMapMatching
{
    public class YTrackingPoint : YPointBase
    {
        #region property
        public int TimeId { get; set; }

        public List<YSnapPoint> SnapPointList { get; set; }
        public Dictionary<string, string> Attributes { get; set; }

        //public List<MatchState> matchedStates { get; set; }
        #endregion
        public YTrackingPoint()
        {
            SnapPointList = new List<YSnapPoint>();
            Attributes = new Dictionary<string, string>();
        }

        public YTrackingPoint(int p_id, float p_lat, float p_lon)
            : base(p_id, p_lat, p_lon)
        {
            SnapPointList = new List<YSnapPoint>();
            Attributes = new Dictionary<string, string>();
        }

        public void AddSnapPoint(uint p_wayId, YSnapPoint p_pt, bool isOneWay)
        {
            foreach (var pt in SnapPointList)
            {
                if (pt.DistanceTo(p_pt) < 2)
                    return;
            }
            p_pt.Id = SnapPointList.Count;
            p_pt.Direction = true;
            p_pt.SnapWayId = p_wayId;

            double sigmaZ = 100;

            p_pt.SnappedEmissionProb = 1 / (Math.Sqrt(2 * Math.PI) * sigmaZ) * Math.Exp(-0.5 * Math.Pow(p_pt.SnappedDistance / sigmaZ, 2));
            SnapPointList.Add(p_pt);

            if (isOneWay)
            {
                return;
            }

            var reversePt = new YSnapPoint(p_pt);
            reversePt.Id = SnapPointList.Count;
            reversePt.Direction = false;
            reversePt.SnapWayId = p_wayId;
            SnapPointList.Add(reversePt);
        }

        
    }
}
