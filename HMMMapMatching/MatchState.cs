using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HMMMapMatching
{
    public class MatchState : List<int>
    {
        // a state of a tracking point
        // the default state include previous 2 snapped points and later 2 snapped points        
        #region Property
        public int CurrenTrackingPointId { get; set; }
        public int ZeroIdx { get; set; }
        public int CurrentSnapPointId
        {
            get
            {
                return this[ZeroIdx];
            }
        }
        #endregion
        #region Construcor
        public MatchState() { }
        public MatchState(MatchState another)
        {
            CurrenTrackingPointId = another.CurrenTrackingPointId;
            ZeroIdx = another.ZeroIdx;
            foreach (var snapId in another)
            {
                this.Add(snapId);
            }
        }
        #endregion
    }
}
