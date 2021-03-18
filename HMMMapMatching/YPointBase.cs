using System;
namespace HMMMapMatching
{
    public class YPointBase
    {
        #region property
        public int Id
        {
            get;
            set;
        }
        public float Latitude
        {
            get;
            set;
        }
        public float Longitude
        {
            get;
            set;
        }
        #endregion
        #region contructor
        public YPointBase()
        {
        }

        public YPointBase(int p_id, float p_lat, float p_lon)
        {
            Id = p_id;
            Latitude = p_lat;
            Longitude = p_lon;
        }
        #endregion

        public double DistanceTo(YPointBase p_anotherPoint)
        {
            return DistanceTo(this.Latitude, this.Longitude, p_anotherPoint.Latitude, p_anotherPoint.Longitude);
        }

        public static double DistanceTo(double p_lat1, double p_lon1, double p_lat2, double p_lon2)
        {
            Func<double, double> degreeToRadian = (angle) => angle * Math.PI / 180.0;

            var R = 6371e3; // metres
            var phi1 = degreeToRadian(p_lat1);
            var phi2 = degreeToRadian(p_lat2);
            var deltaPhi = degreeToRadian(p_lat2 - p_lat1);
            var deltaLamda = degreeToRadian(p_lon2 - p_lon1);

            var a = Math.Sin(deltaPhi / 2) * Math.Sin(deltaPhi / 2) +
                        Math.Cos(phi1) * Math.Cos(phi2) *
                        Math.Sin(deltaLamda / 2) * Math.Sin(deltaLamda / 2);
            var c = 2 * Math.Atan2(Math.Sqrt(a), Math.Sqrt(1 - a));

            return R * c;
        }
    }
}
