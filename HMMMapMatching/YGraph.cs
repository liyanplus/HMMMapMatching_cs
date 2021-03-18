using NetTopologySuite.Geometries;
using OsmSharp.Streams;
using OsmSharp.Geo;
using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;

using Itinero;
using Itinero.IO.Osm;
using Itinero.IO.Shape;
using Itinero.Osm.Vehicles;

namespace HMMMapMatching
{
    public class YGraph
    {
        private double m_BETA = 50;

        #region property
        public IEnumerable<NetTopologySuite.Features.IFeature> WayList
        {
            get;
            set;
        }

        public Router GraphRouter { get; set; }

        public Dictionary<int, Dictionary<int, Dictionary<int, Dictionary<int, double>>>> DistanceCache { get; set; }

        #endregion

        #region constructor
        public YGraph()
        {
        }

        public YGraph(string p_pbfFilePath, float p_filterLeft = 0, float p_filterTop = 0, float p_filterRight = 0, float p_filterBotom = 0)
        {
            //p_pbfFilePath = @"C:\Users\lixx4266\Google Drive\Research\MapMatching\data\base_map.osm";
            using (var osmFileStream = File.OpenRead(p_pbfFilePath))
            {
                // create source stream.
                var source = new XmlOsmStreamSource(osmFileStream);
                if (p_filterLeft != 0)
                {
                    source = (XmlOsmStreamSource)source.FilterBox(p_filterLeft, p_filterTop, p_filterRight, p_filterBotom);
                }

                var filteredSource = from feature in source
                                     where feature.Type == OsmSharp.OsmGeoType.Node || (
                                     feature.Type == OsmSharp.OsmGeoType.Way && feature.Tags != null && (
                                     feature.Tags.Contains("highway", "motorway") || feature.Tags.Contains("highway", "trunk") || feature.Tags.Contains("highway", "primary") || feature.Tags.Contains("highway", "secondary") ||
                                     feature.Tags.Contains("highway", "tertiary") || feature.Tags.Contains("highway", "unclassified") || feature.Tags.Contains("highway", "residential") ||
                                     feature.Tags.Contains("highway", "service") || feature.Tags.Contains("highway", "motorway_link") || feature.Tags.Contains("highway", "trunk_link") ||
                                     feature.Tags.Contains("highway", "primary_link") || feature.Tags.Contains("highway", "secondary_link") || feature.Tags.Contains("highway", "tertiary_link") || feature.Tags.Contains("highway", "living_street"))
                                     )
                                     select feature;

               
                WayList = filteredSource.ToFeatureSource().ToList();
            }

            // load routerDb data
            RouterDb rtDb = new RouterDb();
            if (File.Exists(p_pbfFilePath + ".routerdb"))
            {
                using (var routerDbStream = File.OpenRead(p_pbfFilePath + ".routerdb"))
                {
                    rtDb = RouterDb.Deserialize(routerDbStream);
                }
            }
            else
            {
                using (var osmFileStream = File.OpenRead(p_pbfFilePath))
                {
                    // create source stream.
                    var source = new XmlOsmStreamSource(osmFileStream);
                    if (p_filterLeft != 0)
                    {
                        source = (XmlOsmStreamSource)source.FilterBox(p_filterLeft, p_filterTop, p_filterRight, p_filterBotom);
                    }

                    rtDb.LoadOsmData(source, Vehicle.Car);
                    using (var routerDbStream = File.OpenWrite(p_pbfFilePath + ".routerdb"))
                    {
                        rtDb.Serialize(routerDbStream);
                    }
                }

                var profiles = new Itinero.Profiles.Profile[] {
                    rtDb.GetSupportedProfile("car")
                };
                rtDb.WriteToShape("shapefile", profiles);
            }

            GraphRouter = new Router(rtDb);


            // initialize distance cache
            DistanceCache = new Dictionary<int, Dictionary<int, Dictionary<int, Dictionary<int, double>>>>();
        }
        #endregion

        #region method
        public void SnapPointToEdges(YTrackingPoint p_pt, double p_snapRange, double p_maxSnapRange)
        {
            if (p_maxSnapRange < p_snapRange)
            {
                p_maxSnapRange = p_snapRange + 100;
            }
            int snapCount = 0;
            do
            {
                foreach (var way in WayList)
                {
                    if (_ReturnSnappedPoint(p_pt, way, p_snapRange))
                    {
                        snapCount++;
                    }
                }
                p_snapRange += 20;
            } while (snapCount == 0 && p_snapRange <= p_maxSnapRange);
        }

        private bool _ReturnSnappedPoint(YTrackingPoint p_pt, NetTopologySuite.Features.IFeature p_way, double p_snapRange)
        {
            var edgeCoordinates = p_way.Geometry.Coordinates;
            // there are 2 cases:
            // 1. there is perpendicular points on the way, add all perpendicular points.
            // 2. there is no perpendicular point, add the closed end point.

            bool isSnapped = false;
            bool isEndPoint = false;

            // record the closed end point
            double minSnapX = 0;
            double minSnapY = 0;
            double minSnapDistance = double.MaxValue;

            // enumerate through the way's edges
            for (int vertexId = 0; vertexId < edgeCoordinates.Length - 1; vertexId++)
            {
                isEndPoint = false;

                var x1 = edgeCoordinates[vertexId].X;
                var y1 = edgeCoordinates[vertexId].Y;
                var x2 = edgeCoordinates[vertexId + 1].X;
                var y2 = edgeCoordinates[vertexId + 1].Y;

                var k = ((y2 - y1) * (p_pt.Longitude - x1) - (x2 - x1) * (p_pt.Latitude - y1)) / (Math.Pow(y2 - y1, 2) + Math.Pow(x2 - x1, 2));
                var snapX = p_pt.Longitude - k * (y2 - y1);
                var snapY = p_pt.Latitude + k * (x2 - x1);

                if ((snapX - x1) / (x2 - x1) < 0 || (snapY - y1) / (y2 - y1) < 0)
                {
                    isEndPoint = true;
                    snapX = x1;
                    snapY = y1;
                }
                else if ((snapX - x1) / (x2 - x1) > 1 || (snapY - y1) / (y2 - y1) > 1)
                {
                    isEndPoint = true;
                    snapX = x2;
                    snapY = y2;
                }

                YSnapPoint resultSnapPoint = new YSnapPoint() { Longitude = Convert.ToSingle(snapX), Latitude = Convert.ToSingle(snapY) };
                resultSnapPoint.SnappedDistance = p_pt.DistanceTo(resultSnapPoint);

                if (resultSnapPoint.SnappedDistance < p_snapRange)
                {
                    if (!isEndPoint)
                    {
                        // if the snapped point is not an end point, add it
                        RouterPoint tempPt = GraphRouter.Resolve(Vehicle.Car.Fastest(), Convert.ToSingle(snapY), Convert.ToSingle(snapX));
                        p_pt.AddSnapPoint(tempPt.EdgeId,
                            resultSnapPoint, p_way.Attributes.Exists("oneway") && "yes".Equals(p_way.Attributes["oneway"]));
                        isSnapped = true;
                    }
                    else if (resultSnapPoint.SnappedDistance < minSnapDistance)
                    {
                        // if the snapped point is an end point, and it is closer than the current snapped end point, replace the recorded snapped end point
                        minSnapDistance = resultSnapPoint.SnappedDistance;
                        minSnapX = snapX;
                        minSnapY = snapY;
                    }
                }
            }

            if (!isSnapped && minSnapX != 0)
            {
                // if there is no perpendicular snapped point and exists a snapped end point.
                RouterPoint tempPt = GraphRouter.Resolve(Vehicle.Car.Fastest(), Convert.ToSingle(minSnapY), Convert.ToSingle(minSnapX));
                p_pt.AddSnapPoint(tempPt.EdgeId,
                    new YSnapPoint() { Longitude = Convert.ToSingle(minSnapX), Latitude = Convert.ToSingle(minSnapY), SnappedDistance = minSnapDistance },
                    p_way.Attributes.Exists("oneway") && "yes".Equals(p_way.Attributes["oneway"]));
                isSnapped = true;
            }
            return isSnapped;
        }

        public bool AreAdjacentStates(MatchState p_aState, MatchState p_bState, List<YTrackingPoint> p_trackingPointList, int p_historyLength)
        {
            int stepGap = p_bState.CurrenTrackingPointId - p_aState.CurrenTrackingPointId;
            int startMatchingIdx = p_aState.ZeroIdx - (p_historyLength - stepGap);
            startMatchingIdx = startMatchingIdx < 0 ? 0 : startMatchingIdx;

            bool isAdjacent = true;
            for (int matchingIdx = startMatchingIdx; matchingIdx < p_aState.Count; matchingIdx++)
            {
                if (p_aState[matchingIdx] != p_bState[matchingIdx - startMatchingIdx])
                {
                    isAdjacent = false;
                    break;
                }
            }

            return isAdjacent;
        }

        public double StateProb(MatchState p_state, List<YTrackingPoint> p_trackingPointList, int p_historyLength)
        {
            List<YTrackingPoint> routeTPs = new List<YTrackingPoint>();
            List<int> snapIds = new List<int>();
            for (int i = 0; i < p_state.Count; i++)
            {
                routeTPs.Add(p_trackingPointList[p_state.CurrenTrackingPointId - p_state.ZeroIdx + i]);
                snapIds.Add(p_state[i]);
            }

            double startEndSnappedDistance = _RouteDistanceBetweenSnapPoints(routeTPs[0], snapIds[0],
                routeTPs[routeTPs.Count - 1], snapIds[snapIds.Count - 1]);

            double actualDistance = 0;
            for (int i = 0; i < routeTPs.Count - 1; i++)
            {
                double stepDistance = _RouteDistanceBetweenSnapPoints(routeTPs[i], snapIds[i],
                    routeTPs[i + 1], snapIds[i + 1]);
                if (stepDistance == double.NaN)
                {
                    return 0;
                }
                actualDistance += stepDistance;
            }
            return 1 / m_BETA * Math.Exp(-Math.Abs(startEndSnappedDistance - actualDistance) / m_BETA);
        }

        public double TransitProbBetween(MatchState p_aState, MatchState p_bState, List<YTrackingPoint> p_trackingPointList, int p_historyLength)
        {
            double prob = 1;
            // comparison between matched route and shortest route
            prob *= _ComparisonBetweenMatchedAndShortestRoute(p_aState, p_bState, p_trackingPointList, p_historyLength);
            return prob;
        }

        private double _RouteDistanceBetweenSnapPoints(YTrackingPoint p_aTracking, int p_aSnappedId, YTrackingPoint p_bTracking, int p_bSnappedId)
        {
            if (DistanceCache.ContainsKey(p_aTracking.Id)
                && DistanceCache[p_aTracking.Id].ContainsKey(p_bTracking.Id)
                && DistanceCache[p_aTracking.Id][p_bTracking.Id].ContainsKey(p_aSnappedId)
                && DistanceCache[p_aTracking.Id][p_bTracking.Id][p_aSnappedId].ContainsKey(p_bSnappedId))
            {
                return DistanceCache[p_aTracking.Id][p_bTracking.Id][p_aSnappedId][p_bSnappedId];
            }
            else
            {
                RouterPoint startPoint = GraphRouter.Resolve(Vehicle.Car.Fastest(), p_aTracking.SnapPointList[p_aSnappedId].Latitude, p_aTracking.SnapPointList[p_aSnappedId].Longitude);
                RouterPoint endPoint = GraphRouter.Resolve(Vehicle.Car.Fastest(), p_bTracking.SnapPointList[p_bSnappedId].Latitude, p_bTracking.SnapPointList[p_bSnappedId].Longitude);

                if (_RoutePointEqual(startPoint, endPoint))
                {
                    _PushToDistanceCache(p_aTracking, p_aSnappedId, p_bTracking, p_bSnappedId, 0);
                    return 0;
                }
                else
                {
                    var rt = GraphRouter.TryCalculate(Vehicle.Car.Fastest(),
                        startPoint, p_aTracking.SnapPointList[p_aSnappedId].Direction,
                        endPoint, p_bTracking.SnapPointList[p_bSnappedId].Direction);
                    if (rt.IsError)
                    {
                        _PushToDistanceCache(p_aTracking, p_aSnappedId, p_bTracking, p_bSnappedId, double.NaN);
                        return double.NaN;
                    }
                    else
                    {
                        _PushToDistanceCache(p_aTracking, p_aSnappedId, p_bTracking, p_bSnappedId, rt.Value.TotalDistance);
                        return rt.Value.TotalDistance;
                    }
                }
            }
        }

        private void _PushToDistanceCache(YTrackingPoint p_aTracking, int p_aSnappedId, YTrackingPoint p_bTracking, int p_bSnappedId, double p_dist)
        {
            if (!DistanceCache.ContainsKey(p_aTracking.Id))
            {
                DistanceCache.Add(p_aTracking.Id, new Dictionary<int, Dictionary<int, Dictionary<int, double>>>());
            }
            if (!DistanceCache[p_aTracking.Id].ContainsKey(p_bTracking.Id))
            {
                DistanceCache[p_aTracking.Id].Add(p_bTracking.Id, new Dictionary<int, Dictionary<int, double>>());
            }
            if (!DistanceCache[p_aTracking.Id][p_bTracking.Id].ContainsKey(p_aSnappedId))
            {
                DistanceCache[p_aTracking.Id][p_bTracking.Id].Add(p_aSnappedId, new Dictionary<int, double>());
            }
            if (!DistanceCache[p_aTracking.Id][p_bTracking.Id][p_aSnappedId].ContainsKey(p_bSnappedId))
            {
                DistanceCache[p_aTracking.Id][p_bTracking.Id][p_aSnappedId].Add(p_bSnappedId, p_dist);
            }
        }

        public void PopDistanceCache(int p_TrackingPtId)
        {
            if (DistanceCache.ContainsKey(p_TrackingPtId))
            {
                DistanceCache.Remove(p_TrackingPtId);
            }
        }

        private double _ComparisonBetweenMatchedAndShortestRoute(MatchState p_aState, MatchState p_bState, List<YTrackingPoint> p_trackingPointList, int p_historyLength)
        {
            List<YTrackingPoint> routeTPs = new List<YTrackingPoint>();
            List<int> snapIds = new List<int>();
            for (int i = 0; i < p_aState.Count; i++)
            {
                routeTPs.Add(p_trackingPointList[p_aState.CurrenTrackingPointId - p_aState.ZeroIdx + i]);
                snapIds.Add(p_aState[i]);
            }
            if (p_bState.ZeroIdx <= (p_bState.Count - 1) / 2)
            {
                routeTPs.Add(p_trackingPointList[p_bState.CurrenTrackingPointId + p_bState.Count - p_bState.ZeroIdx - 1]);
                snapIds.Add(p_bState[p_bState.Count - 1]);
            }

            double startEndSnappedDistance = _RouteDistanceBetweenSnapPoints(routeTPs[0], snapIds[0],
                routeTPs[routeTPs.Count - 1], snapIds[snapIds.Count - 1]);

            if (startEndSnappedDistance == double.NaN)
            {
                return 0;
            }

            double actualDistance = 0;
            for (int i = 0; i < routeTPs.Count - 1; i++)
            {
                double stepDistance = _RouteDistanceBetweenSnapPoints(routeTPs[i], snapIds[i],
                    routeTPs[i + 1], snapIds[i + 1]);
                if (stepDistance == double.NaN)
                {
                    return 0;
                }
                actualDistance += stepDistance;
            }

            return 1 / m_BETA * Math.Exp(-Math.Abs(startEndSnappedDistance - actualDistance) / m_BETA);
        }

        private bool _RoutePointEqual(RouterPoint p_aPoint, RouterPoint p_bPoint)
        {
            if (Math.Abs(p_aPoint.Latitude - p_bPoint.Latitude) + Math.Abs(p_aPoint.Longitude - p_bPoint.Longitude) < 1e-4)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        #endregion

    }
}
