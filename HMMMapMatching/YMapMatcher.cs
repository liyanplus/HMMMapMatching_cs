using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using CsvHelper;
using Itinero;
using Itinero.Osm.Vehicles;

namespace HMMMapMatching
{
    public class YMapMatcher
    {
        #region property
        public List<YTrackingPoint> TrackingPointList { get; set; }
        public int[] ResultSnapIdxList { get; set; }

        public YGraph BaseGraph { get; set; }
        #endregion

        public YMapMatcher(string p_pbfFilePath, string p_trackingPointFilePath, double p_snapRange = 30, double p_maxSnapRange = 200, int p_historyLength = 2)
        {
            BaseGraph = new YGraph(p_pbfFilePath);
            _LoadCSVTrackingPointFile(p_trackingPointFilePath, p_snapRange, p_maxSnapRange);
            if (TrackingPointList.Count < 50)
            {
                return;
            }

            ResultSnapIdxList = new int[TrackingPointList.Count];

            _PartialViterbi(p_historyLength, 0);

            var directory = Path.GetDirectoryName(p_trackingPointFilePath);
            var fileName = Path.GetFileNameWithoutExtension(p_trackingPointFilePath);
            _GenerateMatchedRouteCSV(Path.Combine(directory, fileName + ".csv.res"));
        }

        private void _LoadCSVTrackingPointFile(string p_trackingPointFilePath, double p_snapRange, double p_maxSnapRange)
        {
            TrackingPointList = new List<YTrackingPoint>();
            using (TextReader fileReader = File.OpenText(p_trackingPointFilePath))
            using (CsvReader csv = new CsvReader(fileReader))
            {
                csv.Configuration.HasHeaderRecord = true;
                csv.Read();
                csv.ReadHeader();
                while (csv.Read())
                {
                    int timeStamp = Convert.ToInt32(csv["Time"]);
                    float lat = 0;
                    float lon = 0;
                    YTrackingPoint newPoint;

                    if (!string.Equals("", csv["Latitude"]) && !string.Equals("", csv["Energy Used"]))
                    {
                        lat = Convert.ToSingle(csv["Latitude"]);
                        lon = Convert.ToSingle(csv["Longitude"]);

                        newPoint = new YTrackingPoint(TrackingPointList.Count, lat, lon);
                        newPoint.TimeId = timeStamp;

                        foreach (var header in csv.Context.HeaderRecord)
                        {
                            newPoint.Attributes.Add(header, csv[header]);
                        }

                        if (TrackingPointList.Count == 0 || newPoint.DistanceTo(TrackingPointList[TrackingPointList.Count - 1]) > 20)
                        {
                            BaseGraph.SnapPointToEdges(newPoint, p_snapRange, p_maxSnapRange);
                            if (newPoint.SnapPointList.Count != 0)
                            {
                                TrackingPointList.Add(newPoint);
                            }
                        }
                    }
                }
            }
        }

        private List<MatchState> _GenerateMatchState(int p_currentTrackingId, int p_historyLength, int p_startTrackingId)
        {
            // generate match state of tracking point with id of p_currentTrackingId
            // each MatchState is a list of snapped points' id belonging to the previous and later p_historyLength and current tracking points.
            List<MatchState> states = new List<MatchState>();
            states.Add(new MatchState() { CurrenTrackingPointId = p_currentTrackingId });

            for (int i = -p_historyLength; i < p_historyLength + 1; i++)
            {
                int ptId = p_currentTrackingId + i;
                if (ptId < p_startTrackingId || ptId > TrackingPointList.Count - 1)
                {
                    continue;
                }

                if (i == 0)
                {
                    foreach (var oldState in states)
                    {
                        oldState.ZeroIdx = oldState.Count;
                    }
                }

                List<MatchState> tempList = new List<MatchState>();
                foreach (var oldState in states)
                {
                    foreach (var ptSnap in TrackingPointList[ptId].SnapPointList)
                    {
                        var newState = new MatchState(oldState);
                        newState.Add(ptSnap.Id);
                        tempList.Add(newState);
                    }
                }
                states = tempList;
            }

            return states;
        }

        private void _PartialViterbi(int p_historyLength, int p_startTrackingIdx)
        {
            if (p_startTrackingIdx >= TrackingPointList.Count)
            {
                return;
            }
            // the probability of states at the current step
            var probRecords = new List<double>();
            // the backward states index
            List<List<int>> backwardStateIdxes = new List<List<int>>();
            // the snap point index corresponding to the backward state
            List<Dictionary<int, int>> backwardSnapPtIdxes = new List<Dictionary<int, int>>();

            var previousStates = _GenerateMatchState(p_startTrackingIdx, p_historyLength, p_startTrackingIdx);
            List<MatchState> currentStates;

            // initialization the 1st record 
            foreach (var state in previousStates)
            {
                probRecords.Add(Math.Log(TrackingPointList[p_startTrackingIdx].SnapPointList[state.CurrentSnapPointId].SnappedEmissionProb));
            }

            int endTrackingIdx = -1;
            // traverse through the rest records            
            for (int i = p_startTrackingIdx + 1; i < TrackingPointList.Count; i++)
            {
                var currentProb = new List<double>();
                var previousStateIdxes = new List<int>();
                var previousSnapPtIdxes = new Dictionary<int, int>();

                currentStates = _GenerateMatchState(i, p_historyLength, p_startTrackingIdx);

                var effectiveStates = new List<MatchState>();

                foreach (var currentState in currentStates)
                {
                    double currentCumulativeMaxProb = double.MinValue;
                    // a criterion to check the transition probability
                    // if it is too small, break the trajectory.
                    double currentMaxProb = double.MinValue;
                    int currentMaxIdx = 0;

                    for (int previouwStateIdx = 0; previouwStateIdx < previousStates.Count; previouwStateIdx++)
                    {
                        if (BaseGraph.AreAdjacentStates(previousStates[previouwStateIdx], currentState, TrackingPointList, p_historyLength))
                        {
                            var temp = Math.Log(BaseGraph.TransitProbBetween(previousStates[previouwStateIdx], currentState, TrackingPointList, p_historyLength))
                                + Math.Log(TrackingPointList[i].SnapPointList[currentState.CurrentSnapPointId].SnappedEmissionProb);
                            var tempCumulative = probRecords[previouwStateIdx] + temp;

                            if (tempCumulative >= currentCumulativeMaxProb)
                            {
                                currentCumulativeMaxProb = tempCumulative;
                                currentMaxIdx = previouwStateIdx;
                            }

                            if (temp >= currentMaxProb)
                            {
                                currentMaxProb = temp;
                            }
                        }
                    }

                    if (currentMaxProb > -1e100)
                    {
                        effectiveStates.Add(currentState);

                        currentProb.Add(currentCumulativeMaxProb);
                        previousStateIdxes.Add(currentMaxIdx);
                        if (!previousSnapPtIdxes.ContainsKey(currentMaxIdx))
                        {
                            previousSnapPtIdxes.Add(currentMaxIdx, previousStates[currentMaxIdx].CurrentSnapPointId);
                        }
                    }
                }

                if (i - p_historyLength >= 0)
                {
                    BaseGraph.PopDistanceCache(i - p_historyLength);
                }

                if (effectiveStates.Count == 0)
                {
                    // the transition is impossible                    
                    break;
                }
                else
                {
                    // the transition is possible
                    probRecords = currentProb;
                    backwardStateIdxes.Add(previousStateIdxes);
                    backwardSnapPtIdxes.Add(previousSnapPtIdxes);
                    previousStates = effectiveStates;
                    endTrackingIdx = i;
                }
            }

            // backtracking
            if (endTrackingIdx == -1)
            {
                _PartialViterbi(p_historyLength, p_startTrackingIdx + 1);
            }
            else
            {
                int backtrackingStateIdx = 0;
                double finalStateProb = double.MinValue;
                for (int i = 0; i < probRecords.Count; i++)
                {
                    if (finalStateProb < probRecords[i])
                    {
                        finalStateProb = probRecords[i];
                        backtrackingStateIdx = i;
                    }
                }
                ResultSnapIdxList[endTrackingIdx] = previousStates[backtrackingStateIdx].CurrentSnapPointId;
                for (int i = endTrackingIdx - 1; i >= p_startTrackingIdx; i--)
                {
                    backtrackingStateIdx = backwardStateIdxes[i - p_startTrackingIdx][backtrackingStateIdx];
                    ResultSnapIdxList[i] = backwardSnapPtIdxes[i - p_startTrackingIdx][backtrackingStateIdx];
                }

                if (endTrackingIdx != TrackingPointList.Count - 1)
                {
                    if (endTrackingIdx - p_historyLength + 1 <= p_startTrackingIdx)
                    {
                        _PartialViterbi(p_historyLength, p_startTrackingIdx + 1);
                    }
                    else
                    {
                        _PartialViterbi(p_historyLength, endTrackingIdx - p_historyLength + 1);
                    }
                }
            }
        }


        private void _GenerateMatchedRouteCSV(string p_matchedRouteCSVFilePath)
        {
            List<bool> directions = new List<bool>();
            List<RouterPoint> pathPoints = new List<RouterPoint>();

            for (int i = 0; i < ResultSnapIdxList.Length; i++)
            {
                int snapId = ResultSnapIdxList[i];
                YSnapPoint pt = TrackingPointList[i].SnapPointList[snapId];

                directions.Add(pt.Direction);
                pathPoints.Add(BaseGraph.GraphRouter.Resolve(Vehicle.Car.Fastest(), pt.Latitude, pt.Longitude));
            }

            List<RoadSegmentCost> pathCosts = new List<RoadSegmentCost>();
            pathCosts.Add(new RoadSegmentCost()
            {
                RoadId = pathPoints[0].EdgeId,
                Cost = 0
            });

            for (int i = 0; i < pathPoints.Count - 1; i++)
            {
                var leftEnergy = Convert.ToDouble(TrackingPointList[i].Attributes["Energy Used"]);
                var rightEnergy = Convert.ToDouble(TrackingPointList[i + 1].Attributes["Energy Used"]);
                var totalCost = rightEnergy - leftEnergy;

                var leftPt = pathPoints[i];
                var rightPt = pathPoints[i + 1];

                var leftDir = directions[i];
                var rightDir = directions[i + 1];

                var rt = BaseGraph.GraphRouter.TryCalculate(Vehicle.Car.Fastest(),
                                                            leftPt, leftDir, rightPt, rightDir);

                if (rt.IsError || rt.Value.Shape.Length < 2)
                {
                    continue;
                }
                else
                {
                    var edgeCoordinates = rt.Value.Shape;
                    double totalDist = 0;

                    var lastPt = edgeCoordinates[0];
                    var lastWayId = leftPt.EdgeId;

                    var currentPathCosts = new List<RoadSegmentCost>();
                    currentPathCosts.Add(new RoadSegmentCost()
                    {
                        RoadId = lastWayId,
                        Cost = 0
                    });
                    for (int j = 1; j < edgeCoordinates.Length; j++)
                    {
                        var currentPt = edgeCoordinates[j];
                        var midPt = new Itinero.LocalGeo.Coordinate()
                        {
                            Latitude = (lastPt.Latitude + currentPt.Latitude) / 2,
                            Longitude = (lastPt.Longitude + currentPt.Longitude) / 2
                        };

                        var resolvedMidPt = BaseGraph.GraphRouter.Resolve(Vehicle.Car.Fastest(),
                                                                              midPt);
                        var currentDist = YPointBase.DistanceTo(lastPt.Latitude, lastPt.Longitude,
                            currentPt.Latitude, currentPt.Longitude);
                        currentDist = currentDist > 0 ? currentDist : 1;
                        totalDist += currentDist;
                        if (resolvedMidPt.EdgeId == lastWayId)
                        {
                            currentPathCosts[currentPathCosts.Count - 1].Cost += currentDist;
                        }
                        else
                        {
                            currentPathCosts.Add(new RoadSegmentCost()
                            {
                                RoadId = resolvedMidPt.EdgeId,
                                Cost = currentDist
                            });
                        }

                        lastPt = currentPt;
                        lastWayId = resolvedMidPt.EdgeId;
                    }

                    for (int j = 0; j < currentPathCosts.Count; j++)
                    {
                        if (pathCosts[pathCosts.Count - 1].RoadId == currentPathCosts[j].RoadId)
                        {
                            pathCosts[pathCosts.Count - 1].Cost += totalCost * currentPathCosts[j].Cost / totalDist;
                        }
                        else
                        {
                            pathCosts.Add(
                                new RoadSegmentCost()
                                {
                                    RoadId = currentPathCosts[j].RoadId,
                                    Cost = totalCost * currentPathCosts[j].Cost / totalDist
                                });
                        }
                    }
                }
            }

            using (var resultWriter = File.CreateText(p_matchedRouteCSVFilePath))
            using (var csvWriter = new CsvWriter(resultWriter))
            {
                csvWriter.WriteField("WayId");
                csvWriter.WriteField("Cost");
                csvWriter.NextRecord();

                foreach (var seg in pathCosts)
                {
                    csvWriter.WriteField(seg.RoadId);
                    csvWriter.WriteField(seg.Cost);
                    csvWriter.NextRecord();
                }
            }
        }
    }
}
