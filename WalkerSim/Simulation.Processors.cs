using System.Collections.Generic;
using System.IO;

namespace WalkerSim
{
    internal partial class Simulation
    {
        private delegate Vector3 MovementProcessorDelegate(State state, Agent agent, FixedBufferList<Agent> nearby, float distance, float power);

        class MovementProcessor
        {
            public List<Processor> Entries;
            public float SpeedScale = 1.0f;
            public int Group = -1;
            public System.Drawing.Color Color;
        }

        class Processor
        {
            public MovementProcessorDelegate Handler;
            public float Distance;
            public float Power;
            public float DistanceSqr;
        }

        private readonly Dictionary<Config.MovementProcessorType, MovementProcessorDelegate> ProcessorTypeToDelegateMap = new Dictionary<Config.MovementProcessorType, MovementProcessorDelegate>()
        {
            { Config.MovementProcessorType.FlockAnyGroup, FlockAny },
            { Config.MovementProcessorType.AlignAnyGroup, AlignAny },
            { Config.MovementProcessorType.AvoidAnyGroup, AvoidAny },
            { Config.MovementProcessorType.FlockSameGroup, FlockSame },
            { Config.MovementProcessorType.AlignSameGroup, AlignSame },
            { Config.MovementProcessorType.AvoidSameGroup, AvoidSame },
            { Config.MovementProcessorType.FlockOtherGroup, FlockOther },
            { Config.MovementProcessorType.AlignOtherGroup, AlignOther },
            { Config.MovementProcessorType.AvoidOtherGroup, AvoidOther },
            { Config.MovementProcessorType.Wind, Wind },
            { Config.MovementProcessorType.WindInverted, WindInverted },
            { Config.MovementProcessorType.StickToRoads, StickToRoads },
            { Config.MovementProcessorType.AvoidRoads, AvoidRoads },
            { Config.MovementProcessorType.StickToPOIs, StickToPOIs },
            { Config.MovementProcessorType.AvoidPOIs, AvoidPOIs },
            { Config.MovementProcessorType.WorldEvents, WorldEvents },
            { Config.MovementProcessorType.AvoidObstacles, AvoidObstacles },
        };

        private List<MovementProcessor> _processors = new List<MovementProcessor>();

        private MovementProcessorDelegate GetProcessorDelegate(Config.MovementProcessorType type)
        {
            return ProcessorTypeToDelegateMap[type];
        }

        private void SetupProcessors()
        {
            if (_state.GroupCount == 0)
            {
                return;
            }

            _processors.Clear();

            if (_state.Config.Processors.Count == 0)
            {
                // Nothing to do.
                return;
            }

            // Zero init the list based on group count.
            for (int i = 0; i < _state.GroupCount; i++)
            {
                _processors.Add(null);
            }

            // Build a list of groups, some specify the exact group some are generic.
            List<MovementProcessor> genericGroups = new List<MovementProcessor>();
            List<MovementProcessor> specificGroups = new List<MovementProcessor>();

            foreach (var processorGroup in _state.Config.Processors)
            {
                var processors = new List<Processor>();

                foreach (var processor in processorGroup.Entries)
                {
                    var entry = new Processor();
                    entry.Distance = processor.Distance;
                    entry.DistanceSqr = entry.Distance * entry.Distance;
                    entry.Power = processor.Power;
                    entry.Handler = GetProcessorDelegate(processor.Type);

                    processors.Add(entry);
                }

                var group = new MovementProcessor()
                {
                    Entries = processors,
                    SpeedScale = processorGroup.SpeedScale,
                    Group = processorGroup.Group,
                };

                if (processorGroup.Color == "")
                {
                    // Assign a default color of purple.
                    group.Color = System.Drawing.Color.Magenta;
                }
                else
                {
                    group.Color = Utils.ParseColor(processorGroup.Color);
                }

                if (group.Group < 0)
                    genericGroups.Add(group);
                else
                    specificGroups.Add(group);
            }

            // Fill the generic ones but alternate.
            if (genericGroups.Count > 0)
            {
                for (int i = 0; i < _state.GroupCount; i++)
                {
                    var group = genericGroups[i % genericGroups.Count];
                    var newGroup = new MovementProcessor()
                    {
                        Entries = group.Entries,
                        SpeedScale = group.SpeedScale,
                        Group = group.Group,
                        Color = group.Color
                    };
                    _processors[i] = newGroup;
                }
            }

            // Fill specific ones.
            for (int i = 0; i < specificGroups.Count; i++)
            {
                var group = specificGroups[i];

                if (group.Group >= _processors.Count)
                {
                    Logging.Warn("Group specifies the group index {0} but there are only a total of {1} groups.", group.Group, _processors.Count);
                }
                else
                {
                    var newGroup = new MovementProcessor()
                    {
                        Entries = group.Entries,
                        SpeedScale = group.SpeedScale,
                        Group = group.Group,
                        Color = group.Color
                    };
                    _processors[group.Group] = newGroup;
                }
            }

            // Colorize all the groups who are still transparent
            for (int i = 0; i < _processors.Count; i++)
            {
                var group = _processors[i];
                if (group == null)
                    continue;

                if (group.Color == System.Drawing.Color.Transparent)
                {
                    group.Color = ColorTable.GetColorForIndex(i);
                }
            }

            // Find the maximum query distance.
            _state.MaxNeighbourDistance = 0.0f;

            foreach (var processorList in _processors)
            {
                if (processorList == null)
                    continue;

                foreach (var processor in processorList.Entries)
                {
                    _state.MaxNeighbourDistance = System.Math.Max(_state.MaxNeighbourDistance, processor.Distance);
                }
            }

            var folderPath = ObstacleMap.Instance.worldFolder;
            if (string.IsNullOrEmpty(ObstacleMap.Instance.worldFolder))
            {
                var fallback = Path.GetDirectoryName(System.Reflection.Assembly.GetExecutingAssembly().Location);
                folderPath = fallback;
                Logging.Warn("worldFolder niet gezet — fallback gebruikt: {0}", fallback);
            }
            var dtmFile = System.IO.Path.Combine(folderPath, "dtm.raw");

            var mapSize =  this.WorldSize;
            var width = (int)mapSize.X;
            var height = (int)mapSize.Y;

            Logging.Info("HeightMap geladen met grootte {0}x{1}", width, height);

            HeightMapGenerator.GenerateObstacleMap(dtmFile, width, height, WorldMins);
        }

        private static Vector3 FlockAny(State state, Agent agent, FixedBufferList<Agent> nearby, float distanceSqr, float power)
        {
            // point toward the center of the flock (mean flock boid position)
            var mean = Vector3.Zero;
            var count = 0;
            for (int i = 0; i < nearby.Count; i++)
            {
                var neighbor = nearby[i];

                var dist = Vector3.DistanceSqr(agent.Position, neighbor.Position);
                if (dist > distanceSqr)
                    continue;
                mean += neighbor.Position;
                count++;
            }
            if (count == 0)
                return Vector3.Zero;

            mean /= count;

            var center = mean - agent.Position;
            return center * power;
        }

        private static Vector3 FlockSame(State state, Agent agent, FixedBufferList<Agent> nearby, float distanceSqr, float power)
        {
            // point toward the center of the flock (mean flock boid position)
            var mean = Vector3.Zero;
            var count = 0;
            for (int i = 0; i < nearby.Count; i++)
            {
                var neighbor = nearby[i];
                if (neighbor.Group != agent.Group)
                    continue;

                var dist = Vector3.DistanceSqr(agent.Position, neighbor.Position);
                if (dist > distanceSqr)
                    continue;
                mean += neighbor.Position;
                count++;
            }
            if (count == 0)
                return Vector3.Zero;

            mean /= count;

            var center = mean - agent.Position;
            return center * power;
        }

        private static Vector3 FlockOther(State state, Agent agent, FixedBufferList<Agent> nearby, float distanceSqr, float power)
        {
            // point toward the center of the flock (mean flock boid position)
            var mean = Vector3.Zero;
            var count = 0;
            for (int i = 0; i < nearby.Count; i++)
            {
                var neighbor = nearby[i];
                if (neighbor.Group == agent.Group)
                    continue;

                var dist = Vector3.DistanceSqr(agent.Position, neighbor.Position);
                if (dist > distanceSqr)
                    continue;
                mean += neighbor.Position;
                count++;
            }
            if (count == 0)
                return Vector3.Zero;

            mean /= count;

            var center = mean - agent.Position;
            return center * power;
        }

        private static Vector3 AlignAny(State state, Agent agent, FixedBufferList<Agent> nearby, float distanceSqr, float power)
        {
            // point toward the center of the flock (mean flock boid position)
            var meanVel = Vector3.Zero;
            var count = 0;
            for (int i = 0; i < nearby.Count; i++)
            {
                var neighbor = nearby[i];

                var dist = Vector3.DistanceSqr(agent.Position, neighbor.Position);
                if (dist > distanceSqr)
                    continue;

                meanVel += neighbor.Velocity;
                count++;
            }

            if (count == 0)
                return Vector3.Zero;

            meanVel /= count;

            var delta = meanVel - agent.Velocity;
            return delta * power;
        }


        private static Vector3 AlignSame(State state, Agent agent, FixedBufferList<Agent> nearby, float distanceSqr, float power)
        {
            // point toward the center of the flock (mean flock boid position)
            var meanVel = Vector3.Zero;
            var count = 0;
            for (int i = 0; i < nearby.Count; i++)
            {
                var neighbor = nearby[i];
                if (neighbor.Group != agent.Group)
                    continue;

                var dist = Vector3.DistanceSqr(agent.Position, neighbor.Position);
                if (dist > distanceSqr)
                    continue;

                meanVel += neighbor.Velocity;
                count++;
            }

            if (count == 0)
                return Vector3.Zero;

            meanVel /= count;

            var delta = meanVel - agent.Velocity;
            return delta * power;
        }

        private static Vector3 AlignOther(State state, Agent agent, FixedBufferList<Agent> nearby, float distanceSqr, float power)
        {
            // point toward the center of the flock (mean flock boid position)
            var meanVel = Vector3.Zero;
            var count = 0;
            for (int i = 0; i < nearby.Count; i++)
            {
                var neighbor = nearby[i];
                if (neighbor.Group == agent.Group)
                    continue;

                var dist = Vector3.DistanceSqr(agent.Position, neighbor.Position);
                if (dist > distanceSqr)
                    continue;

                meanVel += neighbor.Velocity;
                count++;
            }

            if (count == 0)
                return Vector3.Zero;

            meanVel /= count;

            var delta = meanVel - agent.Velocity;
            return delta * power;
        }

        private static Vector3 AvoidAny(State state, Agent agent, FixedBufferList<Agent> nearby, float distanceSqr, float power)
        {
            // point away as boids get close
            var sumCloseness = Vector3.Zero;
            for (int i = 0; i < nearby.Count; i++)
            {
                var neighbor = nearby[i];

                var dist = Vector3.DistanceSqr(agent.Position, neighbor.Position);
                if (dist > distanceSqr)
                    continue;

                var closeness = distanceSqr - dist;
                sumCloseness += (agent.Position - neighbor.Position) * closeness;
            }

            return sumCloseness * power;
        }


        private static Vector3 AvoidSame(State state, Agent agent, FixedBufferList<Agent> nearby, float distanceSqr, float power)
        {
            // point away as boids get close
            var sumCloseness = Vector3.Zero;
            for (int i = 0; i < nearby.Count; i++)
            {
                var neighbor = nearby[i];
                if (neighbor.Group != agent.Group)
                    continue;

                var dist = Vector3.DistanceSqr(agent.Position, neighbor.Position);
                if (dist > distanceSqr)
                    continue;

                var closeness = distanceSqr - dist;
                sumCloseness += (agent.Position - neighbor.Position) * closeness;
            }

            return sumCloseness * power;
        }

        private static Vector3 AvoidOther(State state, Agent agent, FixedBufferList<Agent> nearby, float distanceSqr, float power)
        {
            // point away as boids get close
            var sumCloseness = Vector3.Zero;
            for (int i = 0; i < nearby.Count; i++)
            {
                var neighbor = nearby[i];
                if (neighbor.Group == agent.Group)
                    continue;

                var dist = Vector3.DistanceSqr(agent.Position, neighbor.Position);
                if (dist > distanceSqr)
                    continue;

                var closeness = distanceSqr - dist;
                sumCloseness += (agent.Position - neighbor.Position) * closeness;
            }

            return sumCloseness * power;
        }

        private static Vector3 Wind(State state, Agent agent, FixedBufferList<Agent> nearby, float distance, float power)
        {
            return state.WindDir * power;
        }

        private static Vector3 WindInverted(State state, Agent agent, FixedBufferList<Agent> nearby, float distance, float power)
        {
            return (state.WindDir * -1.0f) * power;
        }

        private static Vector3 StickToRoads(State state, Agent agent, FixedBufferList<Agent> nearby, float distance, float power)
        {
            if (state.MapData == null)
            {
                return Vector3.Zero;
            }

            var roads = state.MapData.Roads;
            if (roads == null)
            {
                return Vector3.Zero;
            }

            var pos = agent.Position;
            var worldMins = state.WorldMins;
            var worldMaxs = state.WorldMaxs;

            // Remap the position to the roads bitmap.
            var x = Math.Remap(pos.X, worldMins.X, worldMaxs.X, 0f, roads.Width);
            var y = Math.Remap(pos.Y, worldMins.Y, worldMaxs.Y, 0f, roads.Height);

            int ix = (int)x;
            int iy = (int)y;

            var closest = roads.GetClosestRoad(ix, iy);
            if (closest.Type == RoadType.None)
            {
                return Vector3.Zero;
            }

            float dx = (float)(closest.X - ix);
            float dy = (float)(closest.Y - iy);

            if (closest.Type == RoadType.Asphalt)
            {
                return new Vector3(dx * 0.75f, dy * 0.75f) * power;
            }
            else if (closest.Type == RoadType.Offroad)
            {
                return new Vector3(dx * 0.5f, dy * 0.5f) * power;
            }

            return Vector3.Zero;
        }

        private static Vector3 AvoidRoads(State state, Agent agent, FixedBufferList<Agent> nearby, float distance, float power)
        {
            if (state.MapData == null)
            {
                return Vector3.Zero;
            }

            var roads = state.MapData.Roads;
            if (roads == null)
            {
                return Vector3.Zero;
            }

            var pos = agent.Position;
            var worldMins = state.WorldMins;
            var worldMaxs = state.WorldMaxs;

            // Remap the position to the roads bitmap.
            var x = Math.Remap(pos.X, worldMins.X, worldMaxs.X, 0f, roads.Width);
            var y = Math.Remap(pos.Y, worldMins.Y, worldMaxs.Y, 0f, roads.Height);

            int ix = (int)x;
            int iy = (int)y;

            var closest = roads.GetClosestRoad(ix, iy);
            if (closest.Type == RoadType.None)
            {
                return Vector3.Zero;
            }

            float dx = (float)(ix - closest.X);
            float dy = (float)(iy - closest.Y);

            if (closest.Type == RoadType.Asphalt)
            {
                return new Vector3(dx * 0.75f, dy * 0.75f) * power;
            }
            else if (closest.Type == RoadType.Offroad)
            {
                return new Vector3(dx * 0.5f, dy * 0.5f) * power;
            }

            return Vector3.Zero;
        }

        private static Vector3 StickToPOIs(State state, Agent agent, FixedBufferList<Agent> nearby, float distance, float power)
        {
            if (state.MapData == null)
            {
                return Vector3.Zero;
            }

            var prefabs = state.MapData.Prefabs;
            var decos = prefabs.Decorations;

            var closestIdx = -1;
            var closestDist = float.MaxValue;
            for (int i = 0; i < decos.Length; i++)
            {
                var deco = decos[i];

                if (state.AgentsNearPOICounter != null)
                {
                    if (state.AgentsNearPOICounter[i] >= 64)
                        continue;
                }

                var dist = Vector3.DistanceSqr(agent.Position, deco.Position);

                if (dist < closestDist)
                {
                    closestDist = dist;
                    closestIdx = i;
                }
            }

            if (closestIdx == -1)
            {
                return Vector3.Zero;
            }

            var direction = Vector3.Normalize(decos[closestIdx].Position - agent.Position);
            return direction * power;
        }


        private static Vector3 AvoidPOIs(State state, Agent agent, FixedBufferList<Agent> nearby, float distance, float power)
        {
            if (state.MapData == null)
            {
                return Vector3.Zero;
            }

            var prefabs = state.MapData.Prefabs;
            var decos = prefabs.Decorations;

            var sumCloseness = Vector3.Zero;
            for (int i = 0; i < decos.Length; i++)
            {
                var deco = decos[i];
                var dist = Vector3.DistanceSqr(agent.Position, deco.Position);
                if (dist > distance)
                {
                    continue;
                }

                var closeness = distance - dist;
                sumCloseness += (agent.Position - deco.Position) * closeness;
            }

            return sumCloseness * power;
        }

        private static Vector3 WorldEvents(State state, Agent agent, FixedBufferList<Agent> nearby, float distance, float power)
        {
            var events = state.EventsTemp;

            Vector3 sum = Vector3.Zero;

            float n = 0;
            for (int i = 0; i < events.Count; i++)
            {
                var ev = events[i];
                if (ev.Type == EventType.Noise)
                {
                    var dist = Vector3.Distance(agent.Position, ev.Position);
                    if (dist > ev.Radius)
                    {
                        continue;
                    }

                    var delta = ev.Position - agent.Position;
                    delta.Z = 0;

                    var closeness = 1.0f - ((dist * 0.7f) / ev.Radius);
                    sum += delta * closeness;
                    n++;
                }
            }

            if (n > 0)
            {
                sum /= n;
            }

            return sum * power;
        }

        private static Vector3 AvoidObstacles(State state, Agent agent, FixedBufferList<Agent> nearby, float distanceSqr, float power)
        {
            if (agent.Velocity == Vector3.Zero)
            {
                //Logging.Info("[ZombieSpawnerMod] Agent {0} is idle (no velocity).", agent.Index);
                return Vector3.Zero;
            }

            var pos = agent.Position;
            var velocityNorm = Vector3.Normalize(agent.Velocity);
            var nextPos = pos + velocityNorm;
            var distToObstacle = Vector3.Distance(pos, nextPos);

            // Log movement data
            //Logging.Info("[ZombieSpawnerMod] Agent {0} - CurrentPos: ({1:F2}, {2:F2}) → NextPos: ({3:F2}, {4:F2})", agent.Index, pos.X, pos.Y, nextPos.X, nextPos.Y);
            //Logging.Info("[ZombieSpawnerMod] Agent {0} - Movement distance: {1:F2}", agent.Index, distToObstacle);

            // Log height data
            var heightHere = ObstacleMap.Instance.GetHeightAt(pos);
            var heightThere = ObstacleMap.Instance.GetHeightAt(nextPos);
            var delta = heightThere - heightHere;

            //Logging.Info("[ZombieSpawnerMod] Agent {0} - Height here: {1:F2}, there: {2:F2}, delta: {3:F2}", agent.Index, heightHere, heightThere, delta);

            // Check for slope too steep
            if (delta > 2.0f)
            {
                //Logging.Info("[ZombieSpawnerMod] Agent {0} - Climb too steep: {1:F2} → {2:F2} (Δ={3:F2}) → avoiding", agent.Index, heightHere, heightThere, delta);

                var avoid = new Vector3(-velocityNorm.Y, velocityNorm.X); // rotate 90°
                //Logging.Info("[ZombieSpawnerMod] Agent {0} - Avoiding direction: ({1:F2}, {2:F2})", agent.Index, avoid.X, avoid.Y);
                return avoid * power;
            }

            // Check for blocked tile
            if (ObstacleMap.Instance.IsBlocked(nextPos))
            {
                //Logging.Info("[ZombieSpawnerMod] Agent {0} - Obstacle detected at ({1:F2}, {2:F2}) → avoiding", agent.Index, nextPos.X, nextPos.Y);

                var avoid = new Vector3(-velocityNorm.Y, velocityNorm.X); // rotate 90°
                //Logging.Info("[ZombieSpawnerMod] Agent {0} - Avoiding direction: ({1:F2}, {2:F2})", agent.Index, avoid.X, avoid.Y);
                return avoid * power;
            }

            //Logging.Info("[ZombieSpawnerMod] Agent {0} - Path is clear.", agent.Index);
            return Vector3.Zero;
        }
    }
}
