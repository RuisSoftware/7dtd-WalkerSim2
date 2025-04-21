using System;

namespace WalkerSim
{
    internal class ObstacleMap
    {
        public static ObstacleMap Instance = new ObstacleMap();

        private bool[,] map;
        private int width, height;
        public string worldFolder;
        private Vector3 worldMins;
        private float[,] heightMap;

        public void SetHeightMap(float[,] map) => heightMap = map;

        public void Generate(Vector3 worldSize, Vector3 worldMins, Func<Vector3, bool> isBlockedFn)
        {
            width = (int)worldSize.X;
            height = (int)worldSize.Y;
            map = new bool[width, height];
            this.worldMins = worldMins;

            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    var pos = new Vector3(worldMins.X + x, worldMins.Y + y);
                    map[x, y] = isBlockedFn(pos); // vul dit met je logica, zie stap 5
                }
            }
        }

        public bool IsBlocked(Vector3 pos)
        {
            int x = (int)pos.X;
            int y = (int)pos.Y;
            if (x < 0 || y < 0 || x >= width || y >= height)
                return true;
            return map[x, y];
        }

        public void SetWorldFolderLocation(string worldFolderLocation) { worldFolder = worldFolderLocation; }

        public float GetHeightAt(Vector3 pos)
        {
            int x = (int)(pos.X - worldMins.X);
            int y = (int)(pos.Y - worldMins.Y);
            if (x < 0 || y < 0 || x >= width || y >= height)
                return 0f;
            return heightMap[x, y];
        }
    }
}
