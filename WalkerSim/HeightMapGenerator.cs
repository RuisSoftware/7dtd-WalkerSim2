using System;
using System.IO;

namespace WalkerSim
{
    internal static class HeightMapGenerator
    {
        public static float[,] LoadHeightMap(string filePath, int width, int height)
        {
            var data = File.ReadAllBytes(filePath);
            var map = new float[width, height];

            int expectedSize = width * height * 2;
            if (data.Length != expectedSize)
            {
                throw new Exception($"Heightmap file size does not match expected dimensions. Length: {data.Length}. Expected size: {expectedSize}");
            }

            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    int index = (y * width + x) * 2;
                    ushort value = BitConverter.ToUInt16(data, index);
                    map[x, y] = value / 65535f * 10f; // schaal naar max 10m
                }
            }

            return map;
        }

        public static void GenerateObstacleMap(string filePath, int width, int height, Vector3 worldMins)
        {
            var heightMap = LoadHeightMap(filePath, width, height);

            ObstacleMap.Instance.SetHeightMap(heightMap);

            ObstacleMap.Instance.Generate(new Vector3(width, height), worldMins, (pos) =>
            {
                int x = (int)(pos.X - worldMins.X);
                int y = (int)(pos.Y - worldMins.Y);

                if (x < 0 || y < 0 || x >= width || y >= height)
                    return true;

                var heightValue = heightMap[x, y];

                // Blokkeer alles boven bv. 2 meter (pas aan zoals je wil)
                return heightValue >= 2.0f;
            });

            Logging.Out("Obstacle map gegenereerd vanuit hoogtebestand '{0}'.", filePath);
        }

        public static float[,] LoadHeightMapAutoSize(string filePath)
        {
            var fileSize = new FileInfo(filePath).Length;

            var pixelCount = (int)System.Math.Sqrt(fileSize);
            if (pixelCount * pixelCount != fileSize)
                throw new Exception($"Heightmap file size does not match expected square dimensions. Length: {fileSize}. Computed: {pixelCount}x{pixelCount}");

            return LoadHeightMap(filePath, pixelCount, pixelCount);
        }
    }
}
