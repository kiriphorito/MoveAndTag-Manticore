using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;

namespace SW4
{
    public class InitialFilesGenerator
    {
        public void createMaps()
        {
            var reader = new StreamReader(Environment.CurrentDirectory + "/position.txt");
            string positions = reader.ReadToEnd();
            Console.WriteLine(Environment.CurrentDirectory + "position.txt");
            List<string> lines = positions.Split('\n').ToList();

            List<string> mapRobots = new List<string>();
            List<List<string>> mapPolygons = new List<List<string>>();

            foreach (string line in lines)
            {
                int counter = 0;
                string robot = "";
                for (counter = 0; counter < line.Length; counter++)
                {
                    if (line[counter] == '#')
                    {
                        break;
                    }
                    robot += line[counter];
                }
                mapRobots.Add(robot);
                List<string> polygons = new List<string>();
                var polygon = "";
                for (int x = counter + 1; x < line.Length; x++)
                {
                    if (line[x] == ';')
                    {
                        polygons.Add(polygon);
                        polygon = "";
                        continue;
                    }
                    polygon += line[x];
                }
                if (polygon != "")
                {
                    polygons.Add(polygon);
                }
                mapPolygons.Add(polygons);
            }

            for (int x = 1; x <= lines.Count; x++)
            {
                List<string> content = new List<string>();
                content.Add("Robots");
                content.Add(mapRobots[x - 1]);
                content.Add("");
                content.Add("Polygons");
                string polygonContent = "[";
                foreach (string polygon in mapPolygons[x - 1])
                {
                    polygonContent += "[" + polygon + "],";
                }
                if (mapPolygons[x - 1].Count != 0)
                    polygonContent = polygonContent.Remove(polygonContent.Length - 1);
                polygonContent += "]";
                content.Add(polygonContent);
                File.WriteAllLines(x + ".txt", content);
            }
        }
    }
}
